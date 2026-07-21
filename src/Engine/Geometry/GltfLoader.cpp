#include "Engine/Geometry/GltfLoader.hpp"
#include "Engine/Math/Mat4.hpp"

#include "cgltf.h"
#include "stb_image.h"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string_view>

namespace MatterEngine {

namespace {

// Rotates glTF's Y-up, -Z-forward convention into the engine's Z-up world
// (see ARCHITECTURE.md "Convencoes 3D"). This is a pure rotation (handedness
// preserving), so triangle winding never needs correcting afterward.
Vec3 yUpToZUp(Vec3 v) {
    return { v.x, -v.z, v.y };
}

// Same Y-up -> Z-up change of basis, applied to a rotation instead of a
// point/direction. yUpToZUp(Vec3) is a fixed +90 degree rotation about X (in
// the source space), so converting a rotation quaternion is the matching
// conjugation C*q*C^-1, not the vector formula directly.
Quaternion yUpToZUp(Quaternion q) {
    static const Quaternion ChangeOfBasis =
        Quaternion::fromAxisAngle({ 1.0f, 0.0f, 0.0f }, 1.57079632679489661923f);
    return (ChangeOfBasis * q * ChangeOfBasis.conjugate()).normalized();
}

void skipJsonWhitespace(std::string_view text, std::size_t& i) {
    while (i < text.size() && std::isspace(static_cast<unsigned char>(text[i]))) {
        ++i;
    }
}

// Parses a JSON string literal starting at text[i] == '"' (unescaping the
// common escape sequences) and advances i past the closing quote.
std::string parseJsonString(std::string_view text, std::size_t& i) {
    std::string result;
    if (i >= text.size() || text[i] != '"') {
        return result;
    }
    ++i;
    while (i < text.size() && text[i] != '"') {
        const char c = text[i];
        if (c == '\\' && i + 1 < text.size()) {
            switch (text[i + 1]) {
            case '"': result += '"'; break;
            case '\\': result += '\\'; break;
            case '/': result += '/'; break;
            case 'n': result += '\n'; break;
            case 't': result += '\t'; break;
            case 'r': result += '\r'; break;
            case 'b': result += '\b'; break;
            case 'f': result += '\f'; break;
            default: result += text[i + 1]; break;
            }
            i += 2;
            continue;
        }
        result += c;
        ++i;
    }
    if (i < text.size()) {
        ++i; // closing quote
    }
    return result;
}

// Parses one JSON scalar (string/number/bool/null) into its raw text form -
// numbers and literals pass through unescaped, matching how the rest of the
// engine treats custom properties as plain text until something needs them
// typed.
std::string parseJsonScalarAsText(std::string_view text, std::size_t& i) {
    if (i < text.size() && text[i] == '"') {
        return parseJsonString(text, i);
    }
    const std::size_t start = i;
    while (i < text.size() && text[i] != ',' && text[i] != '}'
        && !std::isspace(static_cast<unsigned char>(text[i]))) {
        ++i;
    }
    return std::string(text.substr(start, i - start));
}

// Skips a nested JSON object/array value wholesale, starting at text[i] ==
// '{' or '['. Only flat scalar extras properties are captured (see
// GltfExtras) - a nested value simply isn't represented in the flattened
// key/value list, though it remains readable via GltfExtras::rawJson.
void skipJsonCompound(std::string_view text, std::size_t& i) {
    const char open = text[i];
    const char close = open == '{' ? '}' : ']';
    int depth = 0;
    bool inString = false;
    for (; i < text.size(); ++i) {
        const char c = text[i];
        if (inString) {
            if (c == '\\') { ++i; continue; }
            if (c == '"') inString = false;
            continue;
        }
        if (c == '"') { inString = true; continue; }
        if (c == open) {
            ++depth;
        } else if (c == close && --depth == 0) {
            ++i;
            break;
        }
    }
}

// Minimal parser for the flat {"key": value, ...} JSON objects Blender
// exports as glTF node/material "extras" - not a general JSON parser (no
// nesting support beyond skipping it), just enough to recover the custom
// property key/value pairs content authors set. Malformed input degrades to
// an empty/partial result rather than throwing, since a bad extras blob
// should never fail an otherwise valid model load.
GltfExtras parseGltfExtras(const cgltf_extras& extras) {
    GltfExtras result;
    if (extras.data == nullptr) {
        return result;
    }
    result.rawJson = extras.data;
    const std::string_view text = result.rawJson;
    std::size_t i = 0;
    skipJsonWhitespace(text, i);
    if (i >= text.size() || text[i] != '{') {
        return result;
    }
    ++i;
    while (true) {
        skipJsonWhitespace(text, i);
        if (i >= text.size() || text[i] == '}') {
            break;
        }
        std::string key = parseJsonString(text, i);
        skipJsonWhitespace(text, i);
        if (i >= text.size() || text[i] != ':') {
            break;
        }
        ++i;
        skipJsonWhitespace(text, i);
        if (i < text.size() && (text[i] == '{' || text[i] == '[')) {
            skipJsonCompound(text, i);
        } else {
            result.values.emplace_back(std::move(key), parseJsonScalarAsText(text, i));
        }
        skipJsonWhitespace(text, i);
        if (i < text.size() && text[i] == ',') {
            ++i;
            continue;
        }
        break;
    }
    return result;
}

Vec3 transformPoint(const Mat4& m, Vec3 p) {
    return {
        m.at(0, 0) * p.x + m.at(0, 1) * p.y + m.at(0, 2) * p.z + m.at(0, 3),
        m.at(1, 0) * p.x + m.at(1, 1) * p.y + m.at(1, 2) * p.z + m.at(1, 3),
        m.at(2, 0) * p.x + m.at(2, 1) * p.y + m.at(2, 2) * p.z + m.at(2, 3)
    };
}

Vec3 transformDirection(const Mat4& m, Vec3 v) {
    return {
        m.at(0, 0) * v.x + m.at(0, 1) * v.y + m.at(0, 2) * v.z,
        m.at(1, 0) * v.x + m.at(1, 1) * v.y + m.at(1, 2) * v.z,
        m.at(2, 0) * v.x + m.at(2, 1) * v.y + m.at(2, 2) * v.z
    };
}

Vec3 multiplyComponents(Vec3 a, Vec3 b) {
    return { a.x * b.x, a.y * b.y, a.z * b.z };
}

// Shared by mesh parts and non-mesh entities: the node's own world-space
// origin and rotation. Position always comes from the fully baked world
// matrix (correct regardless of how translation got there); orientation is
// deliberately NOT decomposed from that same matrix - it's rebuilt by
// hand-multiplying only nodes' own decomposed TRS rotations up the parent
// chain, bailing out (hasOrientation = false) the moment any ancestor (or
// the node itself) has no rotation to read because it uses a raw matrix
// instead of TRS, since decomposing rotation out of an arbitrary matrix that
// might also carry non-uniform scale is exactly the kind of silently-wrong
// shortcut this avoids.
void computeNodeWorldPositionAndOrientation(const cgltf_node& node, const Mat4& worldMatrix,
    Vec3& outPosition, Quaternion& outOrientation, bool& outHasOrientation) {
    outPosition = yUpToZUp(transformPoint(worldMatrix, {}));
    outOrientation = {};
    outHasOrientation = false;

    bool ancestorHasMatrix = node.has_matrix != 0;
    for (const cgltf_node* parent = node.parent;
        parent != nullptr && !ancestorHasMatrix; parent = parent->parent) {
        ancestorHasMatrix = parent->has_matrix != 0;
    }
    if (node.has_rotation && !ancestorHasMatrix) {
        Quaternion rotation {
            node.rotation[0], node.rotation[1], node.rotation[2], node.rotation[3]
        };
        for (const cgltf_node* parent = node.parent; parent != nullptr;
            parent = parent->parent) {
            if (parent->has_rotation) {
                rotation = Quaternion { parent->rotation[0], parent->rotation[1],
                    parent->rotation[2], parent->rotation[3] } * rotation;
            }
        }
        outOrientation = yUpToZUp(rotation.normalized());
        outHasOrientation = true;
    }
}

std::string directoryOf(const std::string& path) {
    const std::size_t slash = path.find_last_of("/\\");
    return slash == std::string::npos ? std::string() : path.substr(0, slash + 1);
}

int imageIndexOf(const cgltf_data& data, const cgltf_image* image) {
    if (image == nullptr) {
        return -1;
    }
    return static_cast<int>(image - data.images);
}

} // namespace

const std::string* GltfExtras::find(std::string_view key) const {
    for (const auto& entry : values) {
        if (entry.first == key) {
            return &entry.second;
        }
    }
    return nullptr;
}

namespace {

void overlayExtras(GltfExtras& destination, const GltfExtras& overlay) {
    for (const auto& entry : overlay.values) {
        auto existing = std::find_if(destination.values.begin(),
            destination.values.end(), [&entry](const auto& candidate) {
                return candidate.first == entry.first;
            });
        if (existing != destination.values.end()) {
            existing->second = entry.second;
        } else {
            destination.values.push_back(entry);
        }
    }
    if (!overlay.rawJson.empty()) destination.rawJson = overlay.rawJson;
}

} // namespace

LoadedGltfModel loadGltfModel(const std::string& path) {
    cgltf_options options {};
    cgltf_data* rawData = nullptr;
    if (cgltf_parse_file(&options, path.c_str(), &rawData) != cgltf_result_success) {
        throw std::runtime_error("Failed to parse glTF file: " + path);
    }
    const auto freeData = [](cgltf_data* data) { cgltf_free(data); };
    std::unique_ptr<cgltf_data, decltype(freeData)> data { rawData, freeData };

    if (cgltf_load_buffers(&options, data.get(), path.c_str()) != cgltf_result_success) {
        throw std::runtime_error("Failed to load glTF buffers for: " + path);
    }

    LoadedGltfModel model;

    // Decode every image up front, index-aligned with data->images so
    // materials can reference them by plain index.
    model.images.resize(data->images_count);
    for (std::size_t i = 0; i < data->images_count; ++i) {
        const cgltf_image& image = data->images[i];
        const std::uint8_t* bytes = nullptr;
        std::size_t byteCount = 0;
        std::vector<std::uint8_t> fileBytes;

        if (image.buffer_view != nullptr) {
            const cgltf_buffer_view& view = *image.buffer_view;
            bytes = static_cast<const std::uint8_t*>(view.buffer->data) + view.offset;
            byteCount = view.size;
        } else if (image.uri != nullptr
            && std::string_view(image.uri).substr(0, 5) != "data:") {
            const std::string imagePath = directoryOf(path) + image.uri;
            std::ifstream file(imagePath,
                std::ios::binary | std::ios::ate);
            if (!file) {
                throw std::runtime_error("Failed to open glTF image file: " + imagePath);
            }
            const std::streamoff fileSize = file.tellg();
            file.seekg(0, std::ios::beg);
            fileBytes.resize(fileSize > 0
                ? static_cast<std::size_t>(fileSize) : 0);
            if (!fileBytes.empty() && !file.read(
                    reinterpret_cast<char*>(fileBytes.data()),
                    static_cast<std::streamsize>(fileBytes.size()))) {
                throw std::runtime_error("Failed to read glTF image file: " + imagePath);
            }
            bytes = fileBytes.data();
            byteCount = fileBytes.size();
        }
        if (bytes == nullptr || byteCount == 0) {
            continue;
        }

        int width = 0;
        int height = 0;
        int sourceChannels = 0;
        stbi_uc* decoded = stbi_load_from_memory(bytes, static_cast<int>(byteCount),
            &width, &height, &sourceChannels, 4);
        if (decoded == nullptr) {
            throw std::runtime_error("Failed to decode an image embedded in: " + path);
        }
        LoadedGltfImage& out = model.images[i];
        out.width = width;
        out.height = height;
        out.rgbaPixels.resize(static_cast<std::size_t>(width) * height * 4);
        std::memcpy(out.rgbaPixels.data(), decoded, out.rgbaPixels.size());
        stbi_image_free(decoded);
    }

    model.materials.resize(data->materials_count);
    for (std::size_t i = 0; i < data->materials_count; ++i) {
        const cgltf_material& material = data->materials[i];
        LoadedGltfMaterial& out = model.materials[i];
        out.name = material.name != nullptr ? material.name : "";
        out.extras = parseGltfExtras(material.extras);
        if (material.has_pbr_metallic_roughness) {
            const cgltf_pbr_metallic_roughness& pbr = material.pbr_metallic_roughness;
            out.baseColor = {
                pbr.base_color_factor[0], pbr.base_color_factor[1], pbr.base_color_factor[2]
            };
            if (pbr.base_color_texture.texture != nullptr) {
                out.imageIndex = imageIndexOf(*data, pbr.base_color_texture.texture->image);
            }
            if (pbr.metallic_roughness_texture.texture != nullptr) {
                out.metallicRoughnessImageIndex = imageIndexOf(*data,
                    pbr.metallic_roughness_texture.texture->image);
            }
            out.metallicFactor = pbr.metallic_factor;
            out.roughnessFactor = pbr.roughness_factor;
        }
    }

    for (std::size_t nodeIndex = 0; nodeIndex < data->nodes_count; ++nodeIndex) {
        const cgltf_node& node = data->nodes[nodeIndex];
        if (node.mesh == nullptr) {
            // Not a mesh - if it carries a name or extras (typically a
            // Blender "empty" marking a gameplay location, like a player
            // spawn point), record it as an entity instead of silently
            // dropping it. A node with neither is almost certainly
            // structural (a pure parent transform) and not worth keeping.
            if ((node.name != nullptr && node.name[0] != '\0') || node.extras.data != nullptr) {
                float rawWorldMatrix[16] {};
                cgltf_node_transform_world(&node, rawWorldMatrix);
                Mat4 worldMatrix;
                std::copy(std::begin(rawWorldMatrix), std::end(rawWorldMatrix), worldMatrix.values.begin());

                LoadedGltfEntity entity;
                entity.name = node.name != nullptr ? node.name : "";
                computeNodeWorldPositionAndOrientation(node, worldMatrix,
                    entity.position, entity.orientation, entity.hasOrientation);
                entity.extras = parseGltfExtras(node.extras);
                model.entities.push_back(std::move(entity));
            }
            continue;
        }
        float rawWorldMatrix[16] {};
        cgltf_node_transform_world(&node, rawWorldMatrix);
        Mat4 worldMatrix;
        std::copy(std::begin(rawWorldMatrix), std::end(rawWorldMatrix), worldMatrix.values.begin());
        const std::string nodeName = node.name != nullptr ? node.name : "";
        const GltfExtras nodeExtras = parseGltfExtras(node.extras);
        Vec3 nodeWorldPosition;
        Quaternion nodeWorldOrientation;
        bool nodeHasOrientation = false;
        computeNodeWorldPositionAndOrientation(node, worldMatrix,
            nodeWorldPosition, nodeWorldOrientation, nodeHasOrientation);
        // Folded into local bounds below (not into worldOrientation, which is
        // rotation-only) so a caller can still rebuild a tight oriented box
        // from local bounds + worldPosition + worldOrientation even when the
        // node carries a non-identity scale.
        const Vec3 nodeScale = node.has_scale
            ? Vec3 { node.scale[0], node.scale[1], node.scale[2] }
            : Vec3 { 1.0f, 1.0f, 1.0f };

        for (std::size_t primitiveIndex = 0; primitiveIndex < node.mesh->primitives_count;
            ++primitiveIndex) {
            const cgltf_primitive& primitive = node.mesh->primitives[primitiveIndex];
            if (primitive.type != cgltf_primitive_type_triangles) {
                continue;
            }

            const cgltf_accessor* positionAccessor = nullptr;
            const cgltf_accessor* normalAccessor = nullptr;
            const cgltf_accessor* uvAccessor = nullptr;
            const cgltf_accessor* colorAccessor = nullptr;
            for (std::size_t a = 0; a < primitive.attributes_count; ++a) {
                const cgltf_attribute& attribute = primitive.attributes[a];
                switch (attribute.type) {
                case cgltf_attribute_type_position:
                    positionAccessor = attribute.data;
                    break;
                case cgltf_attribute_type_normal:
                    normalAccessor = attribute.data;
                    break;
                case cgltf_attribute_type_texcoord:
                    if (attribute.index == 0) {
                        uvAccessor = attribute.data;
                    }
                    break;
                case cgltf_attribute_type_color:
                    colorAccessor = attribute.data;
                    break;
                default:
                    break;
                }
            }
            if (positionAccessor == nullptr) {
                continue;
            }

            LoadedGltfPart part;
            part.materialIndex = primitive.material != nullptr
                ? static_cast<int>(primitive.material - data->materials) : -1;
            part.nodeName = nodeName;
            if (part.materialIndex >= 0) {
                part.extras = model.materials[
                    static_cast<std::size_t>(part.materialIndex)].extras;
            }
            // Propriedades do objeto prevalecem sobre o material, permitindo
            // uma excecao local sem duplicar o material visual no Blender.
            overlayExtras(part.extras, nodeExtras);
            part.worldPosition = nodeWorldPosition;
            part.worldOrientation = nodeWorldOrientation;
            part.hasOrientation = nodeHasOrientation;
            const Vec3 materialTint = part.materialIndex >= 0
                ? model.materials[static_cast<std::size_t>(part.materialIndex)].baseColor
                : Vec3 { 1.0f, 1.0f, 1.0f };

            Vec3 localBoundsMin {
                std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                std::numeric_limits<float>::max()
            };
            Vec3 localBoundsMax {
                std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(),
                std::numeric_limits<float>::lowest()
            };

            const std::size_t vertexCount = positionAccessor->count;
            part.mesh.vertices.resize(vertexCount);
            for (std::size_t v = 0; v < vertexCount; ++v) {
                float raw[4] {};
                cgltf_accessor_read_float(positionAccessor, v, raw, 3);
                const Vec3 worldPosition = transformPoint(worldMatrix, { raw[0], raw[1], raw[2] });
                part.mesh.vertices[v].position = yUpToZUp(worldPosition);

                // Local-frame bounds (node scale applied, same Y-up->Z-up
                // conversion as everything else) - see LoadedGltfPart's own
                // doc comment for why this is kept alongside worldPosition/
                // worldOrientation instead of just the world-space AABB.
                const Vec3 localScaled = {
                    raw[0] * nodeScale.x, raw[1] * nodeScale.y, raw[2] * nodeScale.z
                };
                const Vec3 localZup = yUpToZUp(localScaled);
                localBoundsMin = {
                    std::min(localBoundsMin.x, localZup.x), std::min(localBoundsMin.y, localZup.y),
                    std::min(localBoundsMin.z, localZup.z)
                };
                localBoundsMax = {
                    std::max(localBoundsMax.x, localZup.x), std::max(localBoundsMax.y, localZup.y),
                    std::max(localBoundsMax.z, localZup.z)
                };

                Vec3 localNormal { 0.0f, 0.0f, 1.0f };
                if (normalAccessor != nullptr) {
                    cgltf_accessor_read_float(normalAccessor, v, raw, 3);
                    localNormal = { raw[0], raw[1], raw[2] };
                }
                const Vec3 worldNormal = transformDirection(worldMatrix, localNormal);
                part.mesh.vertices[v].normal = yUpToZUp(worldNormal).normalized();

                if (uvAccessor != nullptr) {
                    cgltf_accessor_read_float(uvAccessor, v, raw, 2);
                    part.mesh.vertices[v].uv = { raw[0], raw[1] };
                }

                Vec3 vertexColor { 1.0f, 1.0f, 1.0f };
                if (colorAccessor != nullptr) {
                    cgltf_accessor_read_float(colorAccessor, v, raw, 4);
                    vertexColor = { raw[0], raw[1], raw[2] };
                }
                part.mesh.vertices[v].color = multiplyComponents(vertexColor, materialTint);
            }
            part.localBoundsMin = localBoundsMin;
            part.localBoundsMax = localBoundsMax;

            if (primitive.indices != nullptr) {
                part.mesh.indices.reserve(primitive.indices->count);
                for (std::size_t idx = 0; idx < primitive.indices->count; ++idx) {
                    part.mesh.indices.push_back(
                        static_cast<std::uint32_t>(cgltf_accessor_read_index(primitive.indices, idx)));
                }
            } else {
                part.mesh.indices.reserve(vertexCount);
                for (std::uint32_t idx = 0; idx < static_cast<std::uint32_t>(vertexCount); ++idx) {
                    part.mesh.indices.push_back(idx);
                }
            }

            recomputeBounds(part.mesh);
            model.parts.push_back(std::move(part));
        }
    }

    if (model.parts.empty()) {
        throw std::runtime_error("glTF file has no triangle meshes: " + path);
    }

    return model;
}

} // namespace MatterEngine
