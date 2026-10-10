#include "Engine/Modeling/Import/GltfImport.hpp"

#include "Engine/Modeling/Mesh/MeshBuilder.hpp"

#include "cgltf.h"

#include <cmath>
#include <memory>

namespace MatterEngine::Modeling {

namespace {

struct Frame {
    Vec3 forward;
    Vec3 left;
    Vec3 up { 0.0f, 1.0f, 0.0f };

    [[nodiscard]] Vec3 toModel(Vec3 v) const { return { dot(v, forward), dot(v, left), dot(v, up) }; }
};

} // namespace

GltfImportResult importGltfMesh(const std::filesystem::path& path, const GltfImportSettings& settings) {
    GltfImportResult result;
    Frame frame;
    frame.forward = safeNormalized(Vec3 { settings.forward.x, 0.0f, settings.forward.z }, { 0.0f, 0.0f, 1.0f });
    frame.left = cross(frame.up, frame.forward);

    cgltf_options options {};
    cgltf_data* raw = nullptr;
    const std::string file = path.string();
    if (cgltf_parse_file(&options, file.c_str(), &raw) != cgltf_result_success || raw == nullptr) {
        result.error = "nao foi possivel ler " + file;
        return result;
    }
    const std::unique_ptr<cgltf_data, decltype(&cgltf_free)> data(raw, cgltf_free);
    if (cgltf_load_buffers(&options, data.get(), file.c_str()) != cgltf_result_success) {
        result.error = "nao foi possivel carregar os dados de " + file;
        return result;
    }

    Mesh& mesh = result.mesh;
    MeshBuilder builder(mesh);
    builder.part(settings.part).root();
    builder.material(settings.material, Palette::Surface);
    for (cgltf_size n = 0; n < data->nodes_count; ++n) {
        const cgltf_node& node = data->nodes[n];
        if (node.mesh == nullptr) continue;
        float world[16];
        cgltf_node_transform_world(&node, world);
        // Matriz coluna a coluna (como no glTF).
        const auto apply = [&](const float* v) {
            return Vec3 { world[0] * v[0] + world[4] * v[1] + world[8] * v[2] + world[12],
                world[1] * v[0] + world[5] * v[1] + world[9] * v[2] + world[13],
                world[2] * v[0] + world[6] * v[1] + world[10] * v[2] + world[14] };
        };
        const float determinant = world[0] * (world[5] * world[10] - world[9] * world[6])
            - world[4] * (world[1] * world[10] - world[9] * world[2]) + world[8] * (world[1] * world[6] - world[5] * world[2]);
        const bool mirrored = determinant < 0.0f;
        for (cgltf_size p = 0; p < node.mesh->primitives_count; ++p) {
            const cgltf_primitive& primitive = node.mesh->primitives[p];
            if (primitive.type != cgltf_primitive_type_triangles) continue;
            const cgltf_accessor* positions = nullptr;
            for (cgltf_size a = 0; a < primitive.attributes_count; ++a)
                if (primitive.attributes[a].type == cgltf_attribute_type_position) positions = primitive.attributes[a].data;
            if (positions == nullptr) continue;
            const std::uint32_t first = static_cast<std::uint32_t>(mesh.positions.size());
            for (cgltf_size v = 0; v < positions->count; ++v) {
                float local[3] {};
                cgltf_accessor_read_float(positions, v, local, 3);
                builder.vertex(frame.toModel(apply(local)));
            }
            const cgltf_size count = primitive.indices != nullptr ? primitive.indices->count : positions->count;
            for (cgltf_size i = 0; i + 2 < count; i += 3) {
                std::uint32_t corner[3];
                for (int k = 0; k < 3; ++k) {
                    const cgltf_size index = i + static_cast<cgltf_size>(k);
                    corner[k] = first
                        + static_cast<std::uint32_t>(primitive.indices != nullptr ? cgltf_accessor_read_index(primitive.indices, index)
                                                                                  : index);
                }
                if (mirrored) builder.triangle(corner[0], corner[2], corner[1]);
                else builder.triangle(corner[0], corner[1], corner[2]);
            }
        }
    }
    if (mesh.triangles.empty()) {
        result.error = file + " nao tem triangulos";
        return result;
    }
    const Box3 box = mesh.bounds();
    const float size = (box.maximum - box.minimum).length();
    builder.weld(std::max(1e-9f, settings.weldTolerance * size), false);
    result.ok = true;
    return result;
}

} // namespace MatterEngine::Modeling
