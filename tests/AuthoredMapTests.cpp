#include "Engine/Geometry/AuthoredMap3D.hpp"
#include "Engine/Geometry/GltfPhysicsMetadata3D.hpp"
#include "Engine/Modeling/Objects/ObjectRecipe.hpp"
#include "Engine/Objects/ObjectAsset3D.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"
#include "Engine/Render/SceneLightPacking.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace MatterEngine;
static void require(bool ok, const char *reason) {
    if (!ok)
        throw std::runtime_error(reason);
}

int main() try {
    const auto model = loadGltfModel(std::string(MATTERENGINE_ASSETS_DIR) + "/maps/compiled/galpao_01.glb");
    const auto map = parseAuthoredMap3D(model);
    require(map.spawns.size() == 3 && map.fields.size() == 1, "Lost gameplay markers");
    const auto &field = map.fields.front();
    require(field.length == 40 && field.width == 20 && std::abs(field.yaw) < 1e-5f, "Field scale/orientation");
    require(model.lights.size() == 27, "Lost authored lights");
    const auto packed = packSceneLights(model.lights);
    require(packed[0].parameters[3] == 1 && packed[0].colorIntensity[3] == 900, "Candela contract");
    for (std::size_t i = 0; i < 24; ++i) {
        require(model.lights[i].type == LightType3D::Spot && model.lights[i].direction.z < -.999f,
                "Ceiling fixture must emit downward");
        require(model.lights[i].coneOuterDegrees == 120 && model.lights[i].coneInnerDegrees == 90,
                "Authored spot cones lost");
    }
    bool grass = false, emissive = false;
    for (const auto &material : model.materials) {
        if (material.name == "gramado_simples") {
            grass = true;
            require(material.normalImageIndex >= 0 && material.metallicRoughnessImageIndex >= 0,
                    "Missing turf PBR data");
            require(model.images.at(material.normalImageIndex).width == 1024, "Turf normal size");
        }
        if (material.name == "luz_led") {
            emissive = true;
            require(material.emissiveImageIndex >= 0 && material.emissiveFactor.z > 4.9f, "Emission strength");
        }
    }
    require(grass && emissive, "Required materials absent");
    LoadedGltfModel rotated = model;
    for (auto &entity : rotated.entities) {
        if (const auto *type = entity.extras.find("entity_type"); type && *type == "matter_field") {
            entity.orientation = Quaternion{}; // engine +Y = local field +X rotated 90 degrees
            entity.position = {2, 3, 4};
        }
    }
    const auto rotatedField = parseAuthoredMap3D(rotated).fields.front();
    const auto paint = buildFutsalMarkings3D(rotatedField);
    require(std::abs(paint.boundsMin.x - (2 - 10.4f)) < .001f && std::abs(paint.boundsMax.y - (3 + 20.45f)) < .001f,
            "Rotated markings bounds");
    for (const auto &vertex : paint.vertices)
        require(std::abs(vertex.position.z - 4.003f) < 1e-5f, "Paint height/field origin");
    // A malformed numeric property must fail, rather than instantiate an invalid
    // field.
    auto invalid = model;
    for (auto &entity : invalid.entities)
        for (auto &[key, value] : entity.extras.values)
            if (key == "length_m")
                value = "40junk";
    bool rejected = false;
    try {
        (void)parseAuthoredMap3D(invalid);
    } catch (const std::invalid_argument &) {
        rejected = true;
    }
    require(rejected, "Malformed field accepted");
    MaterialLibrary materials;
    PhysicsEngine3D physics;
    PhysicsSceneSettings3D settings;
    settings.workerThreadCount = 1;
    settings.runJobsInline = true;
    auto scene = physics.createScene(settings, materials);
    std::uint64_t id = 1;
    for (const auto &part : model.parts) {
        const auto metadata = parseGltfPhysicsMetadata(part.extras);
        require(materials.find(metadata.materialId) != nullptr, "Unknown collision material");
        require(part.materialIndex >= 0 && static_cast<std::size_t>(part.materialIndex) < model.materials.size(),
                "Material/image index confused");
        PhysicsShape3D shape;
        shape.type = PhysicsShapeType3D::TriangleMesh;
        shape.mesh = physics.cookStaticTriangleMesh(part.mesh, metadata.materialId);
        shape.materialId = metadata.materialId;
        PhysicsBodyDefinition3D body;
        body.entityId = id++;
        body.motionType = PhysicsMotionType3D::Static;
        body.materialId = metadata.materialId;
        (void)scene->createBody(body, std::span(&shape, 1));
    }
    for (const auto &spawn : map.spawns) {
        PhysicsRayHit3D hit;
        require(scene->raycastStatic({spawn.position + Vec3{0, 0, .2f}, {0, 0, -1}}, 4, hit), "Spawn without floor");
        require(hit.materialId == "grass", "Spawn floor lost its grass material");
        require(std::abs(hit.position.z) < .01f, "Spawn must land on the pitch, below the roof");
        require(scene->raycastStatic({spawn.position, {0, 0, 1}}, 20, hit) && hit.position.z > 5,
                "Indoor spawn must remain under its roof");
    }
    for (Vec3 direction :
         {Vec3{1, 0, 0}, Vec3{-1, 0, 0}, Vec3{0, 1, 0}, Vec3{0, -1, 0}, Vec3{0, 0, 1}, Vec3{0, 0, -1}}) {
        PhysicsRayHit3D hit;
        require(scene->raycastStatic({{0, 0, 1.7f}, direction}, 100, hit), "Open indoor shell");
    }
    // A shoe-shaped rigid contact uses the same material pairing as physical feet.
    const auto slideSole = [&](float y, const char *floorMaterial) {
        PhysicsRayHit3D hit;
        require(scene->raycastStatic({{0, y, .5f}, {0, 0, -1}}, 1, hit), "Sole probe has no floor");
        require(hit.materialId == floorMaterial, "Wrong authored floor material");
        PhysicsShape3D shape;
        shape.type = PhysicsShapeType3D::Box;
        shape.halfExtents = {.12f, .06f, .04f};
        shape.materialId = "shoe_sole";
        PhysicsBodyDefinition3D body;
        body.entityId = id++;
        body.materialId = "shoe_sole";
        body.position = {0, y, .041f};
        body.linearVelocity = {2, 0, 0};
        const auto handle = scene->createBody(body, std::span(&shape, 1));
        for (int i = 0; i < 240; ++i)
            scene->simulate(1.0f / 120);
        const auto state = scene->bodyState(handle);
        require(state.position.z > 0 && state.position.z < .1f, "Sole sank or floated above floor");
        scene->destroyBody(handle);
        return state.position.x;
    };
    const float grassSlide = slideSole(0, "grass"), concreteSlide = slideSole(-12.3f, "concrete");
    require(grassSlide > 0 && grassSlide < concreteSlide, "Grass/concrete friction is not reaching physical feet");
    CharacterMotorSettings3D characterSettings;
    scene->createCharacter({0, 0, .05f}, characterSettings);
    CharacterMotorCommand3D command;
    for (int i = 0; i < 120; ++i) {
        scene->moveCharacter(command, characterSettings, 1.0f / 120);
        scene->simulate(1.0f / 120);
    }
    require(scene->characterState().grounded, "Character does not stand on authored grass");
    scene->destroyCharacter();
    std::string error;
    auto recipe =
        Modeling::readObjectRecipe(std::string(MATTERENGINE_ASSETS_DIR) + "/objects/trave_futsal.json", &error);
    require(recipe.has_value(), "Goal recipe missing");
    const auto goal = Modeling::generateObject(*recipe, &error);
    require(goal && goal->net, "Futsal goal generation");
    std::vector<NetHandle3D> nets;
    for (float side : {-1.0f, 1.0f}) {
        const auto q = Quaternion::fromAxisAngle({0, 0, 1}, side < 0 ? 0 : 3.14159265359f);
        const auto p =
            Vec3{side * 20, 0, 0} - q.rotate({goal->size.x * .5f - .08f, 0, 0}) + Vec3{0, 0, goal->size.z * .5f};
        auto body = objectBodyData3D(*goal, materials);
        body.body.motionType = PhysicsMotionType3D::Static;
        body.body.position = p;
        body.body.orientation = q;
        body.body.entityId = id++;
        (void)scene->createBody(body.body, body.shapes);
        nets.push_back(scene->createNet(objectNetDescription3D(*goal, p, q, 0)));
        scene->settleNet(nets.back(), 8);
        const Vec3 inward = q.rotate({1, 0, 0});
        PhysicsRayHit3D hit;
        require(scene->raycastStatic({{side * 18, 1.54f, 1}, inward * -1}, 3, hit), "Goalpost collision placement");
        require(hit.distance < 2.1f, "Ray reached fence instead of post");
    }
    require(nets[0] != nets[1], "Second net replaced first");
    const auto profile = loadRagdollProfile3D(std::string(MATTERENGINE_ASSETS_DIR)
        + "/characters/football_player/FootballPlayerV1.ragdoll.json");
    const auto bodyCollision = [&](Vec3 position, Vec3 force, const char *label) {
        RagdollSpawnDefinition3D spawn;
        spawn.entityId = id++;
        spawn.pelvisPosition = position + Vec3{0, 0, profile.standingRootHeightMeters + .05f};
        spawn.active = false;
        const auto body = scene->createRagdoll(profile, spawn);
        std::vector<float> milliseconds;
        bool lateralContact = false;
        for (int tick = 0; tick < 180; ++tick) {
            if (tick < 60)
                scene->applyRagdollRootForce(body, force, {});
            const auto begin = std::chrono::steady_clock::now();
            scene->simulate(1.0f / 120);
            const auto state = scene->ragdollState(body);
            std::vector<Vec3> positions;
            std::vector<Quaternion> orientations;
            for (const auto &link : state.links) {
                require(std::isfinite(link.position.x + link.position.y + link.position.z), "Collision destabilized body");
                positions.push_back(link.position);
                orientations.push_back(link.orientation);
            }
            const auto contacts = scene->ragdollPoseWorldContacts(body, positions, orientations, .001f);
            for (const auto &contact : contacts)
                lateralContact |= std::abs(contact.normalWorld.z) < .6f;
            for (const auto &contact : state.interactions)
                lateralContact |= contact.pointCount > 0 && std::abs(contact.normalWorld.z) < .6f;
            milliseconds.push_back(std::chrono::duration<float, std::milli>(
                std::chrono::steady_clock::now() - begin).count());
        }
        require(lateralContact, "Body did not touch the wall/goal in collision regression");
        std::sort(milliseconds.begin(), milliseconds.end());
        std::cout << label << ": tick P50 " << milliseconds[90] << ", P95 " << milliseconds[171]
                  << ", max " << milliseconds.back() << " ms\n";
        scene->destroyRagdoll(body);
    };
    bodyCollision({0, -11.15f, 0}, {0, -600, 0}, "Body/fence");
    bodyCollision({18.9f, 1.54f, 0}, {600, 0, 0}, "Body/goal");
    InflatedBallDefinition3D ball;
    ball.entityId = id++;
    ball.position = {0, 0, 1};
    const auto handle = scene->createInflatedBall(ball);
    for (int step = 0; step < 720; ++step)
        scene->simulate(1.0f / 120);
    const auto state = scene->inflatedBallState(handle);
    require(state.position.z > .05f && state.position.z < .5f, "Ball crossed turf collision");
    // Calibrated rolling starts inside the actual exported turf, not on a substitute test floor.
    const float r = ball.profile.radiusMeters;
    scene->setInflatedBallMotion(handle, {-8, 3, r - .004f}, {}, {});
    for (int i = 0; i < 120; ++i)
        scene->simulate(1.0f / 120);
    const auto start = scene->inflatedBallState(handle);
    scene->setInflatedBallMotion(handle, start.position, {3.43f, 0, 0}, {0, 3.43f / r, 0});
    for (int i = 0; i < 7200; ++i) {
        scene->simulate(1.0f / 120);
        if (scene->inflatedBallState(handle).linearVelocity.length() < .02f && i > 30)
            break;
    }
    const float roll = scene->inflatedBallState(handle).position.x - start.position.x;
    require(roll > 4 && roll < 8, "Actual artificial turf roll outside 4..8 m");
    scene->setInflatedBallMotion(handle, {-6, 3, 2 + r}, {}, {});
    bool bounced = false;
    float apex = 0;
    for (int i = 0; i < 480; ++i) {
        scene->simulate(1.0f / 120);
        const auto state = scene->inflatedBallState(handle);
        if (state.linearVelocity.z > .1f)
            bounced = true;
        if (bounced)
            apex = std::max(apex, state.position.z - r);
    }
    require(apex > .6f && apex < 1, "Actual turf bounce outside 0.6..1 m");
    std::cout << "Actual turf: roll " << roll << " m; bounce " << apex << " m; sole slide grass/concrete " << grassSlide
              << "/" << concreteSlide << " m\n";
    for (float side : {-1.0f, 1.0f}) {
        scene->setInflatedBallMotion(handle, {side * 18, 1.54f, 1}, {side * 12, 0, 0}, {});
        bool rebounded = false;
        float farthest = 0, maximumStep = 0;
        for (int tick = 0; tick < 120; ++tick) {
            scene->simulate(1.0f / 120);
            const auto shot = scene->inflatedBallState(handle);
            farthest = std::max(farthest, side * shot.position.x);
            rebounded |= side * shot.linearVelocity.x < -1;
            maximumStep = std::max(maximumStep, scene->diagnostics().totalStepMilliseconds);
        }
        require(rebounded && farthest < 20.1f, "Ball crossed procedural goalpost instead of rebounding");
        std::cout << "Ball/post " << side << ": max tick " << maximumStep << " ms\n";
    }
    for (float side : {-1.0f,1.0f}) {
        scene->setInflatedBallMotion(handle,{side*14,0,.65f},{side*14,0,1.5f},{});
        float farthest=0;
        for (int tick=0;tick<480;++tick) {
            scene->simulate(1.0f/120);
            farthest=std::max(farthest,side*scene->inflatedBallState(handle).position.x);
        }
        require(farthest>20 && farthest<22.2f,"Goal net did not catch ball before outer fence");
        std::cout<<"Goal "<<side<<": shot stopped before x "<<farthest<<" m\n";
    }
    for (const auto net : nets) {
        require(scene->contains(net), "Net lost during simulation");
        for (auto point : scene->netPositions(net))
            require(std::isfinite(point.x + point.y + point.z), "Non-finite goal net");
    }
    std::cout << "Authored maps: markers, PBR, lights, rotated markings, shell, "
                 "two goals/nets and ball passed\n";
    return 0;
} catch (const std::exception &error) {
    std::cerr << "Authored map failure: " << error.what() << '\n';
    return 1;
}
