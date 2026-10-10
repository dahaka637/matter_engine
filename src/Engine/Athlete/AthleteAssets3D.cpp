#include "Engine/Athlete/AthleteAssets3D.hpp"

#include <algorithm>
#include <exception>
#include <filesystem>

namespace MatterEngine {

std::vector<AnimationClip3D> loadAthleteClipLibrary3D(const std::string& clipDirectory,
    const RagdollProfile3D& profile, std::vector<std::string>* errors) {
    namespace fs = std::filesystem;
    std::vector<AnimationClip3D> clips;
    std::error_code error;
    if (!fs::is_directory(clipDirectory, error)) {
        if (errors != nullptr) errors->push_back("pasta de clipes nao encontrada: " + clipDirectory);
        return clips;
    }
    std::vector<fs::path> files;
    for (fs::directory_iterator iterator(clipDirectory, error), end; !error && iterator != end; iterator.increment(error))
        if (iterator->is_regular_file() && iterator->path().filename().string().ends_with(".matteranim.json"))
            files.push_back(iterator->path());
    std::sort(files.begin(), files.end());
    for (const fs::path& file : files) {
        try {
            AnimationClip3D clip = loadAnimationClip3D(file.string());
            if (clip.targetRigId != profile.id) continue;
            const auto issues = validateAnimationClipForRagdoll3D(clip, profile);
            if (!issues.empty()) {
                if (errors != nullptr)
                    errors->push_back(file.filename().string() + ": " + issues.front().path + ": " + issues.front().message);
                continue;
            }
            clips.push_back(std::move(clip));
        } catch (const std::exception& exception) {
            if (errors != nullptr) errors->push_back(file.filename().string() + ": " + exception.what());
        }
    }
    return clips;
}

CharacterLocomotionAnimations3D athleteLocomotionClips3D(const RagdollCharacter3D& character,
    const std::vector<AnimationClip3D>& clips, int getUpFrontChoice, int getUpBackChoice) {
    CharacterLocomotionAnimations3D roles;
    for (const auto& clip : clips) {
        if (clip.id == character.idleClipId) roles.idle = &clip;
        if (clip.id == character.walkClipId) roles.walk = &clip;
        if (clip.id == character.idleToSprintClipId) roles.idleToSprint = &clip;
        if (clip.id == character.crouchToSprintClipId) roles.crouchToSprint = &clip;
        if (clip.id == character.runForwardArcLeftClipId) roles.runForwardArcLeft = &clip;
        if (clip.id == character.runForwardArcRightClipId) roles.runForwardArcRight = &clip;
        if (clip.id == character.runBackwardArcRightClipId) roles.runBackwardArcRight = &clip;
        if (clip.id == character.walkBackwardClipId) roles.walkBackward = &clip;
        if (clip.id == character.sprintClipId) roles.sprint = &clip;
        if (clip.id == character.sprintBackwardClipId) roles.sprintBackward = &clip;
        if (clip.id == character.strafeLeftClipId) roles.strafeLeft = &clip;
        if (clip.id == character.strafeRightClipId) roles.strafeRight = &clip;
        if (clip.id == character.sprintStrafeLeftClipId) roles.sprintStrafeLeft = &clip;
        if (clip.id == character.sprintStrafeRightClipId) roles.sprintStrafeRight = &clip;
        if (clip.id == character.jumpStandingClipId) roles.jumpStanding = &clip;
        if (clip.id == character.jumpForwardClipId) roles.jumpForward = &clip;
        if (clip.id == character.jumpBackwardClipId) roles.jumpBackward = &clip;
        if (clip.id == character.jumpLeftClipId) roles.jumpLeft = &clip;
        if (clip.id == character.jumpRightClipId) roles.jumpRight = &clip;
        if (clip.id == character.jumpPrepClipId) roles.jumpPrep = &clip;
        if (clip.id == character.standUpBackClipId) roles.standUpBack = &clip;
        if (clip.id == character.standUpFrontClipId) roles.standUpFront = &clip;
        if (clip.id == character.turnLeftClipId) roles.turnLeft = &clip;
        if (clip.id == character.turnRightClipId) roles.turnRight = &clip;
        for (std::size_t k = 0; k < character.tackleClipIds.size() && k < roles.tackles.size(); ++k)
            if (clip.id == character.tackleClipIds[k]) roles.tackles[k] = &clip;
    }
    // O levantar de brucos e o de costas escolhidos no painel (para comparar).
    {
        const char* ids[2] = { "stand_up_front_kimodo", "stand_up_front" };
        const int choice = std::clamp(getUpFrontChoice, 0, 1);
        for (const auto& clip : clips)
            if (clip.id == ids[choice]) roles.standUpFront = &clip;
    }
    {
        const char* ids[2] = { "stand_up_back_kimodo", "stand_up_back" };
        const int choice = std::clamp(getUpBackChoice, 0, 1);
        for (const auto& clip : clips)
            if (clip.id == ids[choice]) roles.standUpBack = &clip;
    }
    return roles;
}

CharacterMotorSettings3D athleteAvatarMotorSettings3D() {
    CharacterMotorSettings3D settings;
    settings.radius = 0.34f;
    settings.standingHeight = 1.83f;
    settings.crouchedHeight = 1.22f;
    // Substituidas pelas velocidades dos clipes em quem chama (corrida leve
    // 3,0 m/s, sprint 7,5 m/s); ficam so como valor antes deles.
    settings.walkSpeed = 3.0f;
    settings.sprintSpeed = 7.5f;
    settings.crouchedSpeed = 0.95f;
    settings.groundAcceleration = 14.0f;
    settings.groundDeceleration = 18.0f;
    settings.airAcceleration = 4.5f;
    settings.jumpSpeed = 4.6f;
    // O toque rapido e um pulo baixo (pedido do usuario, 05/10 noite: a
    // altura pelo tempo segurado): 0,72 da velocidade; segurando, sobe ate o
    // padrao em ~0,35 s e ate 1,16 em 0,55 s.
    settings.jumpTapSeconds = 0.06f;
    settings.jumpFullChargeSeconds = 0.55f;
    settings.jumpTapSpeedScale = 0.72f;
    settings.jumpChargedSpeedScale = 1.16f;
    settings.gravityScale = 1.0f;
    settings.maximumFallSpeed = 48.0f;
    settings.maximumSlopeDegrees = 50.0f;
    // Sobe os degraus da escada do laboratorio (18 cm) e ainda deixa os
    // meios-fios de 20/30 cm como obstaculo de verdade.
    settings.maximumStepHeight = 0.19f;
    settings.skinWidth = 0.018f;
    settings.coyoteTime = 0.10f;
    settings.jumpBufferTime = 0.12f;
    settings.flightSpeed = 12.0f;
    settings.fastFlightSpeed = 28.0f;
    settings.swimSpeed = 3.4f;
    settings.fastSwimSpeed = 5.0f;
    settings.swimAcceleration = 9.0f;
    settings.maximumPushSpeedMetersPerSecond = 2.2f;
    settings.pushSaturationPenetrationMeters = 0.25f;
    return settings;
}

} // namespace MatterEngine
