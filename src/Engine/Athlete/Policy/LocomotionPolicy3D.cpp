#include "Engine/Athlete/Policy/LocomotionPolicy3D.hpp"

#include "Engine/Athlete/AthleteTelemetry3D.hpp"
#include "Engine/Athlete/Dribble/BodyForecast3D.hpp"
#include "Engine/RL/Policy/PolicyRuntime3D.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <vector>

namespace MatterEngine {

namespace {

std::string hex(std::uint64_t value) {
    char text[17];
    std::snprintf(text, sizeof text, "%016llx", static_cast<unsigned long long>(value));
    return text;
}

// O cabecalho JSON do .mrlpolicy (o que o PolicyRuntime3D nao guarda: o
// contrato, a autoridade e os tetos com que a rede treinou).
nlohmann::json readHeader(const std::filesystem::path& file) {
    std::ifstream in(file, std::ios::binary);
    char magic[8] = {};
    std::uint32_t size = 0;
    if (!in.read(magic, sizeof magic) || !in.read(reinterpret_cast<char*>(&size), sizeof size) || size > (64u << 20))
        throw std::runtime_error("politica da locomocao: arquivo invalido: " + file.string());
    std::string text(size, '\0');
    if (!in.read(text.data(), size)) throw std::runtime_error("politica da locomocao: cabecalho cortado: " + file.string());
    return nlohmann::json::parse(text);
}

} // namespace

struct LocomotionPolicy3D::Impl {
    std::filesystem::path file;
    LocomotionContract3D contract;
    PolicyRuntime3D runtime;
    LocomotionAuthority3D authority;
    std::string run;
    std::uint64_t iteration = 0;
    BodyForecast3D forecast;
    std::vector<float> observation;
    std::vector<float> action;

    Impl(const RagdollProfile3D& profile, const std::filesystem::path& path)
        : file(path), contract(profile), runtime(path) {}
};

LocomotionPolicy3D::LocomotionPolicy3D(const RagdollProfile3D& profile, const std::filesystem::path& file)
    : m_impl(std::make_unique<Impl>(profile, file)) {
    Impl& impl = *m_impl;
    const nlohmann::json header = readHeader(file);
    if (header.value("observationLayout", std::string()) != "locomocao.observacao"
        || header.value("contractHash", std::string()) != hex(impl.contract.hash()))
        throw std::runtime_error("politica da locomocao: de outro contrato (o deste corpo e "
            + hex(impl.contract.hash()) + "): " + file.string());
    if (impl.runtime.observationSize() != impl.contract.observationSize()
        || impl.runtime.actionSize() != impl.contract.actionSize())
        throw std::runtime_error("politica da locomocao: tamanhos diferentes do contrato: " + file.string());
    const std::vector<std::string>& names = impl.runtime.info().observationNames;
    if (!names.empty()) {
        const RLVectorLayout3D& layout = impl.contract.observationLayout();
        for (std::size_t i = 0; i < names.size() && i < layout.channels.size(); ++i)
            if (names[i] != layout.channels[i].name)
                throw std::runtime_error("politica da locomocao: a entrada " + names[i] + " nao e a do contrato");
    }
    const float authority = std::clamp(header.value("authority", 1.0f), 0.0f, 1.0f);
    impl.authority = LocomotionAuthority3D::uniform(authority);
    if (header.contains("caps") && header["caps"].is_object()) {
        const nlohmann::json& c = header["caps"];
        LocomotionCaps3D& caps = impl.authority.caps;
        const auto read = [&](const char* key, float& value) {
            if (c.contains(key)) value = c.at(key).get<float>();
        };
        read("touchdownMeters", caps.touchdownMeters);
        read("swingUpMeters", caps.swingUpMeters);
        read("swingDownMeters", caps.swingDownMeters);
        read("phaseLeadCycles", caps.phaseLeadCycles);
        read("recoveryUrgency", caps.recoveryUrgency);
        read("cadenceOctaves", caps.cadenceOctaves);
        read("strideOctaves", caps.strideOctaves);
        read("leanRadians", caps.leanRadians);
        read("comMeters", caps.comMeters);
        read("pelvisUpMeters", caps.pelvisUpMeters);
        read("pelvisDownMeters", caps.pelvisDownMeters);
        read("contactAcceleration", caps.contactAcceleration);
        read("jointRadians", caps.jointRadians);
        read("gainOctaves", caps.gainOctaves);
    }
    impl.run = header.value("run", std::string());
    impl.iteration = header.value("iteration", std::uint64_t { 0 });
    impl.observation.assign(impl.contract.observationSize(), 0.0f);
    impl.action.assign(impl.contract.actionSize(), 0.0f);
}

LocomotionPolicy3D::~LocomotionPolicy3D() = default;

const std::string& LocomotionPolicy3D::run() const { return m_impl->run; }
std::uint64_t LocomotionPolicy3D::iteration() const { return m_impl->iteration; }
float LocomotionPolicy3D::authority() const { return m_impl->authority.footwork; }
const std::filesystem::path& LocomotionPolicy3D::file() const { return m_impl->file; }

void LocomotionPolicy3D::before(AthleteState3D& athlete, const RagdollState3D& body, const AthleteCommand3D& command,
    float deltaTime) {
    Impl& impl = *m_impl;
    // A decisao: como no treino, a entrada do instante (o corpo publicado no
    // fim do passo anterior) e a acao media da rede, a cada TicksPerDecision
    // ticks.
    if (athlete.policyTo.size() != impl.contract.actionSize()
        || athlete.policyTicks >= LocomotionContract3D::TicksPerDecision) {
        (void)impl.contract.observe(body, command, athleteControlView3D(athlete), athlete.policyHistory, impl.forecast,
            impl.observation);
        impl.runtime.evaluate(impl.observation, impl.action);
        impl.contract.decide(athlete, impl.action);
    }
    impl.contract.follow(athlete, impl.authority, deltaTime);
}

} // namespace MatterEngine
