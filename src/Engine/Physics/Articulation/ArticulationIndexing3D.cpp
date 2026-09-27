#include "Engine/Physics/Articulation/ArticulationIndexing3D.hpp"

namespace MatterEngine {

ArticulationIndexing3D buildArticulationIndexing3D(
    const RagdollProfile3D& profile) {
    ArticulationIndexing3D indexing;
    indexing.jointGeneralizedDof.assign(profile.links.size(),
        { ArticulationIndexing3D::InvalidIndex,
          ArticulationIndexing3D::InvalidIndex,
          ArticulationIndexing3D::InvalidIndex });

    std::uint32_t nextDof = 0;
    for (std::size_t linkIndex = 0; linkIndex < profile.links.size();
            ++linkIndex) {
        // A raiz flutuante nao tem junta de entrada. Um link com parentIndex
        // negativo em qualquer outra posicao e perfil invalido, rejeitado por
        // validateRagdollProfile3D; aqui so nao consumimos DOF por ele.
        if (profile.links[linkIndex].parentIndex < 0) continue;

        const RagdollJointDefinition3D& joint =
            profile.links[linkIndex].inboundJoint;
        for (std::size_t axis = 0; axis < 3; ++axis) {
            if (!joint.axes[axis].enabled) continue;
            indexing.jointGeneralizedDof[linkIndex][axis] =
                ArticulationIndexing3D::RootDofCount + nextDof;
            ++nextDof;
        }
    }

    indexing.jointDofCount = nextDof;
    indexing.generalizedDofCount =
        ArticulationIndexing3D::RootDofCount + nextDof;
    return indexing;
}

} // namespace MatterEngine
