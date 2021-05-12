#pragma once

#include "Levelset/LevelSetOperator.h"
#include "Math/Function3D.h"
#include "gtx/component_wise.hpp"
#include "Util/Stopwatch.h"

/*! \brief A level set operator that does external advection
 *
 * This class implements level set advectionr in an external vector field by the
 * PDE
 *
 *  \f$
 *  \dfrac{\partial \phi}{\partial t} + \mathbf{V}(\mathbf{x})\cdot \nabla \phi
 * = 0 \f$
 */
//! \lab4 Implement advection in external vector field
class OperatorAdvect : public LevelSetOperator {
protected:
    Function3D<glm::vec3> *mVectorField;

public:
    OperatorAdvect(LevelSet *LS, Function3D<glm::vec3> *vf)
        : LevelSetOperator(LS), mVectorField(vf) {}

    virtual float ComputeTimestep() {
        // Compute and return a stable timestep
        const glm::vec3 maxV = mVectorField->GetMaxValue();
        return 0.7f * mLS->GetDx() / glm::compMax(maxV);
    }
   
    virtual void Propagate(float time) {
        Stopwatch stopwatch;
        stopwatch.start();

        // Determine timestep for stability
        float dt = ComputeTimestep();

        // Propagate level set with stable timestep dt
        // until requested time is reached
        for (float elapsed = 0; elapsed < time;) {
            if (dt > time - elapsed) dt = time - elapsed;
            elapsed += dt;

            IntegrateEuler(dt);
            // IntegrateRungeKutta(dt);
        }
        stopwatch.stop();
        std::cout << "Advect time: " << stopwatch.read() << '\n';
    }

    virtual float Evaluate(size_t i, size_t j, size_t k) {
        // Compute the rate of change (dphi/dt)
        float x = i, y = j, z = k;
        mLS->TransformGridToWorld(x, y, z);
        const glm::vec3 V = mVectorField->GetValue(x, y, z);
        const float dx = (V.x > 0) ? mLS->DiffXm(i, j, k) : mLS->DiffXp(i, j, k);
        const float dy = (V.y > 0) ? mLS->DiffYm(i, j, k) : mLS->DiffYp(i, j, k);
        const float dz = (V.z > 0) ? mLS->DiffZm(i, j, k) : mLS->DiffZp(i, j, k);
        const glm::vec3 gradient{ dx, dy, dz };
        return -glm::dot(V, gradient);
    }
};