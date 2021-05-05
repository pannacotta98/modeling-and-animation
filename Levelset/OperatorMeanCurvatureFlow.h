/*************************************************************************************************
 *
 * Modeling and animation (TNM079) 2007
 * Code base for lab assignments. Copyright:
 *   Gunnar Johansson (gunnar.johansson@itn.liu.se)
 *   Ken Museth (ken.museth@itn.liu.se)
 *   Michael Bang Nielsen (bang@daimi.au.dk)
 *   Ola Nilsson (ola.nilsson@itn.liu.se)
 *   Andreas Sderstrm (andreas.soderstrom@itn.liu.se)
 *
 *************************************************************************************************/
#pragma once

#include "Levelset/LevelSetOperator.h"

/*! \brief A level set operator that does mean curvature flow.
 *
 * This class implements level set propagation in the normal direction
 * as defined by the mean curvature flow \f$\kappa\f$ in the following PDE
 *
 *  \f[
 *  \dfrac{\partial \phi}{\partial t} + \alpha \kappa|\nabla \phi| = 0
 *  \f]
 */
//! \lab4 Implement mean curvature flow
class OperatorMeanCurvatureFlow : public LevelSetOperator {
protected:
    //! Scaling parameter, affects time step constraint
    float mAlpha;

public:
    OperatorMeanCurvatureFlow(LevelSet *LS, float alpha = .9f)
        : LevelSetOperator(LS), mAlpha(alpha) {}

    virtual float ComputeTimestep() {
        return 0.95f * mLS->GetDx() * mLS->GetDx() / (6.f * mAlpha);
    }

    virtual void Propagate(float time) {
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
    }

    virtual float Evaluate(size_t i, size_t j, size_t k) {
        const float dx = mLS->Diff2Xpm(i, j, k);
        const float dy = mLS->Diff2Ypm(i, j, k);
        const float dz = mLS->Diff2Zpm(i, j, k);

        const float dxx = mLS->Diff2Xpm(i, j, k);
        const float dyy = mLS->Diff2Ypm(i, j, k);
        const float dzz = mLS->Diff2Zpm(i, j, k);

        const float dxy = mLS->Diff2XYpm(i, j, k);
        const float dxz = mLS->Diff2ZXpm(i, j, k);
        const float dyz = mLS->Diff2YZpm(i, j, k);

        float kappa = (dx * dx * (dyy + dzz) - 2.f * dy * dz * dyz) / (2.f * pow(dx * dx + dy * dy + dz * dz, 3.f / 2.f))
            + (dy * dy * (dxx + dzz) - 2.f * dx * dz * dxz) / (2.f * pow(dx * dx + dy * dy + dz * dz, 3.f / 2.f))
            + (dz * dz * (dxx + dyy) - 2.f * dx * dy * dxy) / (2.f * pow(dx * dx + dy * dy + dz * dz, 3.f / 2.f));

        const glm::vec3 gradient{ dx, dy, dz };

        return mAlpha * kappa * glm::length(gradient);
    }
};
