#include <Fluid/FluidSolver.h>
#include <gtx/norm.hpp>

void FluidSolver::AddSolid(Implicit *impl) {
    mSolids.insert(impl);
    std::cerr << "'" << impl->GetName() << "' added as a solid" << std::endl;
}

void FluidSolver::AddFluid(LevelSet *LS) {
    if (mFluids.find(LS) != mFluids.end()) {
        std::cerr << "'" << LS->GetName() << "' already added as a fluid" << std::endl;
        return;
    }

    mFluids.insert(LS);
    std::cerr << "'" << LS->GetName() << "' added as a fluid" << std::endl;

    // Update the bounding box (in world coordinates)
    mBox = Bbox::BoxUnion(mBox, LS->GetBoundingBox());

    // Compute size of the grid (given dx)
    glm::vec3 extent = mBox.pMax - mBox.pMin;
    int dimX = (int)Round(extent[0] / mDx) + 1;
    int dimY = (int)Round(extent[1] / mDx) + 1;
    int dimZ = (int)Round(extent[2] / mDx) + 1;

    std::cerr << "Creating fluid grid of size " << dimX << " x " << dimY << " x " << dimZ
              << std::endl;

    // Create new velocity, voxel grid and solid mask
    mVelocityField = Volume<glm::vec3>(dimX, dimY, dimZ);
    mVoxels = Volume<float>(dimX, dimY, dimZ);
    mSolidMask = Volume<bool>(dimX, dimY, dimZ);

    // Add the volume
    mInitialVolume += LS->ComputeVolume(LS->GetDx());
}

glm::vec3 FluidSolver::GetValue(float x, float y, float z) const {
    // Transform to grid coordinates
    x = (x - mBox.pMin[0]) / mDx;
    y = (y - mBox.pMin[1]) / mDx;
    z = (z - mBox.pMin[2]) / mDx;

    return mVelocityField.GetValue(x, y, z);
}

glm::vec3 FluidSolver::GetMaxValue() const {
    glm::vec3 maxVal(0.0f, 0.0f, 0.0f);
    for (size_t i = 0; i < mVoxels.GetDimX(); i++) {
        for (size_t j = 0; j < mVoxels.GetDimY(); j++) {
            for (size_t k = 0; k < mVoxels.GetDimZ(); k++) {

                if (glm::l1Norm(maxVal) < glm::l1Norm(mVelocityField.GetValue(i, j, k)))
                    maxVal = mVelocityField.GetValue(i, j, k);
            }
        }
    }

    return maxVal;
}

int FluidSolver::Solve(float time) {
    // Propagate the solution until requested time is reached
    int iterations = 0;
    for (float elapsed = 0; elapsed < time;) {

        // Determine timestep for stability
        float dt = ComputeTimestep();
        std::cerr << "Propagating solution with dt = " << dt << std::endl;

        if (dt > time - elapsed) dt = time - elapsed;
        elapsed += dt;

        // Compute current volume
        mCurrentVolume = 0;
        std::set<LevelSet *>::const_iterator iter = mFluids.begin();
        std::set<LevelSet *>::const_iterator iend = mFluids.end();
        while (iter != iend) {
            mCurrentVolume += (*iter)->ComputeVolume((*iter)->GetDx());
            iter++;
        }
        std::cout << "Current volume: " << mCurrentVolume << std::endl;
        std::cout << "Initial volume: " << mInitialVolume << std::endl;
        std::cout << "Loss of volume: " << mInitialVolume - mCurrentVolume << std::endl;

        // Classify all voxels as either solid, fluid or empty
        std::cerr << "Classifying voxels..." << std::endl;
        ClassifyVoxels();

        // Self advection
        std::cerr << "Self advection..." << std::endl;
        SelfAdvection(dt, 6);

        // Add the external forces
        std::cerr << "Adding external forces..." << std::endl;
        ExternalForces(dt);

        // Enforce boundary conditions
        std::cerr << "Boundary conditions..." << std::endl;
        EnforceDirichletBoundaryCondition();

        // Compute the projection for preserving volume
        std::cerr << "Projection..." << std::endl;
        Projection();

        // The projection should not violate the boundary conditions,
        // but it might becacuse of numerical errors (the solution is not
        // exact). Enforce the boundary conditions again to safeguard
        // against this.
        std::cerr << "Boundary conditions..." << std::endl;
        EnforceDirichletBoundaryCondition();

        // Extend the velocites to "air" so the entire level set narrowband
        // is advected by the velocity field
        VelocityExtension();

        std::cerr << "Done with iteration" << std::endl;
        iterations++;
    }

    return iterations;
}

// Compute a stable timestep given the external forces
float FluidSolver::ComputeTimestep() {
    // If there are no external forces the system is unconditionally stable
    if (mExternalForces == NULL) return std::numeric_limits<float>::max();

    // Get the max value from the vector field
    glm::vec3 maxVal = mExternalForces->GetMaxValue();

    return 0.7f * mDx / glm::length(maxVal);
}

float FluidSolver::ComputePotentialEnergy() { return 0.0f; }

float FluidSolver::ComputeKineticEnergy() { return 0.0f; }

// Add the external forces
void FluidSolver::ExternalForces(float dt) {
    if (mExternalForces == NULL) return;

    float x, y, z;
    for (int i = 0; i < mVoxels.GetDimX(); i++) {
        for (int j = 0; j < mVoxels.GetDimY(); j++) {
            for (int k = 0; k < mVoxels.GetDimZ(); k++) {

                if (IsFluid(i, j, k))
                {
                    // If we're in fluid (see FluidSolver::IsFluid()), sample the external
                    // force field (using world coordinates, see
                    // FluidSolver::TransformGridToWorld()) and perform the integration to
                    // update the velocity field (mVelocityField). The simplest possible
                    // integrator is the explicit Euler.
                    TransformGridToWorld(i, j, k, x, y, z);
                    const glm::vec3 F = mExternalForces->GetValue(x, y, z);
                    mVelocityField.SetValue(i, j, k, mVelocityField.GetValue((size_t)i,j,k) + dt * F);
                }
            }
        }
    }
}

// Compute the self advection term
void FluidSolver::SelfAdvection(float dt, int steps) {
    // Copy the current velocity field
    Volume<glm::vec3> velocities = mVelocityField;
    glm::vec3 currVelocityField = { 0.0f, 0.0f, 0.0f };
    glm::vec3 zeroMassParticle = { 0.0f, 0.0f, 0.0f };

    //for (int i = 0; i < mVoxels.GetDimX(); i++) {
    //    for (int j = 0; j < mVoxels.GetDimY(); j++) {
    //        for (int k = 0; k < mVoxels.GetDimZ(); k++) {
    //            
    //            // If we're in fluid, sample the current velocity field at (i,j,k).
    //            // Then, trace a particle at initial position (i,j,k) back in time
    //            // through the velocity field using 'steps' number of steps. Note that
    //            // each step is dt/steps time units long. Note also that the velocities
    //            // are given in world space, but you perform the trace in grid space so
    //            // you need to scale the velocities accordingly (grid spacing: mDx).
    //            // When you trace the particle you interpolate the velocities inbetween
    //            // the grid points, use mVelocityField.GetValue(float i, float j, float
    //            // k) for trilinear interpolation.
    //            if (IsFluid(i, j, k)) {

    //                currVelocityField = velocities.GetValue((size_t)i, j, k);
    //                zeroMassParticle = glm::vec3(i, j, k);

    //                for (int step = 0; step < steps; ++step) {
    //                    zeroMassParticle -= (currVelocityField * (dt / steps)) / mDx;
    //                    currVelocityField = mVelocityField.GetValue(
    //                        zeroMassParticle.x, zeroMassParticle.y, zeroMassParticle.z);
    //                }

    //                velocities.SetValue(i, j, k, currVelocityField);
    //            }
    //           
    //            
    //        }
    //    }
    //}

    // Update the current velocity field
    mVelocityField = velocities;
}

// Enforce the Dirichlet boundary conditions
void FluidSolver::EnforceDirichletBoundaryCondition() {
    for (size_t i = 0; i < mVoxels.GetDimX(); i++) {
        for (size_t j = 0; j < mVoxels.GetDimY(); j++) {
            for (size_t k = 0; k < mVoxels.GetDimZ(); k++) {

                if (IsFluid(i, j, k))
                {
                    // If we're in fluid, check the neighbors of (i,j,k) to
                    // see if it's next to a solid boundary. If so, project
                    // the velocity to the boundary plane by setting the
                    // velocity to zero along the given dimension.
                    glm::vec3 V = mVelocityField.GetValue(i, j, k);
                    if (i == 0 || (IsSolid(i - 1, j, k) && V.x < 0)) {
                        V.x = 0;
                    }
                    else if (i == mVoxels.GetDimX() - 1 || (IsSolid(i + 1, j, k) && V.x > 0)) {
                        V.x = 0;
                    }

                    if (j == 0 || (IsSolid(i, j - 1, k) && V.y < 0)) {
                        V.y = 0;
                    }
                    else if (j == mVoxels.GetDimY() - 1 || (IsSolid(i, j + 1, k) && V.y > 0)) {
                        V.y = 0;
                    }

                    if (k == 0 || (IsSolid(i, j, k - 1) && V.z < 0)) {
                        V.z = 0;
                    }
                    else if (k == mVoxels.GetDimZ() - 1 || (IsSolid(i, j, k + 1) && V.z > 0)) {
                        V.z = 0;
                    }

                    mVelocityField.SetValue(i, j, k, V);
                }
            }
        }
    }
}

// Project the velocity field to preserve the volume
void FluidSolver::Projection() {

    // Compute number of elements in the grid
    auto elements = mVoxels.GetDimX() * mVoxels.GetDimY() * mVoxels.GetDimZ();

    // Create sparse matrix and guess that we have 7 non-zero elements
    // per grid point
    CoordMatrix<float, size_t> A(elements, elements);
    A.reserve(elements * 7);
    A.beginPush();

    // Create vectors x, b in the linear system of equations Ax=b
    std::vector<float> x(elements, 0), b(elements, 0);

    float dx2 = mDx * mDx;

    std::cerr << "Building A matrix and b vector..." << std::endl;
    for (int i = 0; i < mVoxels.GetDimX(); i++) {
        for (int j = 0; j < mVoxels.GetDimY(); j++) {
            for (int k = 0; k < mVoxels.GetDimZ(); k++) {

                // If we're in fluid...
                if (IsFluid(i, j, k)) {

                    // Compute the linear indices of (i,j,k) and its neighbors
                    // (you need these to index into the A matrix and x,b vectors)
                    size_t ind = mVoxels.ComputeLinearIndex(i, j, k);
                    size_t ind_ip = mVoxels.ComputeLinearIndex(i + 1, j, k);
                    size_t ind_im = mVoxels.ComputeLinearIndex(i - 1, j, k);
                    size_t ind_jp = mVoxels.ComputeLinearIndex(i, j + 1, k);
                    size_t ind_jm = mVoxels.ComputeLinearIndex(i, j - 1, k);
                    size_t ind_kp = mVoxels.ComputeLinearIndex(i, j, k + 1);
                    size_t ind_km = mVoxels.ComputeLinearIndex(i, j, k - 1);

                    const float divergence = (
                        mVelocityField.GetValue(static_cast<size_t>(i + 1), j, k).x
                        - mVelocityField.GetValue(static_cast<size_t>(i - 1), j, k).x
                        +
                        mVelocityField.GetValue(static_cast<size_t>(i), j + 1, k).y
                        - mVelocityField.GetValue(static_cast<size_t>(i), j - 1, k).y
                        +
                        mVelocityField.GetValue(static_cast<size_t>(i), j, k + 1).z
                        - mVelocityField.GetValue(static_cast<size_t>(i), j, k - 1).z
                        ) / (2.0f * mDx);
                    b.at(ind) = divergence;

                    // Compute entries for A matrix (discrete Laplacian operator).
                    // The A matrix is a sparse matrix but can be used like a regular
                    // matrix. That is, you access the elements by A(row, column).
                    // However, due to the matrix data structure you cannot read
                    // elements at this point (until you do A.endPush(), see below).
                    // So, only use A(row, column) = ... to set a value in the matrix,
                    // don't use A(row, column) to get a value.
                    // Remember to enforce the boundary conditions if we're next to
                    // a solid (allow no change of flow in that direction).
                    // Remember to treat the boundaries of (i,j,k).
                    int numSolids = 0;

                    if (i + 1 == mVoxels.GetDimX() || IsSolid(i + 1, j, k)) {
                        ++numSolids;
                        A(ind, ind_ip) = 0;
                    } else {
                        A(ind, ind_ip) = 1.0f / dx2;
                    }
                    if (i - 1 == -1 || IsSolid(i - 1, j, k)) {
                        ++numSolids;
                        A(ind, ind_im) = 0;
                    } else {
                        A(ind, ind_im) = 1.0f / dx2;
                    }

                    if (j + 1 == mVoxels.GetDimY() || IsSolid(i, j + 1, k)) {
                        ++numSolids;
                        A(ind, ind_jp) = 0;
                    } else {
                        A(ind, ind_jp) = 1.0f / dx2;
                    }
                    if (IsSolid(i, j - 1, k) || j - 1 == -1) {
                        ++numSolids;
                        A(ind, ind_jm) = 0;
                    } else {
                        A(ind, ind_jm) = 1.0f / dx2;
                    }


                    if (k + 1 == mVoxels.GetDimZ() || IsSolid(i, j, k + 1)) {
                        ++numSolids;
                        A(ind, ind_kp) = 0;
                    } else {
                        A(ind, ind_kp) = 1.0f / dx2;
                    }
                    if (k - 1 == -1 || IsSolid(i, j, k - 1)) {
                        ++numSolids;
                        A(ind, ind_km) = 0;
                    } else {
                        A(ind, ind_km) = 1.0f / dx2;
                    }

                    A(ind, ind) = (-6 + numSolids) / dx2;
                }
            }
        }
    }

    // Rebuild the sparse matrix structure
    A.endPush();

    // Solve Ax=b using conjugate gradient
    std::cerr << "Conjugate gradient solver... ";
    ConjugateGradient<CoordMatrix<float, size_t>, std::vector<float>, float> CG(100, 1e-3f);
    CG.solve(A, x, b);
    std::cerr << "finished with tolerance " << CG.getTolerance() << " in " << CG.getNumIter()
              << " iterations" << std::endl;

    // Subtract the gradient of x to preserve the volume
    for (int i = 0; i < mVoxels.GetDimX(); i++) {
        for (int j = 0; j < mVoxels.GetDimY(); j++) {
            for (int k = 0; k < mVoxels.GetDimZ(); k++) {

                // If we're in fluid...
                if (IsFluid(i, j, k)) {

                    // Compute the linear indices of (i,j,k) and its neighbors
                    size_t ind_ip = mVoxels.ComputeLinearIndex(i + 1, j, k);
                    size_t ind_im = mVoxels.ComputeLinearIndex(i - 1, j, k);
                    size_t ind_jp = mVoxels.ComputeLinearIndex(i, j + 1, k);
                    size_t ind_jm = mVoxels.ComputeLinearIndex(i, j - 1, k);
                    size_t ind_kp = mVoxels.ComputeLinearIndex(i, j, k + 1);
                    size_t ind_km = mVoxels.ComputeLinearIndex(i, j, k - 1);

                    // Compute the gradient of x at (i,j,k) using central differencing
                    // and subtract this gradient from the velocity field.
                    // Thereby removing divergence - preserving volume.
                    const float dx = (x.at(ind_ip) - x.at(ind_im)) / (2.0f * mDx);
                    const float dy = (x.at(ind_jp) - x.at(ind_jm)) / (2.0f * mDx);
                    const float dz = (x.at(ind_kp) - x.at(ind_km)) / (2.0f * mDx);
                    const glm::vec3 gradient{ dx, dy, dz };
                    const glm::vec3 newValue = mVelocityField.GetValue((float_t)i, j, k) - gradient;
                    mVelocityField.SetValue(i, j, k, newValue);
                }
            }
        }
    }
}

// Extent the velocities to "air"
void FluidSolver::VelocityExtension() {
    auto iterations = 0;
    auto narrowbandWidth = 0.0f;
    auto dt = 0.7f;
    std::set<LevelSet *>::const_iterator iter = mFluids.begin();
    std::set<LevelSet *>::const_iterator iend = mFluids.end();
    while (iter != iend) {
        iterations = std::max(iterations, (int)((*iter)->GetNarrowBandWidth() * 0.5f / dt));
        narrowbandWidth =
            std::max(narrowbandWidth, (*iter)->GetNarrowBandWidth() * (*iter)->GetDx() * 0.5f);
        iter++;
    }

    iterations = std::min(iterations, (int)(mVoxels.GetDimX() / dt));
    iterations = std::min(iterations, (int)(mVoxels.GetDimY() / dt));
    iterations = std::min(iterations, (int)(mVoxels.GetDimZ() / dt));

    std::cerr << "Velocity extension (" << iterations << " iterations)..." << std::endl;
    for (int iter = 0; iter < iterations; iter++) {

        Volume<glm::vec3> velocities = mVelocityField;

        for (size_t i = 0; i < mVoxels.GetDimX(); i++) {
            for (size_t j = 0; j < mVoxels.GetDimY(); j++) {
                for (size_t k = 0; k < mVoxels.GetDimZ(); k++) {

                    const float val = mVoxels.GetValue(i, j, k);

                    // Extend only in air or solid (don't extend beyond narrowband for
                    // efficiency)
                    if (IsFluid(i, j, k) || val >= narrowbandWidth) continue;

                    glm::vec3 normal(mVoxels.GetValue(i + 1, j, k) - mVoxels.GetValue(i - 1, j, k),
                                     mVoxels.GetValue(i, j + 1, k) - mVoxels.GetValue(i, j - 1, k),
                                     mVoxels.GetValue(i, j, k + 1) - mVoxels.GetValue(i, j, k - 1));

                    if (glm::length(normal) > 0) normal = glm::normalize(normal);

                    glm::vec3 v = mVelocityField.GetValue(i, j, k);

                    glm::vec3 ip = mVelocityField.GetValue(i + 1, j, k);
                    glm::vec3 im = mVelocityField.GetValue(i - 1, j, k);

                    glm::vec3 jp = mVelocityField.GetValue(i, j + 1, k);
                    glm::vec3 jm = mVelocityField.GetValue(i, j - 1, k);

                    glm::vec3 kp = mVelocityField.GetValue(i, j, k + 1);
                    glm::vec3 km = mVelocityField.GetValue(i, j, k - 1);

                    glm::vec3 v0 = v - dt * (std::max(0.0f, normal[0]) * (v - im) +
                                             std::min(0.0f, normal[0]) * (ip - v) +
                                             std::max(0.0f, normal[1]) * (v - jm) +
                                             std::min(0.0f, normal[1]) * (jp - v) +
                                             std::max(0.0f, normal[2]) * (v - km) +
                                             std::min(0.0f, normal[2]) * (kp - v));

                    velocities.SetValue(i, j, k, v0);
                }
            }
        }

        mVelocityField = velocities;
    }
}

void FluidSolver::ClassifyVoxels() {
    // Iterate the domain and classify all voxels as either
    // fluid, solid or empty
    for (int i = 0; i < mVoxels.GetDimX(); i++) {
        for (int j = 0; j < mVoxels.GetDimY(); j++) {
            for (int k = 0; k < mVoxels.GetDimZ(); k++) {

                ClassifyVoxel(i, j, k);
            }
        }
    }
}

void FluidSolver::ClassifyVoxel(int i, int j, int k) {
    // Transform grid to world coordinates
    float x, y, z;
    TransformGridToWorld(i, j, k, x, y, z);

    // Check if we're inside a solid and update the solid mask
    bool solid = false;
    auto iterSolid = mSolids.begin();
    auto iendSolid = mSolids.end();
    while (iterSolid != iendSolid && !solid) {
        if ((*iterSolid)->GetValue(x, y, z) <= 0) solid = true;
        iterSolid++;
    }
    mSolidMask.SetValue(i, j, k, solid);

    // Compute the closest distance to all fluids
    auto iterFluid = mFluids.begin();
    auto iendFluid = mFluids.end();
    auto distance = std::numeric_limits<float>::max();
    
    while (iterFluid != iendFluid) {
        distance = std::min(distance, (*iterFluid)->GetValue(x, y, z));
        iterFluid++;
    }
    mVoxels.SetValue(i, j, k, distance);
}

bool FluidSolver::IsSolid(size_t i, size_t j, size_t k) const {
    return mSolidMask.GetValue(i, j, k);
}

bool FluidSolver::IsFluid(size_t i, size_t j, size_t k) const {
    return mVoxels.GetValue(i, j, k) <= 0.5f * mDx;
}