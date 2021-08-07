# TNM079 — Modeling and Animation

[Setup and additional lab info](setup.md)

The content of the labs:

- **Lab 1 — Mesh data structures**

  - The half-edge mesh and how it is used to find neighborhood information
  - Vertex and face normal calculations
  - Surface area and mesh volume calculations
  - Gaussian and mean curvature
  - Genus classification using the Euler-Poincaré formula
  - Computation of the number of shells of a model using flood fill

- **Lab 2 — Mesh decimation**

  - Mesh decimation based on _Surface Simplification Using Quadric Error Metrics_ by Michael Garlandand Paul S. Heckbert
  - Alterations to the error metric to achieve more decimation on surfaces that are not seen
  - Visualization of the error quadrics

- **Lab 3 — Splines and subdivision**

  - Subdivision curves based on B-splines
  - Surface subdivision based on Loop’s subdivision scheme
  - Efficient evaluation of B-splines
  - A simple scheme to preserve edges when refining surfaces

- **Lab 4 — Implicit surfaces and modeling**

  - Implicit surface representations
  - Constructive solid geometry operators
  - Quadric surfaces
  - Discrete gradient operator for implicit surfaces
  - Super-elliptic blending

- **Lab 5 — Level-set methods**

  - Time-dependent level-set representations of surfaces and discretization schemes for resolving them
  - Erosion and dilation
  - Advection from external vector fields
  - The use of mean curvature flow to achieve smoothing
  - Narrow band optimization

- **Lab 6 — Fluid simulation**
  - Fluid simulation using the _Stable Fluids_ method
