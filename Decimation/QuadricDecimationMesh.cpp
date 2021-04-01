#include "QuadricDecimationMesh.h"

const QuadricDecimationMesh::VisualizationMode QuadricDecimationMesh::QuadricIsoSurfaces =
    NewVisualizationMode("Quadric Iso Surfaces");

void QuadricDecimationMesh::Initialize() {
    // Allocate memory for the quadric array
    size_t numVerts = mVerts.size();
    mQuadrics.reserve(numVerts);
    std::streamsize width = std::cerr.precision();  // store stream precision
    for (size_t i = 0; i < numVerts; i++) {

        // Compute quadric for vertex i here
        mQuadrics.push_back(createQuadricForVert(i));

        // Calculate initial error, should be numerically close to 0

        glm::vec3 v0 = mVerts[i].pos;
        glm::vec4 v(v0[0], v0[1], v0[2], 1);
        auto m = mQuadrics.back();

        // TODO CHECK
        auto error = glm::dot(v, (m * v));
        // std::cerr << std::scientific << std::setprecision(2) << error << " ";
    }
    std::cerr << std::setprecision(width) << std::fixed;  // reset stream precision

    // Run the initialize for the parent class to initialize the edge collapses
    DecimationMesh::Initialize();
}

/*! \lab2 Implement the computeCollapse here */
/*!
 * \param[in,out] collapse The edge collapse object to (re-)compute,
 * DecimationMesh::EdgeCollapse
 */
void QuadricDecimationMesh::computeCollapse(EdgeCollapse* collapse) {
    // Compute collapse->position and collapse->cost here
    // based on the quadrics at the edge endpoints

    const HalfEdge& halfEdge = e(collapse->halfEdge);
    const size_t v1Idx = halfEdge.vert;
    const size_t v2Idx = e(halfEdge.pair).vert;

    const glm::mat4& Q1 = mQuadrics.at(v1Idx);
    const glm::mat4& Q2 = mQuadrics.at(v2Idx);

    const glm::mat4 Q = Q1 + Q2;
    glm::mat4 veryNiceMatrix = Q;
    veryNiceMatrix[0][3] = 0;
    veryNiceMatrix[1][3] = 0;
    veryNiceMatrix[2][3] = 0;
    veryNiceMatrix[3][3] = 1;

    // Invertible?!
    const glm::vec4 v = glm::inverse(veryNiceMatrix) * glm::vec4(0,0,0,1);
    collapse->position = v;
    collapse->cost = glm::dot(v, Q * v);
}

/*! After each edge collapse the vertex properties need to be updated */
void QuadricDecimationMesh::updateVertexProperties(size_t ind) {
    DecimationMesh::updateVertexProperties(ind);
    mQuadrics[ind] = createQuadricForVert(ind);
}

/*!
 * \param[in] indx vertex index, points into HalfEdgeMesh::mVerts
 */
glm::mat4 QuadricDecimationMesh::createQuadricForVert(size_t indx) const {
    glm::mat4 Q({0.0f, 0.0f, 0.0f, 0.0f},
                {0.0f, 0.0f, 0.0f, 0.0f},
                {0.0f, 0.0f, 0.0f, 0.0f},
                {0.0f, 0.0f, 0.0f, 0.0f});

    const auto facesIdx = FindNeighborFaces(indx);
    for (const size_t faceIdx : facesIdx) {
        Q += createQuadricForFace(faceIdx);
    }

    return Q;
}

/*!
 * \param[in] indx face index, points into HalfEdgeMesh::mFaces
 */
glm::mat4 QuadricDecimationMesh::createQuadricForFace(size_t indx) const {

    const glm::vec3& normal = f(indx).normal;
    const glm::vec3& pointOnPlane = v(e(f(indx).edge).vert).pos;
    const float d = -glm::dot(normal, pointOnPlane);
    const glm::vec4 p(normal, d);

    return glm::outerProduct(p, p);
}

void QuadricDecimationMesh::Render() {
    DecimationMesh::Render();

    glEnable(GL_LIGHTING);
    glMatrixMode(GL_MODELVIEW);

    if (mVisualizationMode == QuadricIsoSurfaces) {
        // Apply transform
        glPushMatrix();  // Push modelview matrix onto stack

        // Implement the quadric visualization here
        std::cout << "Quadric visualization not implemented" << std::endl;

        // Restore modelview matrix
        glPopMatrix();
    }
}
