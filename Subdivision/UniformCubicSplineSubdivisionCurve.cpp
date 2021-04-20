#include <Subdivision/UniformCubicSplineSubdivisionCurve.h>
#include <glm.hpp>
#include <gtc/type_ptr.hpp>

UniformCubicSplineSubdivisionCurve::UniformCubicSplineSubdivisionCurve(
    const std::vector<glm::vec3> &joints, glm::vec3 lineColor, float lineWidth)
    : mCoefficients(joints), mControlPolygon(joints) {
    this->mLineColor = lineColor;
    this->mLineWidth = lineWidth;
}

void UniformCubicSplineSubdivisionCurve::Subdivide() {
    // Allocate space for new coefficients
    std::vector<glm::vec3> newc;

    assert(mCoefficients.size() > 4 && "Need at least 5 points to subdivide");

    // Implement the subdivision scheme for a natural cubic spline here

    newc.push_back(mCoefficients.front());
    for(size_t i = 1; i < mCoefficients.size() - 1; i++) {
        glm::vec3 cPrimei = (1.f * mCoefficients[i - 1] + 6.f * mCoefficients[i] + 1.f * mCoefficients[i + 1]) / 8.f;
        glm::vec3 cPrimeiHalf = (4.f * mCoefficients[i] + 4.f * mCoefficients[i + 1]) / 8.f;
        newc.push_back(cPrimei);
        newc.push_back(cPrimeiHalf);
    }
    newc.push_back(mCoefficients.back());

    // If 'mCoefficients' had size N, how large should 'newc' be? Perform a check
    // here!
    assert(newc.size() == 2*mCoefficients.size() - 2 && "Incorrect number of new coefficients!");

    mCoefficients = newc;
}

void UniformCubicSplineSubdivisionCurve::Render() {
    // Apply transform
    glPushMatrix();  // Push modelview matrix onto stack

    // Convert transform-matrix to format matching GL matrix format
    // Load transform into modelview matrix
    glMultMatrixf(glm::value_ptr(mTransform));

    mControlPolygon.Render();

    // save line point and color states
    glPushAttrib(GL_POINT_BIT | GL_LINE_BIT | GL_CURRENT_BIT);

    // draw segments
    glLineWidth(mLineWidth);
    glColor3fv(glm::value_ptr(mLineColor));
    glBegin(GL_LINE_STRIP);
    // just draw the spline as a series of connected linear segments
    for (size_t i = 0; i < mCoefficients.size(); i++) {
        glVertex3fv(glm::value_ptr(mCoefficients.at(i)));
    }
    glEnd();

    // restore attribs
    glPopAttrib();

    glPopMatrix();

    GLObject::Render();
}
