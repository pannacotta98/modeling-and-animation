#ifndef _strange_dubdivmesh_
#define _strange_dubdivmesh_

#include "AdaptiveLoopSubdivisionMesh.h"

class StrangeSubdivisionMesh : public AdaptiveLoopSubdivisionMesh {
public:
    virtual void Subdivide() {
        // ....
        AdaptiveLoopSubdivisionMesh::Subdivide();
    }

protected:
    bool Subdividable(size_t fi) {   
        Face& currentFace = f(fi);
        EdgeIterator it = GetEdgeIterator(currentFace.edge);
        Face& f1 = f(it.Pair().GetEdgeFaceIndex());
        Face& f2 = f(it.Pair().Prev().Pair().GetEdgeFaceIndex());
        Face& f3 = f(it.Pair().Prev().Pair().GetEdgeFaceIndex());

        float f1CosAngle = glm::dot(currentFace.normal, f1.normal);
        float f2CosAngle = glm::dot(currentFace.normal, f2.normal);
        float f3CosAngle = glm::dot(currentFace.normal, f3.normal);

        float maxCosAngle = glm::max(glm::max(f1CosAngle, f2CosAngle), f3CosAngle);

        return (maxCosAngle > 0.2);

        //constexpr float eps = 1e-4;
        //constexpr float pointyThreshold = 0.2f;
        //bool f1IsPointyOrFlat = (abs(f1CosAngle - 1) < eps) || f1CosAngle > pointyThreshold;
        //bool f2IsPointyOrFlat = (abs(f2CosAngle - 1) < eps) || f2CosAngle > pointyThreshold;
        //bool f3IsPointyOrFlat = (abs(f3CosAngle - 1) < eps) || f3CosAngle > pointyThreshold;

        //return !(f1IsPointyOrFlat && f2IsPointyOrFlat && f3IsPointyOrFlat) && (maxCosAngle > 0.2);
    }
};

#endif
