#pragma once
#include "Model.h"
class BspileFunction
{
public:

     float CalculateQuadSpline(float z0, float z1, float z2, float t) ;
    float B2(float T, int i, const std::vector<float>& knots);
    glm::vec3 evaluateBiquadratic(float u, float v, const std::vector<glm::vec3>& controlPoints, const std::vector<float>& uKnots, const std::vector<float>& vKnots, int n, int m); 
    std::vector<glm::vec3> calculateBspline();
    
    void CreateBspline(model& bsplinemodel);
    
};
