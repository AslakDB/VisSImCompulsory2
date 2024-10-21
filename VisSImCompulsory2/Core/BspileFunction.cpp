#include "BspileFunction.h"

float BspileFunction::B2(float T, int i, const std::vector<float>& knots)
{
    
    if (knots[i] <= T && T < knots[i + 1]) {
        return (T - knots[i]) / (knots[i + 1] - knots[i]);
    } else if (knots[i + 1] <= T && T < knots[i + 2]) {
        return (knots[i + 2] - T) / (knots[i + 2] - knots[i + 1]);
    } else {
        return 0.0f;
    }
}

glm::vec3 BspileFunction::evaluateBiquadratic(float u, float v, const std::vector<glm::vec3>& controlPoints,
    const std::vector<float>& uKnots, const std::vector<float>& vKnots, int n, int m)
{
    glm::vec3 point(0.0f);

    for (int i = 0; i < n ; ++i) {
        for (int j = 0; j < m ; ++j) {
            float Bu = B2(u , i, uKnots);
            float Bv = B2(v, j, vKnots);
            point += Bu * Bv * controlPoints[i * m  + j ];
        }
    }

    return point;
}

std::vector<glm::vec3> BspileFunction::calculateBspline()
{
    std::vector<glm::vec3> CalculatedPoints;
    
    std::vector<glm::vec3> controlPoints = {
        {1.0f, 0.0f, 0.0f}, {1.0f, 3.0f, 3.0f}, {3.0f, 0.0f, 1.0f},
        {0.0f, 1.0f, 2.0f}, {2.0f, 3.0f, 2.0f}, {1.0f, 1.0f, 1.0f},
        {0.0f, 2.0f, 0.0f}, {2.0f, 2.0f, 0.0f}, {3.0f, 2.0f, 1.0f} 
          //{1.0f, 2.0f, 0.0f}, {2.0f, 2.0f, 0.0f}, {3.0f, 2.0f, 1.0f} ,{3.0f, 3.0f, 0.0f}
    };
    
    
   // Define knot vectors
    std::vector<float> uKnots = {0, 0, 0, 1, 3, 1, 3};
    std::vector<float> vKnots = {0, 0, 0, 1, 3, 1, 3};
    glm::vec3 point(0.0f);
    // Evaluate the surface at a grid of points
    int resolution = 20;
    for (int i = 0; i <= resolution ; ++i) {
        for (int j = 0; j <= resolution - 1; ++j) {
            float u = (static_cast<float>(i) / static_cast<float>(resolution));
            float v = (static_cast<float>(j) / static_cast<float>(resolution));
            point = evaluateBiquadratic(u, v, controlPoints, uKnots, vKnots, 3, 3);
           CalculatedPoints.emplace_back(point);
        } 
    }
    return CalculatedPoints;
   
}

void BspileFunction::CreateBspline(model& bsplinemodel)
{
    std::vector<glm::vec3> calculatedPoints = calculateBspline() ;
    int resolution = static_cast<int>(sqrt(calculatedPoints.size())) - 1;
    
    for (const auto& point : calculatedPoints) {
        std::cout << "Point : (" << point.x << ", " << point.y << ", " << point.z << ")\n";
        bsplinemodel.vertices.emplace_back(glm::vec3(point.x, point.y, point.z), glm::vec3(0.f), glm::vec3(0.9f));
    }

    for (int i = 0; i < resolution; ++i) {
        for (int j = 0; j < resolution; ++j) {
            int index = i * (resolution + 1) + j;
            bsplinemodel.indices.emplace_back(index, index + 1, index + resolution + 1);
            std::cout << bsplinemodel.indices.back().A << " " << bsplinemodel.indices.back().B << " " << bsplinemodel.indices.back().C << std::endl;

            bsplinemodel.indices.emplace_back(index + 1, index + resolution + 2, index + resolution + 1);
            std::cout << bsplinemodel.indices.back().A << " " << bsplinemodel.indices.back().B << " " << bsplinemodel.indices.back().C << std::endl;
        }
    }

    for (Triangle& index : bsplinemodel.indices) {
        glm::vec3 normal = glm::cross(bsplinemodel.vertices[index.B].XYZ - bsplinemodel.vertices[index.A].XYZ, bsplinemodel.vertices[index.C].XYZ - bsplinemodel.vertices[index.A].XYZ);
        bsplinemodel.vertices[index.A].Normals += glm::normalize(normal);
        bsplinemodel.vertices[index.B].Normals += glm::normalize(normal);
        bsplinemodel.vertices[index.C].Normals += glm::normalize(normal);
    }

    
    bsplinemodel.Bind();
}
