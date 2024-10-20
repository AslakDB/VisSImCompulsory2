#pragma once
#include <iosfwd>
#include <iosfwd>
#include <string>
#include <vector>
#include <vector>
#include <vector>

#include "AABB.h"
#include "AABB.h"
#include "Camera.h"
#include "Vertex.h"

class AMath
{
public:

    AMath();
    
    int pointIndex = 0;
    
    glm::vec3 barycentricCoordinates(glm::vec3 A, glm::vec3 B, glm::vec3 C, glm::vec3 P);

    float calculateHeightUsingBarycentric2(const glm::vec3& A, const glm::vec3& B, const glm::vec3& C, const glm::vec3& P);
    
    glm::vec3 RandomVec3(float min, float max);

    glm::vec3 deCasteljau(std::vector<glm::vec3> points, float t);

    glm::vec3 lerp(const glm::vec3& start, const glm::vec3& end, float t);

    std::vector<Vertex> loadPointCloud(const std::string& filename, float radius);

    
    
    
};
