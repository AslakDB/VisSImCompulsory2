#include "AMath.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <glm/glm.hpp>


AMath::AMath()
{
    
}

std::vector<Vertex> AMath::loadPointCloud(const std::string& filename, float radius) {
    std::vector<Vertex> points;
    std::ifstream file(filename);
    std::string line;

    if (!file.is_open()) {
        std::cout << "Failed to open file: " << filename << std::endl;
        return points;
    }

    glm::vec3 minPoint(FLT_MAX);
    glm::vec3 maxPoint(-FLT_MAX);
    bool skipLine = true;

    while (std::getline(file, line)) {
        if (skipLine) {
            skipLine = !skipLine;
            continue;       
        }

        float x, y, z;
        //Supports both space and tab seperated values
        if (sscanf_s(line.c_str(), "%f %f %f", &x, &y, &z) == 3 || sscanf_s(line.c_str(), "%f\t%f\t%f", &x, &y, &z) == 3) {
            float xf = x;
            float yf = z;
            float zf = y;
            Vertex point(glm::vec3(xf, yf, zf), glm::vec3(0.f), glm::vec3(0.f));

            // Update min/max bounds
            minPoint = glm::min(minPoint, point.XYZ);
            maxPoint = glm::max(maxPoint, point.XYZ);

            points.push_back(point);
        }

        //skipLine = !skipLine;
    }

    file.close();

    // Translate points to move the minimum point to the origin
    for (auto &point : points) {
        point.XYZ -= minPoint;
        point.XYZ *= 0.1f;
        point.XYZ.x *= -1;
    }

    return points;
}

glm::vec3 AMath::barycentricCoordinates(glm::vec3 A, glm::vec3 B, glm::vec3 C, glm::vec3 P)
{
    glm::vec3 v0 = B - A, v1 = C - A, v2 = P - A;
    float d00 = glm::dot(v0, v0);
    float d01 = glm::dot(v0, v1);
    float d11 = glm::dot(v1, v1);
    float d20 = glm::dot(v2, v0);
    float d21 = glm::dot(v2, v1);
    float denom = d00 * d11 - d01 * d01;

    float v = (d11 * d20 - d01 * d21) / denom;
    float w = (d00 * d21 - d01 * d20) / denom;
    float u = 1.0f - v - w;

    return glm::vec3(u, v, w);
}

float AMath::calculateHeightUsingBarycentric2(const glm::vec3& A, const glm::vec3& B, const glm::vec3& C,
    const glm::vec3& P)
{
    // Calculate barycentric coordinates for P within the triangle ABC.
    glm::vec3 baryCoords = barycentricCoordinates(A, B, C, P);

    // Interpolate Y cood at P.
    float height = A.y * baryCoords.x + B.y * baryCoords.y + C.y * baryCoords.z;
    return height;
}

glm::vec3 AMath::RandomVec3(float min, float max)
{
    float x = min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min)));
    float y = min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min)));
    float z = min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min)));
    return glm::vec3(x, y, z);
}

glm::vec3 AMath::deCasteljau(std::vector<glm::vec3> points, float t)
{
    for (int i = 0; i < points.size() - 1; i++)
    {
        for (int j = 0; j < points.size() - i - 1; j++)
        {
            points[j] = points[j] * (1 - t)  + points[j + 1] * t;
        }
    }
    return points[0];
}

glm::vec3 AMath::lerp(const glm::vec3 &start, const glm::vec3 &end, float t) {
    return start + t * (end - start);
}



