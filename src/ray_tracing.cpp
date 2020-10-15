#include "ray_tracing.h"
#include "disable_all_warnings.h"
// Suppress warnings in third-party code.
DISABLE_WARNINGS_PUSH()
#include <glm/geometric.hpp>
#include <glm/gtx/component_wise.hpp>
#include <glm/vector_relational.hpp>
DISABLE_WARNINGS_POP()
#include <cmath>
#include <iostream>
#include <limits>

bool pointInTriangle(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& n, const glm::vec3& p)
{
    glm::vec3 v01 = { v1.x - v0.x, v1.y - v0.y, v1.z - v0.z };
    glm::vec3 v02 = { v2.x - v0.x, v2.y - v0.y, v2.z - v0.z };
    float area = glm::length(glm::cross(v01, v02)) / 2;

    glm::vec3 v12 = { v2.x - v1.x, v2.y - v1.y, v2.z - v1.z };
    glm::vec3 v1p = { p.x - v1.x, p.y - v1.y, p.z - v1.z };
    float a = (glm::length(glm::cross(v1p, v12)) / 2) / area;

    glm::vec3 v0p = { p.x - v0.x, p.y - v0.y, p.z - v0.z };
    float b = (glm::length(glm::cross(v0p, v02)) / 2) / area;

    float c = (glm::length(glm::cross(v0p, v01)) / 2) / area;

    if (a < 0 || b < 0 || c < 0 || ((a + b + c) > 1.000001 || (a + b + c) < 0.9999999)) {
        return false;
    }

    return true;
}

bool intersectRayWithPlane(const Plane& plane, Ray& ray)
{
    float t = (plane.D - glm::dot(ray.origin, plane.normal)) / glm::dot(ray.direction, plane.normal);
    if (glm::dot(ray.direction, plane.normal) < 0) {
        ray.t = t;
        return true;
    }
    return false;
}

Plane trianglePlane(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2)
{
    glm::vec3 n = glm::cross((v0 - v2), (v1 - v2)) / glm::length(glm::cross((v0 - v2), (v1 - v2)));
    float D = glm::dot(n, v0);
    return Plane{ D, n };
}

/// Input: the three vertices of the triangle
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithTriangle(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, Ray& ray, HitInfo& hitInfo)
{
    float t = ray.t;
    Plane plane = trianglePlane(v0, v1, v2);
    if (intersectRayWithPlane(plane, ray)) {
        glm::vec3 p = ray.origin + ray.direction * ray.t;
        if (pointInTriangle(v0, v1, v2, plane.normal, p)) {
            return true;
        }
    }
    ray.t = t;
    return false;
}

/// Input: a sphere with the following attributes: sphere.radius, sphere.center
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithShape(const Sphere& sphere, Ray& ray, HitInfo& hitInfo)
{
    float a = pow(ray.direction.x, 2) + pow(ray.direction.y, 2) + pow(ray.direction.z, 2);
    float b = 2 * (ray.direction.x * (ray.origin.x - sphere.center.x) + ray.direction.y * (ray.origin.y - sphere.center.y) + ray.direction.z * (ray.origin.z - sphere.center.z));
    float c = pow(ray.origin.x - sphere.center.x, 2) + pow(ray.origin.y - sphere.center.y, 2) + pow(ray.origin.z - sphere.center.z, 2) - pow(sphere.radius, 2);
    float discriminant = pow(b, 2) - 4 * a * c;
    if (discriminant >= 0) {
        float t0 = fmin((-b - sqrt(discriminant)) / (2 * a), (-b + sqrt(discriminant)) / (2 * a));
        float t1 = fmax((-b - sqrt(discriminant)) / (2 * a), (-b + sqrt(discriminant)) / (2 * a));

        if (2 * sphere.radius > glm::length(sphere.radius)) {
            if (t0 >= 0 && t0 < ray.t) {
                ray.t = t0;
                return true;
            }
        }

        if (t1 >= 0 && t1 < ray.t) {
            ray.t = t1;
            return true;
        }
    }
    return false;
}

/// Input: an axis-aligned bounding box with the following parameters: minimum coordinates box.lower and maximum coordinates box.upper
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithShape(const AxisAlignedBox& box, Ray& ray)
{
    float txmin = (box.lower.x - ray.origin.x) / ray.direction.x;
    float tymin = (box.lower.y - ray.origin.y) / ray.direction.y;
    float tzmin = (box.lower.z - ray.origin.z) / ray.direction.z;
    float txmax = (box.upper.x - ray.origin.x) / ray.direction.x;
    float tymax = (box.upper.y - ray.origin.y) / ray.direction.y;
    float tzmax = (box.upper.z - ray.origin.z) / ray.direction.z;
    float tin = fmax(fmax(fmin(txmin, txmax), fmin(tymin, tymax)), fmin(tzmin, tzmax));
    float tout = fmin(fmin(fmax(txmin, txmax), fmax(tymin, tymax)), fmax(tzmin, tzmax));
    if (tin > tout || tout < 0) {
        return false;
    }
    if (0 > tin) {
        ray.t = fmin(tout, ray.t);
        return true;
    }
    ray.t = fmin(tin, ray.t);
    return true;
}
