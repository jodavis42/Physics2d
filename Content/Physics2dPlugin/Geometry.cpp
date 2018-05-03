#include "Physics2dPluginPrecompiled.hpp"

bool BarycentricCoordinates(const Real2& point, const Real2& a, const Real2& b,
  Real& u, Real& v, Real expansionEpsilon)
{
  u = v = 0;
  Real2 v0 = point - b;
  Real2 v1 = a - b;

  Real dot01 = Math::Dot(v0, v1);
  Real dot11 = Math::Dot(v1, v1);

  // Check for a zero division
  if (dot11 == 0)
    return false;

  u = dot01 / dot11;
  v = 1 - u;

  if (u >= -expansionEpsilon && u <= Real(1.0) + expansionEpsilon)
    return true;

  return false;
}

bool BarycentricCoordinates(const Real2& point, const Real2& a, const Real2& b, const Real2& c,
  Real& u, Real& v, Real& w, Real expansionEpsilon)
{
  // Compute the vectors
  Real2 v0 = point - c;
  Real2 v1 = a - c;
  Real2 v2 = b - c;

  u = v = w = 0;

  // Compute the dot products
  Real dot01 = Math::Dot(v0, v1);
  Real dot02 = Math::Dot(v0, v2);
  Real dot12 = Math::Dot(v1, v2);
  Real dot11 = Math::Dot(v1, v1);
  Real dot22 = Math::Dot(v2, v2);

  // Compute barycentric coordinates
  float denom = dot11 * dot22 - dot12 * dot12;
  // check for a zero division
  if (denom == float(0.0))
    return false;

  denom = float(1.0) / denom;
  u = (dot01 * dot22 - dot02 * dot12) * denom;
  v = (dot11 * dot02 - dot12 * dot01) * denom;
  w = 1 - u - v;

  if (u >= -expansionEpsilon && v >= -expansionEpsilon && (u + v <= float(1.0) + expansionEpsilon))
    return true;

  return false;
}

IntersectionType::Enum PointPlane(const Real2& point, const Real2& planePoint, const Real2& normal, Real epsilon)
{
  Real distance = Math::Dot(normal, point - planePoint);
  if (distance > epsilon)
    return IntersectionType::Inside;
  else if(distance < -epsilon)
    return IntersectionType::Outside;
  return IntersectionType::Coplanar;
}

bool RayAabb(const Ray2d& ray, const Aabb2d& aabb, Real2& tResults)
{
  Real tFinalMin = 0;
  Real tFinalMax = Math::PositiveMax();

  for (size_t i = 0; i < 2; ++i)
  {
    if (ray.mDirection[i] == 0)
    {
      // Ray is parallel and outside the aabb, return false
      if (ray.mStart[i] < aabb.mMin[i] || aabb.mMax[i] < ray.mStart[i])
        return false;
      // Otherwise it's parallel but we don't know, leave this to other axes
    }

    Real tMin = (aabb.mMin[i] - ray.mStart[i]) / ray.mDirection[i];
    Real tMax = (aabb.mMax[i] - ray.mStart[i]) / ray.mDirection[i];
    if (ray.mDirection[i] < 0)
      Math::Swap(tMin, tMax);

    tFinalMin = Math::Max(tMin, tFinalMin);
    tFinalMax = Math::Min(tMax, tFinalMax);
    if (tMin > tMax)
      return false;
  }

  tResults = Real2(tFinalMin, tFinalMax);
  return true;
}

bool RayAabb(const Ray2d& ray, const Aabb2d& aabb, Real& t)
{
  Real2 tResults;
  bool result = RayAabb(ray, aabb, tResults);
  t = tResults.x;
  return result;
}

bool AabbAabb(const Aabb2d& aabb0, const Aabb2d& aabb1)
{
  for (size_t i = 0; i < 2; ++i)
  {
    if (aabb0.mMin[i] > aabb1.mMax[i])
      return false;
    if (aabb1.mMin[i] > aabb0.mMax[i])
      return false;
  }
  return true;
}
