#pragma once

namespace IntersectionType
{
  enum Enum
  {
    Coplanar, Inside, Outside, Overlap, None
  };
}

inline Real2 Cross2d(const Real& lhs, const Real2& rhs)
{
  return Real2(-lhs * rhs.y, lhs * rhs.x);
}

inline Real2 Cross2d(const Real2& lhs, const Real& rhs)
{
  return Real2(rhs * lhs.y, -rhs * lhs.x);
}

inline Real Cross2d(const Real2& lhs, const Real2& rhs)
{
  return lhs.x * rhs.y - lhs.y * rhs.x;
}

bool BarycentricCoordinates(const Real2& point, const Real2& a, const Real2& b,
  Real& u, Real& v, Real expansionEpsilon = 0.0f);
bool BarycentricCoordinates(const Real2& point, const Real2& a, const Real2& b, const Real2& c,
  Real& u, Real& v, Real& w, Real expansionEpsilon = 0.0f);

IntersectionType::Enum PointPlane(const Real2& point, const Real2& planePoint, const Real2& normal, Real epsilon = 0.001f);

bool RayAabb(const Ray2d& ray, const Aabb2d& aabb, Real2& tResults);
bool RayAabb(const Ray2d& ray, const Aabb2d& aabb, Real& t);
bool AabbAabb(const Aabb2d& aabb0, const Aabb2d& aabb1);
