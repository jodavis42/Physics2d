#include "Physics2dPluginPrecompiled.hpp"

VoronoiRegion::Enum IdentifyVoronoiRegion(const Real2& q, const Real2& p0,
  size_t& newSize, int newIndices[3],
  Real2& closestPoint, Real2& searchDirection)
{
  closestPoint = p0;
  searchDirection = q - closestPoint;
  newSize = 1;
  newIndices[0] = 0;
  return VoronoiRegion::Point0;
}

VoronoiRegion::Enum IdentifyVoronoiRegion(const Real2& q, const Real2& p0, const Real2& p1,
  size_t& newSize, int newIndices[3],
  Real2& closestPoint, Real2& searchDirection)
{
  float u, v;
  BarycentricCoordinates(q, p0, p1, u, v);

  // Test Region P0
  if (v <= 0)
  {
    newIndices[0] = 0;
    newSize = 1;
    closestPoint = p0;
    searchDirection = q - closestPoint;
    return VoronoiRegion::Point0;
  }
  // Test Region P1
  if (u <= 0)
  {
    newIndices[0] = 1;
    newSize = 1;
    closestPoint = p1;
    searchDirection = q - closestPoint;
    return VoronoiRegion::Point1;
  }

  // Otherwise we're in Region P0P1
  newIndices[0] = 0;
  newIndices[1] = 1;
  newSize = 2;
  closestPoint = u * p0 + v * p1;
  searchDirection = q - closestPoint;

  return VoronoiRegion::Edge01;
}

VoronoiRegion::Enum IdentifyVoronoiRegion(const Real2& q, const Real2& p0, const Real2& p1, const Real2& p2,
  size_t& newSize, int newIndices[3],
  Real2& closestPoint, Real2& searchDirection)
{
  float u01, v01;
  float u02, v02;
  float u12, v12;
  float u012, v012, w012;

  BarycentricCoordinates(q, p0, p1, u01, v01);
  BarycentricCoordinates(q, p0, p2, u02, v02);
  BarycentricCoordinates(q, p1, p2, u12, v12);
  BarycentricCoordinates(q, p0, p1, p2, u012, v012, w012);

  // Test Region P0
  if (v01 <= 0 && v02 <= 0)
  {
    newSize = 1;
    newIndices[0] = 0;
    closestPoint = p0;
    searchDirection = q - closestPoint;
    return VoronoiRegion::Point0;
  }
  // Test Region P1
  if (u01 <= 0 && v12 <= 0)
  {
    newSize = 1;
    newIndices[0] = 1;
    closestPoint = p1;
    searchDirection = q - closestPoint;
    return VoronoiRegion::Point1;
  }
  // Test Region P2
  if (u02 <= 0 && u12 <= 0)
  {
    newSize = 1;
    newIndices[0] = 2;
    closestPoint = p2;
    searchDirection = q - closestPoint;
    return VoronoiRegion::Point2;
  }

  // Test Region P0P1
  if (u01 >= 0 && v01 >= 0 && w012 <= 0)
  {
    newSize = 2;
    newIndices[0] = 0;
    newIndices[1] = 1;
    closestPoint = p0 * u01 + p1 * v01;
    searchDirection = q - closestPoint;
    return VoronoiRegion::Edge01;
  }
  // Test Region P0P2
  if (u02 >= 0 && v02 >= 0 && v012 <= 0)
  {
    newSize = 2;
    newIndices[0] = 0;
    newIndices[1] = 2;
    closestPoint = p0 * u02 + p2 * v02;
    searchDirection = q - closestPoint;
    return VoronoiRegion::Edge02;
  }
  // Test Region P1P2
  if (u12 >= 0 && v12 >= 0 && u012 <= 0)
  {
    newSize = 2;
    newIndices[0] = 1;
    newIndices[1] = 2;
    closestPoint = p1 * u12 + p2 * v12;
    searchDirection = q - closestPoint;
    return VoronoiRegion::Edge12;
  }

  // Otherwise we're in region P0P1P2
  newSize = 3;
  newIndices[0] = 0;
  newIndices[1] = 1;
  newIndices[2] = 2;
  closestPoint = q;
  searchDirection = Real2::cZero;
  return VoronoiRegion::Triangle012;
}
