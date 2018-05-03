#pragma once

namespace VoronoiRegion
{
  enum Enum {
    Point0, Point1, Point2, Point3,
    Edge01, Edge02, Edge03, Edge12, Edge13, Edge23,
    Triangle012, Triangle013, Triangle023, Triangle123,
    Tetrahedra0123,
    Unknown
  };
  static const char* Names[] = { "Point0", "Point1", "Point2", "Point3",
    "Edge01", "Edge02", "Edge03", "Edge12", "Edge13", "Edge23",
    "Triangle012", "Triangle013", "Triangle023", "Triangle123",
    "Tetrahedra0123",
    "Unknown" };
}

// Point Test
VoronoiRegion::Enum IdentifyVoronoiRegion(const Real2& q, const Real2& p0,
  size_t& newSize, int newIndices[3],
  Real2& closestPoint, Real2& searchDirection);

// Edge Test
VoronoiRegion::Enum IdentifyVoronoiRegion(const Real2& q, const Real2& p0, const Real2& p1,
  size_t& newSize, int newIndices[3],
  Real2& closestPoint, Real2& searchDirection);

// Triangle Test
VoronoiRegion::Enum IdentifyVoronoiRegion(const Real2& q, const Real2& p0, const Real2& p1, const Real2& p2,
  size_t& newSize, int newIndices[3],
  Real2& closestPoint, Real2& searchDirection);
