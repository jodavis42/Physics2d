#pragma once

struct Aabb2d
{
  Aabb2d();
  Aabb2d(Real2Param min, Real2Param max);

  static Aabb2d BuildFromHalfSize(Real2Param center, Real2Param halfSize);
  static Aabb2d BuildFromSize(Real2Param center, Real2Param size);
  
  Real2 GetCenter() const;
  Real2 GetHalfSize() const;
  Real2 GetSize() const;

  void Expand(Real2Param point);
  void Expand(const Aabb2d& aabb);

  Real2 mMin;
  Real2 mMax;
};

struct Sphere2d
{
  Sphere2d();
  Sphere2d(Real2Param center, Real radius);

  Real2 mCenter;
  Real mRadius;
};

struct Ray2d
{
  Ray2d();
  Ray2d(Real2Param start, Real2Param dir);

  Real2 GetPoint(Real t) const;

  Real2 mStart;
  Real2 mDirection;
};