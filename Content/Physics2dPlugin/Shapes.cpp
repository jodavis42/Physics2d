#include "Physics2dPluginPrecompiled.hpp"

Aabb2d::Aabb2d()
{
  mMin = Real2(Math::PositiveMax());
  mMax = -mMin;
}

Aabb2d::Aabb2d(Real2Param min, Real2Param max)
{
  mMin = min;
  mMax = max;
}

Aabb2d Aabb2d::BuildFromHalfSize(Real2Param center, Real2Param halfSize)
{
  Aabb2d result;
  result.mMin = center - halfSize;
  result.mMax = center + halfSize;
  return result;
}

Aabb2d Aabb2d::BuildFromSize(Real2Param center, Real2Param size)
{
  return BuildFromHalfSize(center, size / 2.0f);
}

Real2 Aabb2d::GetCenter() const
{
  return (mMax + mMin) / 2.0f;
}

Real2 Aabb2d::GetHalfSize() const 
{
  return GetSize() / 2.0f;
}

Real2 Aabb2d::GetSize() const
{
  return mMax - mMin;
}

void Aabb2d::Expand(Real2Param point)
{
  for (size_t i = 0; i < 2; ++i)
  {
    mMin[i] = Math::Min(point[i], mMin[i]);
    mMax[i] = Math::Max(point[i], mMax[i]);
  }
}

void Aabb2d::Expand(const Aabb2d& aabb)
{
  for (size_t i = 0; i < 2; ++i)
  {
    mMin[i] = Math::Min(aabb.mMin[i], mMin[i]);
    mMax[i] = Math::Max(aabb.mMax[i], mMax[i]);
  }
}

Sphere2d::Sphere2d()
{
  mCenter = Real2::cZero;
  mRadius = 1;
}

Sphere2d::Sphere2d(Real2Param center, Real radius)
{
  mCenter = center;
  mRadius = radius;
}

Ray2d::Ray2d()
{
  mStart = Real2::cZero;
  mDirection = Real2(1, 0);
}

Ray2d::Ray2d(Real2Param start, Real2Param dir)
{
  mStart = start;
  mDirection = dir;
}

Real2 Ray2d::GetPoint(Real t) const
{
  return mStart + mDirection * t;
}
