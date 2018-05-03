#include "Physics2dPluginPrecompiled.hpp"

void DrawPoint(const Real2& point, Real size, Real4Param color)
{
  Zilch::HandleOf<ZeroEngine::DebugSphere> handle = ZilchAllocate(ZeroEngine::DebugSphere);
  handle->SetPosition(Real3(point.x, point.y, 0));
  handle->SetRadius(size);
  handle->SetColor(color);
  handle->SetOnTop(true);
  ZeroEngine::DebugDraw::Add(handle);
}

void DrawRay(const Ray2d& ray, Real t, Real4Param color)
{
  Real3 start = Math::ToVector3(ray.mStart);
  Real3 end = Math::ToVector3(ray.GetPoint(t));
  Zilch::HandleOf<ZeroEngine::DebugLine> handle = ZilchAllocate(ZeroEngine::DebugLine);
  handle->SetStart(start);
  handle->SetEnd(end);
  handle->SetColor(color);
  handle->SetHeadSize(0.1f);
  ZeroEngine::DebugDraw::Add(handle);
}

void DrawAabb(const Aabb2d& aabb, Real4Param color)
{
  Zilch::HandleOf<ZeroEngine::DebugBox> handle = ZilchAllocate(ZeroEngine::DebugBox);
  handle->SetPosition(Math::ToVector3(aabb.GetCenter(), 0));
  handle->SetHalfExtents(aabb.GetHalfSize());
  handle->SetColor(color);
  ZeroEngine::DebugDraw::Add(handle);
}

void DrawSphere(const Sphere2d& sphere, Real4Param color)
{
  Zilch::HandleOf<ZeroEngine::DebugCircle> handle = ZilchAllocate(ZeroEngine::DebugCircle);
  handle->SetPosition(Math::ToVector3(sphere.mCenter));
  handle->SetRadius(sphere.mRadius);
  handle->SetColor(color);
  ZeroEngine::DebugDraw::Add(handle);
}
