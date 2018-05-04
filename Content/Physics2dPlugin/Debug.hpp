#pragma once

void DrawPoint(const Real2& point, Real size = 0.05f, Real4Param color = Real4(1));
void DrawRay(const Ray2d& ray, Real t, Real4Param color = Real4(1));
void DrawAabb(const Aabb2d& aabb, Real4Param color = Real4(1));
void DrawObb(const Aabb2d& aabb, Real rotation, Real4Param color = Real4(1));
void DrawSphere(const Sphere2d& sphere, Real4Param color = Real4(1));