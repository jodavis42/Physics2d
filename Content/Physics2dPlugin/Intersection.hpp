#pragma once
#include "SupportShapes.hpp"


struct Manifold
{
  struct ContactPoint
  {
    Real2 mNormal;
    Real mPenetration;
    Real2 mPointA;
    Real2 mPointB;
  };

  Collider2d* mColliderA;
  Collider2d* mColliderB;

  ContactPoint mContactPoints[2];
  size_t mContactCount;
};



struct Intersection
{
  static bool Intersect(Collider2d* colliderA, Collider2d* colliderB, Manifold* manifold);

  static bool Intersect(SphereCollider2d* colliderA, SphereCollider2d* colliderB, Manifold* manifold);
  static bool Intersect(SphereCollider2d* sphereCollider, BoxCollider2d* boxCollider, Manifold* manifold);
  static bool Intersect(BoxCollider2d* colliderA, BoxCollider2d* colliderB, Manifold* manifold);

  static bool RunGjk(Collider2d* colliderA, Collider2d* colliderB, Manifold* manifold);
};

