#include "Physics2dPluginPrecompiled.hpp"

bool Intersection::Intersect(Collider2d* colliderA, Collider2d* colliderB, Manifold* manifold)
{
  Collider2dType::Enum typeA = colliderA->mColliderType;
  Collider2dType::Enum typeB = colliderB->mColliderType;

  if (typeA == Collider2dType::Sphere && typeB == Collider2dType::Sphere)
    return Intersect(colliderA->mSphereCollider, colliderB->mSphereCollider, manifold);
  if (typeA == Collider2dType::Box && typeB == Collider2dType::Box)
    return Intersect(colliderA->mBoxCollider, colliderB->mBoxCollider, manifold);
  if (typeA == Collider2dType::Sphere && typeB == Collider2dType::Box)
    return Intersect(colliderA->mSphereCollider, colliderB->mBoxCollider, manifold);
  if (typeA == Collider2dType::Box&& typeB == Collider2dType::Sphere)
    return Intersect(colliderB->mSphereCollider, colliderA->mBoxCollider, manifold);
  return false;
}

bool Intersection::Intersect(SphereCollider2d* colliderA, SphereCollider2d* colliderB, Manifold* manifold)
{
  Real2 posA = colliderA->mCollider->mWorldTranslation;
  Real2 posB = colliderB->mCollider->mWorldTranslation;
  Real rA = colliderA->mWorldRadius;
  Real rB = colliderB->mWorldRadius;

  Real2 aToB = posB - posA;
  Real distSq = Math::LengthSq(aToB);
  Real radiusSum = rA + rB;
  Real radiusSumSq = radiusSum * radiusSum;
  if (distSq > radiusSumSq)
    return false;

  manifold->mColliderA = colliderA->mCollider;
  manifold->mColliderB = colliderB->mCollider;
  manifold->mContactCount = 1;
  manifold->mContactPoints[0].mPenetration = radiusSum - Math::Sqrt(distSq);
  manifold->mContactPoints[0].mNormal = aToB.AttemptNormalized();
  manifold->mContactPoints[0].mPointA = posA + manifold->mContactPoints[0].mNormal * rA;
  manifold->mContactPoints[0].mPointB = posB - manifold->mContactPoints[0].mNormal * rB;
  return true;
}

bool Intersection::Intersect(SphereCollider2d* sphereCollider, BoxCollider2d* boxCollider, Manifold* manifold)
{
  Real2 boxPos = boxCollider->mCollider->mWorldTranslation;
  Real2 boxRadius = boxCollider->mWorldSize / 2.0f;

  Real2x2 rotation;
  Real2x2 invRotation;
  invRotation.Rotate(-boxCollider->mCollider->mWorldRotation);
  rotation.Rotate(boxCollider->mCollider->mWorldRotation);

  Real2 spherePos = sphereCollider->mCollider->mWorldTranslation;
  Real sphereRadius = sphereCollider->mWorldRadius;

  Real2 localSpherePos = Math::Multiply(invRotation, spherePos - boxPos);
  

  Real2 localClampedPos = localSpherePos;
  for (size_t i = 0; i < 2; ++i)
    localClampedPos[i] = Math::Clamp(localClampedPos[i], -boxRadius[i], boxRadius[i]);

  Real2 clampedPos = Math::Multiply(rotation, localClampedPos) + boxPos;
  Real2 normal = clampedPos - spherePos;
  Real distSq = Math::LengthSq(normal);
  if (distSq > sphereRadius * sphereRadius)
    return false;

  manifold->mColliderA = sphereCollider->mCollider;
  manifold->mColliderB = boxCollider->mCollider;
  manifold->mContactCount = 1;
  manifold->mContactPoints[0].mPenetration = sphereRadius - Math::Sqrt(distSq);
  manifold->mContactPoints[0].mNormal = normal.AttemptNormalized();
  manifold->mContactPoints[0].mPointA = spherePos + manifold->mContactPoints[0].mNormal * sphereRadius;
  manifold->mContactPoints[0].mPointB = clampedPos;
  return true;
}

bool Intersection::Intersect(BoxCollider2d* colliderA, BoxCollider2d* colliderB, Manifold* manifold)
{
  BoxSupportShape supportShapeA(colliderA);
  BoxSupportShape supportShapeB(colliderB);
  bool result = Sat::IntersectPolygons(&supportShapeA, &supportShapeB, manifold);
  if (result)
  {
    manifold->mColliderA = colliderA->mCollider;
    manifold->mColliderB = colliderB->mCollider;
  }

  return result;
}

bool Intersection::RunGjk(Collider2d* colliderA, Collider2d* colliderB, Manifold* manifold)
{
  manifold->mColliderA = colliderA;
  manifold->mColliderB = colliderB;

  SupportShape* shapeA = colliderA->CreateSupportShape();
  SupportShape* shapeB = colliderB->CreateSupportShape();

  Gjk::CsoPoint closestPoints;
  Gjk gjk;
  bool result = gjk.Intersect(shapeA, shapeB, 0.1f, manifold);

  delete shapeA;
  delete shapeB;

  return result;
}
