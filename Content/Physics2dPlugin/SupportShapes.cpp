#include "Physics2dPluginPrecompiled.hpp"

Real2 Face2d::GetNormal() const
{
  Real2 edge = mPoint1 - mPoint0;
  // Assuming counter clockwise
  Real2 normal = Real2(-edge.y, edge.x);
  return Math::Normalized(normal);
}

SphereSupportShape::SphereSupportShape(SphereCollider2d* collider)
{
  mPosition = collider->mCollider->mWorldTranslation;
  mRadius = collider->mRadius;
}

Real2 SphereSupportShape::GetCenter() const
{
  return mPosition;
}

Real2 SphereSupportShape::Support(Real2Param dir) const
{
  Real2 normalizedDir = Math::AttemptNormalized(dir);
  return mPosition + normalizedDir * mRadius;
}

BoxSupportShape::BoxSupportShape(BoxCollider2d* collider)
{
  mCollider = collider;
  mRotation.Rotate(mCollider->mCollider->mWorldRotation);
  mInvRotation.Rotate(-mCollider->mCollider->mWorldRotation);
}

Real2 BoxSupportShape::GetCenter() const
{
  return mCollider->mCollider->mWorldTranslation;
}

Real2 BoxSupportShape::Support(Real2Param dir) const
{
  Real2 localDir = Math::Multiply(mInvRotation, dir);
  Real2 radius = Real2(0.5f);
  Real2 localPoint = Real2::cZero;
  for (size_t i = 0; i < 2; ++i)
    localPoint[i] += Math::Sign(localDir[i]) * radius[i];

  Real2 worldPoint = Math::Multiply(mRotation, localPoint) + mCollider->mCollider->mWorldTranslation;
  return worldPoint;
}

void BoxSupportShape::GetFaceNormals(Array<Real2>& axes)
{
  axes.PushBack(mRotation.GetBasis(0));
  axes.PushBack(mRotation.GetBasis(1));
}

void BoxSupportShape::GetFaces(Array<Face2d>& faces)
{
  Real2 center = mCollider->mCollider->mWorldTranslation;
  Real2 vertices[4];
  vertices[0] = Math::Multiply(mRotation, Real2(0.5f, 0.5f)) + center;
  vertices[1] = Math::Multiply(mRotation, Real2(0.5f, -0.5f)) + center;
  vertices[2] = Math::Multiply(mRotation, Real2(-0.5f, -0.5f)) + center;
  vertices[3] = Math::Multiply(mRotation, Real2(-0.5f, 0.5f)) + center;

  for (int i = 0; i < 4; ++i)
    faces.PushBack(Face2d(vertices[i], vertices[(i + 1) % 4]));
}
