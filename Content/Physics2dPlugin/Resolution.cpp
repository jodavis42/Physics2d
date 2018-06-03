#include "Physics2dPluginPrecompiled.hpp"

void ObjectProperties::Compute(RigidBody2d* body, Real2Param point)
{
  if (body != nullptr)
  {
    mInvMass = body->mInvMass;
    mInvInertia = body->mInvInertia;
    mLinearVelocity = body->mLinearVelocity;
    mAngularVelocity = body->mAngularVelocity;
    mCenterOfMass = body->mWorldCenterOfMass;
    mR = point - mCenterOfMass;
  }
  else
  {
    mInvMass = 0;
    mInvInertia = 0;
    mLinearVelocity = Real2::cZero;
    mAngularVelocity = 0;
    mCenterOfMass = Real2::cZero;
    mR = Real2::cZero;
  }
}

void ObjectProperties::Compute(Collider2d* collider, Real2Param point)
{
  Compute(collider->mRigidBody, point);
}

Real2 ObjectProperties::ComputePointVelocity()
{
  return mLinearVelocity + Cross2d(mAngularVelocity, mR);
}

void ObjectProperties::ApplyImpulse(Real2Param impulse)
{
  mLinearVelocity += impulse * mInvMass;
  mAngularVelocity += mInvInertia * Cross2d(mR, impulse);
}

void ObjectProperties::CopyTo(Collider2d* collider)
{
  RigidBody2d* body = collider->mRigidBody;
  if (body != nullptr)
  {
    body->mLinearVelocity = mLinearVelocity;
    body->mAngularVelocity = mAngularVelocity;

    body->mWorldCenterOfMass = mCenterOfMass;
  }
}

ISolver::ISolver()
{
  mIterations = 10;
  mAllowedPenetration = 0.02f;
}
