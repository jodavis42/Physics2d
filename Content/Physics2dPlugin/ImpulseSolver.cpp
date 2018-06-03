#include "Physics2dPluginPrecompiled.hpp"


ContactImpulse::ContactImpulse()
{
  mColliderA = nullptr;
  mColliderB = nullptr;
}

void ContactImpulse::Set(Manifold& manifold, int contactIndex)
{
  Manifold::ContactPoint& contactPoint = manifold.mContactPoints[contactIndex];
  mColliderA = manifold.mColliderA;
  mColliderB = manifold.mColliderB;
  mNormal = contactPoint.mNormal;
  mPenetration = contactPoint.mPenetration;
  mPointA = contactPoint.mPointA;
  mPointB = contactPoint.mPointB;
}

void ContactImpulse::SolveVelocityImpulse()
{
  Real restitution = 0.5f;
  ObjectProperties objA;
  ObjectProperties objB;
  objA.Compute(mColliderA, mPointA);
  objB.Compute(mColliderB, mPointB);

  Real2 pointVelA = objA.ComputePointVelocity();
  Real2 pointVelB = objB.ComputePointVelocity();
  Real2 relativeVelocity = pointVelB - pointVelA;
  Real2 tangent = relativeVelocity - Math::Dot(mNormal, relativeVelocity) * mNormal;
  // Negate because we apply a tangent impulse in the opposite direction of the relative velocity (to slow it down)
  tangent *= -1;
  Math::AttemptNormalize(tangent);

  Real separatingVelocity = Math::Dot(mNormal, relativeVelocity);
  if (separatingVelocity > 0)
    return;


  Real jN = ComputeImpulse(mNormal, objA, objB, restitution);
  Real2 normalImpulse = jN * mNormal;
  objA.ApplyImpulse(-normalImpulse);
  objB.ApplyImpulse(normalImpulse);

  Real muS = 0.7f;
  Real muK = 0.7f;

  Real jT = ComputeImpulse(tangent, objA, objB, 0);

  // Columbs friction
  if (jN * muS < jT)
    jT = jN * muK;

  Real2 tangentImpulse = jT * tangent;
  tangentImpulse *= 0.5f;
  objA.ApplyImpulse(-tangentImpulse);
  objB.ApplyImpulse(tangentImpulse);

  //Real2 totalImpulse = normalImpulse + tangentImpulse * 0.5f;
  //objA.ApplyImpulse(-totalImpulse);
  //objB.ApplyImpulse(totalImpulse);

  objA.CopyTo(mColliderA);
  objB.CopyTo(mColliderB);
}

void ContactImpulse::SolvePositionImpulse(Real allowedPenetration, Real percentage)
{
  if (mPenetration < allowedPenetration)
    return;

  ObjectProperties objA;
  ObjectProperties objB;
  objA.Compute(mColliderA, mPointA);
  objB.Compute(mColliderB, mPointB);

  Real numerator = mPenetration * percentage;
  Real denominator = objA.mInvMass + objB.mInvMass;
  Real j = numerator / denominator;
  Real2 penetrationImpulse = mNormal * j;
  objA.mCenterOfMass -= objA.mInvMass * penetrationImpulse;
  objB.mCenterOfMass += objB.mInvMass * penetrationImpulse;
  objA.CopyTo(mColliderA);

  objB.CopyTo(mColliderB);
}

Real ContactImpulse::ComputeImpulse(Real2Param direction, ObjectProperties& objA, ObjectProperties& objB, Real restitution)
{
  Real2 pointVelA = objA.ComputePointVelocity();
  Real2 pointVelB = objB.ComputePointVelocity();
  Real2 relativeVelocity = pointVelB - pointVelA;
  Real separatingVelocity = Math::Dot(direction, relativeVelocity);

  Real numerator = -(1 + restitution) * separatingVelocity;
  Real inertiaTermA = Math::Sq(Cross2d(objA.mR, direction)) * objA.mInvInertia;
  Real inertiaTermB = Math::Sq(Cross2d(objB.mR, direction)) * objB.mInvInertia;
  Real denominator = objA.mInvMass + objB.mInvMass + inertiaTermA + inertiaTermB;

  Real j = numerator / denominator;
  return j;
}

ImpulseSolver::ImpulseSolver()
{

}

void ImpulseSolver::StartFrame()
{
  mContactImpulses.Clear();
}

void ImpulseSolver::Add(Manifold& manifold)
{
  RigidBody2d* bodyA = manifold.mColliderA->mRigidBody;
  RigidBody2d* bodyB = manifold.mColliderB->mRigidBody;
  Real invMassA = 0;
  Real invMassB = 0;
  if (bodyA != nullptr)
    invMassA = bodyA->mInvMass;
  if (bodyB != nullptr)
    invMassB = bodyB->mInvMass;

  if (invMassA == 0 && invMassB == 0)
    return;

  for (size_t i = 0; i < manifold.mContactCount; ++i)
  {
    ContactImpulse& contact = mContactImpulses.PushBack();
    contact.Set(manifold, i);
  }
}

void ImpulseSolver::Solve()
{
  for (size_t iteration = 0; iteration < mIterations; ++iteration)
  {
    for (size_t i = 0; i < mContactImpulses.Size(); ++i)
    {
      mContactImpulses[i].SolveVelocityImpulse();
    }
  }

  for (size_t i = 0; i < mContactImpulses.Size(); ++i)
  {
    mContactImpulses[i].SolvePositionImpulse(mAllowedPenetration, 0.5f);
  }
}

void ImpulseSolver::DebugDraw()
{
  for (size_t i = 0; i < mContactImpulses.Size(); ++i)
  {
    ContactImpulse& contact = mContactImpulses[i];
    DrawPoint(contact.mPointA, 0.05f, Real4(0, 0, 1, 1));
    DrawPoint(contact.mPointB, 0.05f, Real4(0, 0, 1, 1));

    Ray2d normalRay(contact.mPointA, contact.mNormal);
    DrawRay(normalRay, 1.0f, Real4(0, 0, 1, 1));
  }
}
