#include "Physics2dPluginPrecompiled.hpp"

ContactConstraint::ContactConstraint()
{

}

void ContactConstraint::Set(Manifold& manifold, int contactIndex)
{
  Manifold::ContactPoint& contactPoint = manifold.mContactPoints[contactIndex];
  mColliderA = manifold.mColliderA;
  mColliderB = manifold.mColliderB;
  mNormal = contactPoint.mNormal;
  mPenetration = contactPoint.mPenetration;
  mPointA = contactPoint.mPointA;
  mPointB = contactPoint.mPointB;

  mAccumulatedNormalImpulse = 0;
  mAccumulatedTangentImpulse = 0;

  ObjectProperties objA;
  ObjectProperties objB;
  objA.Compute(mColliderA, mPointA);
  objB.Compute(mColliderB, mPointB);
  Real2 pointVelA = objA.ComputePointVelocity();
  Real2 pointVelB = objB.ComputePointVelocity();
  Real2 relativeVelocity = pointVelB - pointVelA;
  Real separatingVelocity = Math::Dot(mNormal, relativeVelocity);

  mTangent = relativeVelocity - Math::Dot(mNormal, relativeVelocity) * mNormal;
  // Negate because we apply a tangent impulse in the opposite direction of the relative velocity (to slow it down)
  mTangent *= -1;

  Real restitution = 0.5f;
  Real bias = 3;
  mB = -mPenetration * bias;
  if (Math::Abs(separatingVelocity) > 1)
    mB += restitution * separatingVelocity;
}

void ContactConstraint::SolveVelocityImpulse()
{
  ObjectProperties objA;
  ObjectProperties objB;
  objA.Compute(mColliderA, mPointA);
  objB.Compute(mColliderB, mPointB);

  SolveDirection(mNormal, 0, Math::PositiveMax(), mAccumulatedNormalImpulse, mB, objA, objB);

  if (mTangent != Real2::cZero)
    SolveDirection(mTangent, -2, 2, mAccumulatedTangentImpulse, 0, objA, objB);

  objA.CopyTo(mColliderA);
  objB.CopyTo(mColliderB);
}

void ContactConstraint::SolvePositionImpulse(Real allowedPenetration, Real percentage)
{

}

void ContactConstraint::SolveDirection(Real2Param direction, Real minImpulse, Real maxImpulse, Real& jTotal, Real b, ObjectProperties& objA, ObjectProperties& objB)
{
  Jacobian jacobian;
  ComputeJacobian(jacobian, direction, objA, objB);

  Real jN = ComputeImpulse(jacobian, objA, objB, b);
  Real oldJ = jTotal;
  jTotal = Math::Clamp(jTotal + jN, minImpulse, maxImpulse);
  jN = jTotal - oldJ;
  ApplyImpulse(jN, jacobian, objA, objB);
}

void ContactConstraint::ComputeJacobian(Jacobian& jacobian, Real2Param direction, ObjectProperties& objA, ObjectProperties& objB)
{
  jacobian.mLinear0 = -direction;
  jacobian.mLinear1 = direction;
  jacobian.mAngular0 = -Cross2d(objA.mR, direction);
  jacobian.mAngular1 = Cross2d(objB.mR, direction);
}

Real ContactConstraint::ComputeEffectiveMass(Jacobian& jacobian, ObjectProperties& objA, ObjectProperties& objB)
{
  Real effectiveMass = Math::LengthSq(jacobian.mLinear0) * objA.mInvMass;
  effectiveMass += Math::LengthSq(jacobian.mLinear1) * objB.mInvMass;
  effectiveMass += jacobian.mAngular0 * jacobian.mAngular0 * objA.mInvInertia;
  effectiveMass += jacobian.mAngular1 * jacobian.mAngular1 * objB.mInvInertia;
  return effectiveMass;
}

Real ContactConstraint::ComputeImpulse(Jacobian& jacobian, ObjectProperties& objA, ObjectProperties& objB, Real b)
{
  Real jv = Math::Dot(jacobian.mLinear0, objA.mLinearVelocity);
  jv += Math::Dot(jacobian.mLinear1, objB.mLinearVelocity);
  jv += jacobian.mAngular0 * objA.mAngularVelocity;
  jv += jacobian.mAngular1 * objB.mAngularVelocity;
  Real effectiveMass = ComputeEffectiveMass(jacobian, objA, objB);

  Real j = -(jv + b) / effectiveMass;
  return j;
}

void ContactConstraint::ApplyImpulse(Real lambda, Jacobian& jacobian, ObjectProperties& objA, ObjectProperties& objB)
{
  objA.mLinearVelocity += lambda * jacobian.mLinear0 * objA.mInvMass;
  objA.mAngularVelocity += lambda * jacobian.mAngular0 * objA.mInvInertia;
  objB.mLinearVelocity += lambda * jacobian.mLinear1 * objB.mInvMass;
  objB.mAngularVelocity += lambda * jacobian.mAngular1 * objB.mInvInertia;
}

ConstraintSolver::ConstraintSolver()
{
}

void ConstraintSolver::StartFrame()
{
  mContactConstraints.Clear();
}

void ConstraintSolver::Add(Manifold& manifold)
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
    ContactConstraint& constraint = mContactConstraints.PushBack();
    constraint.Set(manifold, i);
  }
}

void ConstraintSolver::Solve()
{
  for (size_t iteration = 0; iteration < mIterations; ++iteration)
  {
    for (size_t i = 0; i < mContactConstraints.Size(); ++i)
    {
      mContactConstraints[i].SolveVelocityImpulse();
    }
  }

  for (size_t i = 0; i < mContactConstraints.Size(); ++i)
  {
    mContactConstraints[i].SolvePositionImpulse(mAllowedPenetration, 0.5f);
  }
}

void ConstraintSolver::DebugDraw()
{
  for (size_t i = 0; i < mContactConstraints.Size(); ++i)
  {
    ContactConstraint& contact = mContactConstraints[i];
    DrawPoint(contact.mPointA, 0.05f, Real4(0, 0, 1, 1));
    DrawPoint(contact.mPointB, 0.05f, Real4(0, 0, 1, 1));

    Ray2d normalRay(contact.mPointA, contact.mNormal);
    DrawRay(normalRay, 1.0f, Real4(0, 0, 1, 1));
  }
}
