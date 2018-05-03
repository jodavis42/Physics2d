#pragma once

struct ObjectProperties
{
  void Compute(RigidBody2d* body, Real2Param point);
  void Compute(Collider2d* collider, Real2Param point);

  Real2 ComputePointVelocity();
  void ApplyImpulse(Real2Param impulse);
  void CopyTo(Collider2d* collider);

  Real2 mLinearVelocity;
  Real mAngularVelocity;
  Real mInvMass;
  Real mInvInertia;
  Real2 mCenterOfMass;
  Real2 mR;
};

struct ContactImpulse
{
  ContactImpulse();
  void Set(Manifold& manifold, int contactIndex);
  void SolveVelocityImpulse();
  void SolvePositionImpulse(Real allowedPenetration, Real percentage = 0.5f);

  Real ComputeImpulse(Real2Param direction, ObjectProperties& objA, ObjectProperties& objB, Real restitution);

  Real2 mPointA;
  Real2 mPointB;
  Real2 mNormal;
  Real mPenetration;
  Collider2d* mColliderA;
  Collider2d* mColliderB;
};

class ResolutionSolver
{
public:
  ResolutionSolver();
  void StartFrame();
  void Add(Manifold& manifold);

  void Solve();
  void DebugDraw();

  size_t mIterations;
  Array<ContactImpulse> mContactImpulses;
  Real mAllowedPenetration;
};
