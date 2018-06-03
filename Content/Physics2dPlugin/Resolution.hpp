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

class ISolver
{
public:
  ISolver();
  virtual ~ISolver() {};
  virtual void StartFrame() = 0;
  virtual void Add(Manifold& manifold) = 0;

  virtual void Solve() = 0;
  virtual void DebugDraw() = 0;

  size_t mIterations;
  Real mAllowedPenetration;
};
