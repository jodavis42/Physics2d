#pragma once

struct Jacobian
{
  Real2 mLinear0;
  Real mAngular0;
  Real2 mLinear1;
  Real mAngular1;
};

struct ContactConstraint
{
  ContactConstraint();

  void Set(Manifold& manifold, int contactIndex);
  void SolveVelocityImpulse();
  void SolvePositionImpulse(Real allowedPenetration, Real percentage = 0.5f);

  void SolveDirection(Real2Param direction, Real minImpulse, Real maxImpulse, Real& jTotal, Real b, ObjectProperties& objA, ObjectProperties& objB);
  void ComputeJacobian(Jacobian& jacobian, Real2Param direction, ObjectProperties& objA, ObjectProperties& objB);
  Real ComputeEffectiveMass(Jacobian& jacobian, ObjectProperties& objA, ObjectProperties& objB);
  Real ComputeImpulse(Jacobian& jacobian, ObjectProperties& objA, ObjectProperties& objB, Real b);
  void ApplyImpulse(Real lambda, Jacobian& jacobian, ObjectProperties& objA, ObjectProperties& objB);

  Real2 mPointA;
  Real2 mPointB;
  Real2 mNormal;
  Real2 mTangent;
  Real mPenetration;

  Real mB;
  Real mAccumulatedNormalImpulse;
  Real mAccumulatedTangentImpulse;
  Collider2d* mColliderA;
  Collider2d* mColliderB;
};

class ConstraintSolver : public ISolver
{
public:
  ConstraintSolver();

  void StartFrame() override;
  void Add(Manifold& manifold) override;

  void Solve() override;
  void DebugDraw() override;

  Array<ContactConstraint> mContactConstraints;
};
