#pragma once

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

class ImpulseSolver : public ISolver
{
public:
  ImpulseSolver();

  void StartFrame() override;
  void Add(Manifold& manifold) override;

  void Solve() override;
  void DebugDraw() override;

  Array<ContactImpulse> mContactImpulses;
};
