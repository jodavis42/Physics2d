#pragma once

class PhysicsSpace2d;
class Collider2d;

class RigidBody2d : public ZeroEngine::ZilchComponent
{
public:
  ZilchDeclareType(Zilch::TypeCopyMode::ReferenceType);
  
  RigidBody2d();
  ~RigidBody2d();
  
  void Initialize(ZeroEngine::CogInitializer* initializer);
  void Destroy();

  void OnLogicUpdate(ZeroEngine::UpdateEvent* event);
  void OnCogDestroy(ZeroEngine::Event* event);

  bool GetStatic();
  void SetStatic(bool state);

  Real2 mWorldCenterOfMass;
  Real mWorldRotation;

  float mInvMass;
  float mInvInertia;

  Real2 mLinearVelocity;
  Real mAngularVelocity;
  Real2 mForce;
  Real mTorque;

  PhysicsSpace2d* mSpace;
  Collider2d* mCollider;

  bool mStatic;
};
