#pragma once

#include "Intersection.hpp"
#include "Resolution.hpp"

class Collider2d;
class RigidBody2d;
class SpatialPartition;

class ColliderPair
{
public:
  Collider2d* mFirst;
  Collider2d* mSecond;
};

class PhysicsSpace2d : public ZeroEngine::ZilchComponent
{
public:
  ZilchDeclareType(Zilch::TypeCopyMode::ReferenceType);
  
  PhysicsSpace2d();
  ~PhysicsSpace2d();
  
  void Initialize(ZeroEngine::CogInitializer* initializer);
  void Destroy();
  
  void OnAllObjectsInitialized(ZeroEngine::Event* event);
  void OnLogicUpdate(ZeroEngine::UpdateEvent* event);
  void OnFrameUpdate(ZeroEngine::UpdateEvent* event);

  void Step(float dt);
  void Integrate(float dt);
  void IntegrateVelocity(float dt);
  void IntegratePosition(float dt);
  void ApplyGlobalForces(float dt);
  void BroadPhase();
  void NarrowPhase();
  void Resolution();
  void Publish();
  void SendEvents();
  void DebugDraw();

  void Add(RigidBody2d* body);
  void Remove(RigidBody2d* body);
  void Add(Collider2d* collider);
  void Remove(Collider2d* collider);

  SpatialPartition* mSpatialPartition;
  Array<RigidBody2d*> mRigidBodies;
  Array<Collider2d*> mColliders;
  Array<ColliderPair> mPossiblePairs;
  ResolutionSolver mSolver;

  Real2 mGravityAcceleration;
  bool mDebugDraw;
};
