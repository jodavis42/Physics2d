﻿#include "Physics2dPluginPrecompiled.hpp"

//***************************************************************************
ZilchDefineType(PhysicsSpace2d, builder, type)
{
  // This is required for component binding
  ZilchBindDestructor();
  ZilchBindConstructor();
  ZilchBindMethod(Initialize);
  ZilchBindMethod(OnAllObjectsInitialized);

  // Note: All event connection methods must be bound
  ZilchBindMethod(OnLogicUpdate);
  ZilchBindMethod(OnFrameUpdate);

  ZilchBindFieldProperty(mGravityAcceleration);
  ZilchBindFieldProperty(mDebugDraw);
  ZilchBindGetterSetterProperty(SolverMode);

  type->AddAttribute("RunInEditor");
}

//***************************************************************************
PhysicsSpace2d::PhysicsSpace2d()
{
  mGravityAcceleration = Real2(0, -10);
  mSolver = new ImpulseSolver();
  mDebugDraw = true;
}

//***************************************************************************
PhysicsSpace2d::~PhysicsSpace2d()
{
  Destroy();
}

//***************************************************************************
void PhysicsSpace2d::Initialize(ZeroEngine::CogInitializer* initializer)
{
  mSpatialPartition = new AabbSpatialPartition();
  ZeroConnectThisTo(initializer, "AllObjectsInitialized", "OnAllObjectsInitialized")
  ZeroConnectThisTo(this->GetSpace(), "LogicUpdate", "OnLogicUpdate");
  ZeroConnectThisTo(this->GetSpace(), "FrameUpdate", "OnFrameUpdate");
}

void PhysicsSpace2d::Destroy()
{
  while (!mRigidBodies.Empty())
  {
    RigidBody2d* body = mRigidBodies.Front();
    Remove(body);
  }
  while (!mColliders.Empty())
  {
    Collider2d* collider = mColliders.Front();
    Remove(collider);
  }
  
  delete mSpatialPartition;
  mSpatialPartition = nullptr;
  delete mSolver;
  mSolver = nullptr;
}

void PhysicsSpace2d::OnAllObjectsInitialized(ZeroEngine::Event* event)
{
  // Currently there's run-in-editor initialization order issues.
  // To get around this add any pre-existing objects when first initialized.
  Zilch::HandleOf<ZeroEngine::SpaceRange> range = GetSpace()->GetAllObjects();
  for (; !range->GetIsEmpty(); range->MoveNext())
  {
    ZeroEngine::Cog* cog = range->GetCurrent();
    RigidBody2d* body = cog->has(RigidBody2d);
    if (body != nullptr)
      Add(body);
    Collider2d* collider = cog->has(Collider2d);
    if (collider != nullptr)
      Add(collider);
  }
}

void PhysicsSpace2d::OnLogicUpdate(ZeroEngine::UpdateEvent* event)
{
  float dt = 1 / 60.0f;
  Step(dt);
  Publish();
}

void PhysicsSpace2d::OnFrameUpdate(ZeroEngine::UpdateEvent* event)
{
  if (mDebugDraw)
    DebugDraw();
}

void PhysicsSpace2d::Step(float dt)
{
  Integrate(dt);
  BroadPhase();
  NarrowPhase();
  Resolution();
}

void PhysicsSpace2d::Integrate(float dt)
{
  IntegrateVelocity(dt);
  IntegratePosition(dt);
}

void PhysicsSpace2d::IntegrateVelocity(float dt)
{
  ApplyGlobalForces(dt);
  for (size_t i = 0; i < mRigidBodies.Size(); ++i)
  {
    RigidBody2d* body = mRigidBodies[i];
    // Static body
    if (body->mInvMass == 0)
      continue;

    body->mLinearVelocity += body->mInvMass * body->mForce * dt;
    body->mAngularVelocity += body->mInvInertia * body->mTorque * dt;

    body->mForce = Real2::cZero;
    body->mTorque = 0;
  }
}

void PhysicsSpace2d::IntegratePosition(float dt)
{
  for (size_t i = 0; i < mRigidBodies.Size(); ++i)
  {
    RigidBody2d* body = mRigidBodies[i];
    body->mWorldCenterOfMass += body->mLinearVelocity * dt;
    body->mWorldRotation += body->mAngularVelocity * dt;

    body->mCollider->mWorldTranslation = body->mWorldCenterOfMass;
    body->mCollider->mWorldRotation = body->mWorldRotation;
  }
}

void PhysicsSpace2d::ApplyGlobalForces(float dt)
{
  for (size_t i = 0; i < mRigidBodies.Size(); ++i)
  {
    RigidBody2d* body = mRigidBodies[i];
    // Static body
    if (body->mInvMass == 0)
      continue;

    float mass = 1.0f / body->mInvMass;
    body->mForce += mGravityAcceleration * mass;
  }
}

void PhysicsSpace2d::BroadPhase()
{
  mPossiblePairs.Clear();
  for (size_t i = 0; i < mColliders.Size(); ++i)
  {
    Collider2d* collider = mColliders[i];
    mSpatialPartition->Update(collider->GetSpatialPartitionData(), collider->mKey);
  }

  SelfQueryResults results;
  mSpatialPartition->SelfQuery(results);

  for (size_t i = 0; i < results.mResults.Size(); ++i)
  {
    SelfQueryResult& result = results.mResults[i];
    ColliderPair pair;
    pair.mFirst = (Collider2d*)result.mClientData0;
    pair.mSecond = (Collider2d*)result.mClientData1;
    mPossiblePairs.PushBack(pair);
  }
}

void PhysicsSpace2d::NarrowPhase()
{
  mSolver->StartFrame();
  for (size_t i = 0; i < mPossiblePairs.Size(); ++i)
  {
    ColliderPair& pair = mPossiblePairs[i];
    Collider2d* colliderA = pair.mFirst;
    Collider2d* colliderB = pair.mSecond;

    Manifold manifold;
    if (!Intersection::Intersect(colliderA, colliderB, &manifold))
      continue;
    
    mSolver->Add(manifold);
  }
}

void PhysicsSpace2d::Resolution()
{
  mSolver->Solve();
}

void PhysicsSpace2d::Publish()
{
  for (size_t i = 0; i < mRigidBodies.Size(); ++i)
  {
    RigidBody2d* body = mRigidBodies[i];
    Transform* transform = body->GetOwner()->has(Transform);

    Real3 worldPosition = Real3(body->mWorldCenterOfMass.x, body->mWorldCenterOfMass.y, 0);
    transform->SetWorldTranslation(worldPosition);

    Quaternion worldRotation = Math::ToQuaternion(Real3(0, 0, 1), body->mWorldRotation);
    transform->SetWorldRotation(worldRotation);

    body->mCollider->UpdateBoundingVolumes();
  }

  SendEvents();
}

void PhysicsSpace2d::SendEvents()
{

}

void PhysicsSpace2d::DebugDraw()
{
  mSpatialPartition->DebugDraw(0);
  mSolver->DebugDraw();
}

SolverMode::Enum PhysicsSpace2d::GetSolverMode()
{
  return mSolverMode;
}

void PhysicsSpace2d::SetSolverMode(SolverMode::Enum mode)
{
  if (mode != mSolverMode)
  {
    delete mSolver;
    if (mode == SolverMode::Constraints)
      mSolver = new ConstraintSolver();
    else if (mode == SolverMode::Impulses)
      mSolver = new ImpulseSolver();
  }
  mSolverMode = mode;
}

void PhysicsSpace2d::Add(RigidBody2d* body)
{
  mRigidBodies.PushBack(body);
  body->mSpace = this;
}

void PhysicsSpace2d::Remove(RigidBody2d* body)
{
  mRigidBodies.EraseValueError(body);
  body->mSpace = nullptr;
}

void PhysicsSpace2d::Add(Collider2d* collider)
{
  mColliders.PushBack(collider);
  mSpatialPartition->Insert(collider->GetSpatialPartitionData(), collider->mKey);
  collider->mSpace = this;
}

void PhysicsSpace2d::Remove(Collider2d* collider)
{
  mColliders.EraseValueError(collider);
  mSpatialPartition->Remove(collider->mKey);
  collider->mSpace = nullptr;
}
