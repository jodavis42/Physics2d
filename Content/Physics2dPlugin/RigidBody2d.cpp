#include "Physics2dPluginPrecompiled.hpp"

//***************************************************************************
ZilchDefineType(RigidBody2d, builder, type)
{
  // This is required for component binding
  ZilchBindDestructor();
  ZilchBindConstructor();
  ZilchBindMethod(Initialize);
  
  // Note: All event connection methods must be bound
  ZilchBindMethod(OnLogicUpdate);
  ZilchBindMethod(OnCogDestroy);

  ZilchBindFieldProperty(mLinearVelocity);
  ZilchBindFieldProperty(mAngularVelocity);
  ZilchBindGetterSetterProperty(Static);

  type->AddAttribute("Dependency")->AddParameter("Collider2d");
}

//***************************************************************************
RigidBody2d::RigidBody2d()
{
  mCollider = nullptr;
  mWorldCenterOfMass = Real2::cZero;
  mWorldRotation = 0.0f;
  mInvMass = 1.0f;
  mInvInertia = 1.0f;
  mLinearVelocity = Real2::cZero;
  mAngularVelocity = 0.0f;
  mForce = Real2::cZero;
  mTorque = 0.0f;
}

//***************************************************************************
RigidBody2d::~RigidBody2d()
{
  Destroy();
}

//***************************************************************************
void RigidBody2d::Initialize(ZeroEngine::CogInitializer* initializer)
{
  ZeroConnectThisTo(this->GetOwner(), "CogDestroy", "OnCogDestroy");

  mSpace = GetSpace()->has(PhysicsSpace2d);
  mSpace->Add(this);
  mCollider = GetOwner()->has(Collider2d);
  mCollider->mRigidBody = this;
  mWorldCenterOfMass = mCollider->mWorldTranslation;
  mWorldRotation = mCollider->mWorldRotation;
  //ZeroConnectThisTo(this->GetSpace(), "LogicUpdate", "OnLogicUpdate");
}

void RigidBody2d::Destroy()
{
  if (mSpace != nullptr)
    mSpace->Remove(this);
  mSpace = nullptr;
  if (mCollider != nullptr)
    mCollider->mRigidBody = nullptr;
  mCollider = nullptr;
}

//***************************************************************************
void RigidBody2d::OnLogicUpdate(ZeroEngine::UpdateEvent* event)
{
}

void RigidBody2d::OnCogDestroy(ZeroEngine::Event* event)
{
  Destroy();
}

bool RigidBody2d::GetStatic()
{
  return mStatic;
}

void RigidBody2d::SetStatic(bool state)
{
  mStatic = state;
  if (mStatic)
  {
    mInvInertia = 0;
    mInvMass = 0;
  }
  else
  {
    mInvMass = 1;
    mInvInertia = 1;
  }
}
