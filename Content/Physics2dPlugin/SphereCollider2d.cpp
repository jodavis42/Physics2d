#include "Physics2dPluginPrecompiled.hpp"

//***************************************************************************
ZilchDefineType(SphereCollider2d, builder, type)
{
  // This is required for component binding
  ZilchBindDestructor();
  ZilchBindConstructor();
  ZilchBindMethod(Initialize);
  
  // Note: All event connection methods must be bound
  ZilchBindMethod(OnLogicUpdate);
  ZilchBindMethod(OnCogDestroy);
  ZilchBindMethod(DebugDraw);

  ZilchBindGetterSetterProperty(Radius);

  type->AddAttribute("RunInEditor");
  type->AddAttribute("Dependency")->AddParameter("Collider2d");
}

//***************************************************************************
SphereCollider2d::SphereCollider2d()
{
  mCollider = nullptr;
  mWorldRadius = mLocalRadius = 1.0f;
}

//***************************************************************************
SphereCollider2d::~SphereCollider2d()
{
}

//***************************************************************************
void SphereCollider2d::Initialize(ZeroEngine::CogInitializer* initializer)
{
  mCollider = GetOwner()->has(Collider2d);
  if(mCollider != nullptr)
    mCollider->Set(this);
  UpdateBoundingVolumes();
}

//***************************************************************************
void SphereCollider2d::OnLogicUpdate(ZeroEngine::UpdateEvent* event)
{
}

void SphereCollider2d::OnCogDestroy(ZeroEngine::Event* event)
{
  mCollider->Clear(this);
}

void SphereCollider2d::DebugDraw()
{
  if (mCollider == nullptr)
    return;

  Sphere2d sphere(mCollider->mWorldTranslation, mWorldRadius);
  DrawSphere(sphere);
}

Real SphereCollider2d::GetRadius()
{
  return mLocalRadius;
}

void SphereCollider2d::SetRadius(Real radius)
{
  mLocalRadius = Math::Max(0.01f, radius);
  UpdateBoundingVolumes();
  // Update mass etc...
}

void SphereCollider2d::UpdateBoundingVolumes()
{
  if (mCollider == nullptr)
    return;

  Real2 scale = mCollider->GetWorldScale();
  Real rotation = mCollider->GetWorldRotation();
  Real2 translation = mCollider->GetWorldTranslation();

  mWorldRadius = mLocalRadius * Math::Max(scale.x, scale.y);
  Aabb2d& aabb = mCollider->mAabb;

  Real2 pos = mCollider->mWorldTranslation;
  Real2 size = Real2(mWorldRadius);
  aabb.mMin = pos - size;
  aabb.mMax = pos + size;
}
