#include "Physics2dPluginPrecompiled.hpp"

//***************************************************************************
ZilchDefineType(BoxCollider2d, builder, type)
{
  // This is required for component binding
  ZilchBindDestructor();
  ZilchBindConstructor();
  ZilchBindMethod(Initialize);
  
  // Note: All event connection methods must be bound
  ZilchBindMethod(OnLogicUpdate);
  ZilchBindMethod(OnCogDestroy);
  ZilchBindMethod(DebugDraw);

  ZilchBindGetterSetterProperty(Size);

  type->AddAttribute("RunInEditor");
  type->AddAttribute("Dependency")->AddParameter("Collider2d");
}

//***************************************************************************
BoxCollider2d::BoxCollider2d()
{
  mCollider = nullptr;
  mWorldSize = mLocalSize = Real2(1);
}

//***************************************************************************
BoxCollider2d::~BoxCollider2d()
{
}

//***************************************************************************
void BoxCollider2d::Initialize(ZeroEngine::CogInitializer* initializer)
{
  mCollider = GetOwner()->has(Collider2d);
  if (mCollider != nullptr)
  {
    mCollider->Set(this);
    mCollider->UpdateBoundingVolumes();
  }
}

//***************************************************************************
void BoxCollider2d::OnLogicUpdate(ZeroEngine::UpdateEvent* event)
{
}

void BoxCollider2d::OnCogDestroy(ZeroEngine::Event* event)
{
  mCollider->Clear(this);
}

void BoxCollider2d::DebugDraw()
{
  if (mCollider == nullptr)
    return;

  Aabb2d aabb = Aabb2d::BuildFromSize(mCollider->mWorldTranslation, mWorldSize);
  DrawObb(aabb, mCollider->mWorldRotation);
}

Real2 BoxCollider2d::GetSize()
{
  return mLocalSize;
}

void BoxCollider2d::SetSize(Real2 size)
{
  mLocalSize[0] = Math::Max(0.01f, size[0]);
  mLocalSize[1] = Math::Max(0.01f, size[1]);
  if (mCollider != nullptr)
    mCollider->UpdateBoundingVolumes();

  if(mCollider != nullptr)
  {
    RigidBody2d* body = mCollider->mRigidBody;
    if (body != nullptr && !body->mStatic)
    {
      Real w = mWorldSize[0];
      Real h = mWorldSize[1];
      Real density = 1;
      Real volume = w * h;
      Real mass = volume * density;
      Real inertia = (1 / 12.0f) * mass * (h * h + w * w);

      body->mInvMass = 1 / mass;
      body->mInvInertia = 1 / inertia;
    }
  }
}

void BoxCollider2d::UpdateBoundingVolumes()
{
  if (mCollider == nullptr)
    return;
  
  Aabb2d& aabb = mCollider->mAabb;

  Real2 scale = mCollider->GetWorldScale();
  Real rotation = mCollider->GetWorldRotation();
  Real2 translation = mCollider->GetWorldTranslation();

  mWorldSize = mLocalSize * scale;

  Real2x2 rotationMat, absRotationMat;
  rotationMat.Rotate(rotation);
  for (size_t i = 0; i < 4; ++i)
    absRotationMat.array[i] = Math::Abs(rotationMat.array[i]);

  Real2 r = Math::Multiply(absRotationMat, mWorldSize * 0.5f);
  aabb.mMin = translation - r;
  aabb.mMax = translation + r;
}
