#include "Physics2dPluginPrecompiled.hpp"

//***************************************************************************
ZilchDefineType(Collider2d, builder, type)
{
  // This is required for component binding
  ZilchBindDestructor();
  ZilchBindConstructor();
  ZilchBindMethod(Initialize);
  
  // Note: All event connection methods must be bound
  ZilchBindMethod(OnTransformUpdated);
  ZilchBindMethod(OnLogicUpdate);
  ZilchBindMethod(OnCogDestroy);
  ZilchBindMethod(DebugDraw);

  type->AddAttribute("RunInEditor");
  type->AddAttribute("Dependency")->AddParameter("Transform");
}

//***************************************************************************
Collider2d::Collider2d()
{
  mSphereCollider = nullptr;
  mBoxCollider = nullptr;
  mRigidBody = nullptr;
  mWorldTranslation = Real2::cZero;
  mWorldRotation = 0.0f;
  mColliderType = Collider2dType::None;
}

//***************************************************************************
Collider2d::~Collider2d()
{
  Destroy();
}

//***************************************************************************
void Collider2d::Initialize(ZeroEngine::CogInitializer* initializer)
{
  ZeroConnectThisTo(this->GetOwner(), "TransformUpdated", "OnTransformUpdated");
  ZeroConnectThisTo(this->GetOwner(), "CogDestroy", "OnCogDestroy");

  mSpace = GetSpace()->has(PhysicsSpace2d);
  if(mSpace != nullptr)
    mSpace->Add(this);

  ReadTransform();
}

void Collider2d::Destroy()
{
  if (mSpace != nullptr)
    mSpace->Remove(this);
  mSpace = nullptr;
}

void Collider2d::OnTransformUpdated(ZeroEngine::Event* event)
{
  ReadTransform();
}

void Collider2d::OnLogicUpdate(ZeroEngine::UpdateEvent* event)
{
}

void Collider2d::OnCogDestroy(ZeroEngine::Event* event)
{
  Destroy();
}

void Collider2d::DebugDraw()
{
  DrawAabb(mAabb, Real4(0.7f));
}

Real2 Collider2d::GetWorldScale()
{
  return mWorldScale;
}

void Collider2d::SetWorldScale(Real2Param scale)
{
  mWorldScale = scale;
}

Real Collider2d::GetWorldRotation()
{
  return mWorldRotation;
}

void Collider2d::SetWorldRotation(Real rotation)
{
  mWorldRotation = rotation;
}

Real2 Collider2d::GetWorldTranslation()
{
  return mWorldTranslation;
}

void Collider2d::SetWorldTranslation(Real2Param translation)
{
  mWorldTranslation = translation;
}

void Collider2d::Set(SphereCollider2d* collider)
{
  mSphereCollider = collider;
  mColliderType = Collider2dType::Sphere;
}

void Collider2d::Clear(SphereCollider2d* collider)
{
  mSphereCollider = nullptr;
  mColliderType = Collider2dType::None;
}

void Collider2d::Set(BoxCollider2d* collider)
{
  mBoxCollider = collider;
  mColliderType = Collider2dType::Box;
}

void Collider2d::Clear(BoxCollider2d* collider)
{
  mBoxCollider = nullptr;
  mColliderType = Collider2dType::None;
}

void Collider2d::ReadTransform()
{
  Transform* transform = GetOwner()->has(Transform);
  mWorldTranslation = Math::ToVector2(transform->GetTranslation());
  Quaternion quat = transform->GetWorldRotation();
  Math::EulerAngles angles = Math::ToEulerAngles(quat);
  mWorldRotation = angles.Angles.z;
  mWorldScale = Math::ToVector2(transform->GetWorldScale());

  UpdateBoundingVolumes();
}

SpatialPartitionData Collider2d::GetSpatialPartitionData() const
{
  return SpatialPartitionData(mAabb, (void*)this);
}

void Collider2d::UpdateBoundingVolumes()
{
  if(mBoxCollider != nullptr)
    mBoxCollider->UpdateBoundingVolumes();
  if(mSphereCollider != nullptr)
    mSphereCollider->UpdateBoundingVolumes();

  if (mSpace != nullptr)
    mSpace->mSpatialPartition->Update(GetSpatialPartitionData(), mKey);
}

SupportShape* Collider2d::CreateSupportShape()
{
  if (mBoxCollider != nullptr)
    return new BoxSupportShape(mBoxCollider);
  if (mSphereCollider != nullptr)
    return new SphereSupportShape(mSphereCollider);
  return nullptr;
}

