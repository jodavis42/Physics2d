#pragma once

#include "Shapes.hpp"
#include "SpatialPartition.hpp"

class PhysicsSpace2d;
class SphereCollider2d;
class BoxCollider2d;
class RigidBody2d;

DeclareEnum3(Collider2dType, None, Sphere, Box);

class Collider2d : public ZeroEngine::ZilchComponent
{
public:
  ZilchDeclareType(Zilch::TypeCopyMode::ReferenceType);
  
  Collider2d();
  ~Collider2d();
  
  void Initialize(ZeroEngine::CogInitializer* initializer);
  void Destroy();

  void OnTransformUpdated(ZeroEngine::Event* event);
  void OnLogicUpdate(ZeroEngine::UpdateEvent* event);
  void OnCogDestroy(ZeroEngine::Event* event);
  void DebugDraw();

  Real2 GetWorldScale();
  void SetWorldScale(Real2Param scale);
  Real GetWorldRotation();
  void SetWorldRotation(Real rotation);
  Real2 GetWorldTranslation();
  void SetWorldTranslation(Real2Param translation);

  void Set(SphereCollider2d* collider);
  void Clear(SphereCollider2d* collider);

  void Set(BoxCollider2d* collider);
  void Clear(BoxCollider2d* collider);

  void ReadTransform();
  SpatialPartitionData GetSpatialPartitionData() const;
  void UpdateBoundingVolumes();
  SupportShape* CreateSupportShape();

  Real2 mWorldScale;
  Real mWorldRotation;
  Real2 mWorldTranslation;

  PhysicsSpace2d* mSpace;

  SphereCollider2d* mSphereCollider;
  BoxCollider2d* mBoxCollider;
  Collider2dType::Enum mColliderType;
  RigidBody2d* mRigidBody;
  Aabb2d mAabb;
  SpatialPartitionKey mKey;
};
