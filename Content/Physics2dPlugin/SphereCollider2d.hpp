#pragma once

class Collider2d;

class SphereCollider2d : public ZeroEngine::ZilchComponent
{
public:
  ZilchDeclareType(Zilch::TypeCopyMode::ReferenceType);
  
  SphereCollider2d();
  ~SphereCollider2d();
  
  void Initialize(ZeroEngine::CogInitializer* initializer);
  
  void OnLogicUpdate(ZeroEngine::UpdateEvent* event);
  void OnCogDestroy(ZeroEngine::Event* event);
  void DebugDraw();

  Real GetRadius();
  void SetRadius(Real radius);

  void UpdateBoundingVolumes();

  Real mRadius;

  Collider2d* mCollider;
};
