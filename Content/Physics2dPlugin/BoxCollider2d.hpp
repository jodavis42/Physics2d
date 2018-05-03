#pragma once

class BoxCollider2d : public ZeroEngine::ZilchComponent
{
public:
  ZilchDeclareType(Zilch::TypeCopyMode::ReferenceType);
  
  BoxCollider2d();
  ~BoxCollider2d();
  
  void Initialize(ZeroEngine::CogInitializer* initializer);

  void OnLogicUpdate(ZeroEngine::UpdateEvent* event);
  void OnCogDestroy(ZeroEngine::Event* event);
  void DebugDraw();

  Real2 GetSize();
  void SetSize(Real2 radius);

  void UpdateBoundingVolumes();

  Real2 mSize;

  Collider2d* mCollider;
};
