#include "Physics2dPluginPrecompiled.hpp"

ZilchDefineExternalBaseType(SolverMode::Enum, Zilch::TypeCopyMode::ValueType, builder, type)
{
  ZilchFullBindEnum(builder, type, Zilch::SpecialType::Enumeration);
  ZilchBindEnumValues(SolverMode);
}

//***************************************************************************
ZilchDefineStaticLibraryAndPlugin(Physics2dPluginLibrary, Physics2dPluginPlugin, ZilchDependencyStub(Core) ZilchDependencyStub(ZeroEngine))
{
  ZilchInitializeType(Physics2dPlugin);
  ZilchInitializeType(Physics2dPluginEvent);
  ZilchInitializeType(RigidBody2d);
  ZilchInitializeType(PhysicsSpace2d);
  ZilchInitializeType(Collider2d);
  ZilchInitializeType(SphereCollider2d);
  ZilchInitializeType(BoxCollider2d);
  ZilchInitializeEnum(SolverMode);
  // Auto Initialize (used by Visual Studio plugins, do not remove this line)
}

//***************************************************************************
void Physics2dPluginPlugin::Initialize()
{
  // One time startup logic goes here
  // This runs after our plugin library/reflection is built
  Zilch::Console::WriteLine("Physics2dPluginPlugin::Initialize");
}

//***************************************************************************
void Physics2dPluginPlugin::Uninitialize()
{
  // One time shutdown logic goes here
  Zilch::Console::WriteLine("Physics2dPluginPlugin::Uninitialize");
}
