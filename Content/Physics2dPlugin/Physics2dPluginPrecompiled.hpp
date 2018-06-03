#pragma once
// We use precompiled headers to support the fastest possible compilation
#define ZeroImportDll
#include "Zilch.hpp"
#include "Core.hpp"
#include "ZeroEngine.hpp"

using Zilch::Real;
using Zilch::Real2;
using Zilch::Real2Param;
using Zilch::Real3;
using Zilch::Real3Param;
using Zilch::Real4;
using Zilch::Real4Param;
using Zilch::Real2x2;
using Zilch::Real2x2Param;
using Zilch::Quaternion;

using Zero::Array;
using Zero::HashMap;
using ZeroEngine::Transform;

// Declares our Zilch Library (where we get LibraryBuilder from)
// This also handles the plugin initialization
ZilchDeclareStaticLibraryAndPlugin(Physics2dPluginLibrary, Physics2dPluginPlugin);

// We also encourage including all files within here, rather than within headers
// If another project needed to include any headers from our project, then they would simply
// include our precompiled header instead (ideally within their own precompiled header)
// This also must means you must order headers in dependency order (who depends on who)

#include "Physics2dPlugin.hpp"
#include "RigidBody2d.hpp"
#include "PhysicsSpace2d.hpp"
#include "Collider2d.hpp"
#include "SphereCollider2d.hpp"
#include "BoxCollider2d.hpp"
#include "Shapes.hpp"
#include "Intersection.hpp"
#include "Resolution.hpp"
#include "Geometry.hpp"
#include "SpatialPartition.hpp"
#include "AabbSpatialPartition.hpp"
#include "Debug.hpp"
#include "SupportShapes.hpp"
#include "Sat.hpp"
#include "Gjk.hpp"
#include "VoronoiRegions.hpp"
#include "Epa.hpp"
#include "ImpulseSolver.hpp"
#include "ConstraintSolver.hpp"
// Auto Includes (used by Visual Studio plugins, do not remove this line)
