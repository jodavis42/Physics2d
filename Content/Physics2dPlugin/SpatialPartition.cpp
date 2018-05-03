#include "Physics2dPluginPrecompiled.hpp"

#include "SpatialPartition.hpp"

SpatialPartitionData::SpatialPartitionData()
{
  mClientData = nullptr;
}

SpatialPartitionData::SpatialPartitionData(const Aabb2d& aabb, void* clientData)
{
  mAabb = aabb;
  mClientData = clientData;
}

RayCastResult::RayCastResult()
{
  mT = 0;
  mClientData = nullptr;
}

RayCastResult::RayCastResult(Real t, void* clientData)
{
  mT = t;
  mClientData = clientData;
}

void RayCastResults::Add(Real t, void* clientData)
{
  Add(RayCastResult(t, clientData));
}

void RayCastResults::Add(const RayCastResult& result)
{
  mResults.PushBack(result);
}

SelfQueryResult::SelfQueryResult()
{
  mClientData0 = nullptr;
  mClientData1 = nullptr;
}

SelfQueryResult::SelfQueryResult(void* clientData0, void* clientData1)
{
  mClientData0 = clientData0;
  mClientData1 = clientData1;
}

void SelfQueryResults::Add(void* clientData0, void* clientData1)
{
  Add(SelfQueryResult(clientData0, clientData1));
}

void SelfQueryResults::Add(const SelfQueryResult& result)
{
  mResults.PushBack(result);
}
