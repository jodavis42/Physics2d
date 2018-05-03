#pragma once

#include "SpatialPartition.hpp"

class AabbSpatialPartition : public SpatialPartition
{
public:
  AabbSpatialPartition();
  ~AabbSpatialPartition();

  void Insert(const SpatialPartitionData& data, SpatialPartitionKey& key) override;
  void Update(const SpatialPartitionData& data, SpatialPartitionKey& key) override;
  void Remove(SpatialPartitionKey& key) override;

  void RayCast(const Ray2d& ray, RayCastResults& results) override;
  void SelfQuery(SelfQueryResults& results) override;

  void DebugDraw(int depth) override;

private:
  size_t GetOrCreateIndex();

  struct Item
  {
    Aabb2d mAabb;
    void* mClientData;
    bool mEmpty;
  };

  Array<Item> mItems;
  Array<size_t> mFreeSlots;
};
