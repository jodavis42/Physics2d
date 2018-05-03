#include "Physics2dPluginPrecompiled.hpp"

#include "AabbSpatialPartition.hpp"

AabbSpatialPartition::AabbSpatialPartition()
{

}

AabbSpatialPartition::~AabbSpatialPartition()
{
}

void AabbSpatialPartition::Insert(const SpatialPartitionData& data, SpatialPartitionKey& key)
{
  size_t index = GetOrCreateIndex();
  Item& item = mItems[index];
  item.mEmpty = false;
  item.mAabb = data.mAabb;
  item.mClientData = data.mClientData;

  key.mIntKey = (int)index;
}

void AabbSpatialPartition::Update(const SpatialPartitionData& data, SpatialPartitionKey& key)
{
  Item& item = mItems[key.mIntKey];
  item.mAabb = data.mAabb;
}

void AabbSpatialPartition::Remove(SpatialPartitionKey& key)
{
  mItems[key.mIntKey].mEmpty = true;
  mFreeSlots.PushBack(key.mIntKey);
}

void AabbSpatialPartition::RayCast(const Ray2d& ray, RayCastResults& results)
{
  for (size_t i = 0; i < mItems.Size(); ++i)
  {
    Item& item = mItems[i];
    if (item.mEmpty)
      continue;

    Real t;
    if (RayAabb(ray, item.mAabb, t))
      results.Add(t, item.mClientData);
  }
}

void AabbSpatialPartition::SelfQuery(SelfQueryResults& results)
{
  for (size_t i = 0; i < mItems.Size(); ++i)
  {
    Item& item0 = mItems[i];
    if (item0.mEmpty)
      continue;

    for (size_t j = i + 1; j < mItems.Size(); ++j)
    {
      Item& item1 = mItems[j];
      if (item1.mEmpty)
        continue;

      if (AabbAabb(item0.mAabb, item1.mAabb))
        results.Add(item0.mClientData, item1.mClientData);
    }
  }
}

void AabbSpatialPartition::DebugDraw(int depth)
{
  for (size_t i = 0; i < mItems.Size(); ++i)
  {
    Item& item = mItems[i];
    if (item.mEmpty)
      continue;

    DrawAabb(item.mAabb, Real4(0.7f));
  }
}

size_t AabbSpatialPartition::GetOrCreateIndex()
{
  if (!mFreeSlots.Empty())
  {
    int index = mFreeSlots.Back();
    mFreeSlots.PopBack();
    return index;
  }

  mItems.PushBack();
  return mItems.Size() - 1;
}
