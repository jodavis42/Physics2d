#pragma once

class SpatialPartitionData
{
public:
  SpatialPartitionData();
  SpatialPartitionData(const Aabb2d& aabb, void* clientData);

  Aabb2d mAabb;
  void* mClientData;
};

class SpatialPartitionKey
{
public:
  union
  {
    void* mVoidKey;
    int mIntKey;
  };
};

class RayCastResult
{
public:

  RayCastResult();
  RayCastResult(Real t, void* clientData);

  void* mClientData;
  Real mT;
};

class RayCastResults
{
public:
  void Add(Real t, void* clientData);
  void Add(const RayCastResult& result);

  Array<RayCastResult> mResults;
};

class SelfQueryResult
{
public:
  SelfQueryResult();
  SelfQueryResult(void* clientData0, void* clientData1);

  void* mClientData0;
  void* mClientData1;
};

class SelfQueryResults
{
public:
  void Add(void* clientData0, void* clientData1);
  void Add(const SelfQueryResult& result);

  Array<SelfQueryResult> mResults;
};

class SpatialPartition
{
public:
  ~SpatialPartition() {};
  virtual void Insert(const SpatialPartitionData& data, SpatialPartitionKey& key) = 0;
  virtual void Update(const SpatialPartitionData& data, SpatialPartitionKey& key) = 0;
  virtual void Remove(SpatialPartitionKey& key) = 0;

  virtual void RayCast(const Ray2d& ray, RayCastResults& results) = 0;
  virtual void SelfQuery(SelfQueryResults& results) = 0;

  virtual void DebugDraw(int depth) = 0;
};
