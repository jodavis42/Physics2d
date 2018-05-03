#pragma once

class Gjk
{
public:
  struct CsoPoint
  {
    Real2 mCsoPoint;
    Real2 mPointA;
    Real2 mPointB;
  };

  Gjk();

  bool Intersect(const SupportShape* shapeA, const SupportShape* shapeB);
  bool Intersect(const SupportShape* shapeA, const SupportShape* shapeB, Manifold* manifold);
  bool Intersect(const SupportShape* shapeA, const SupportShape* shapeB, Real skin, Manifold* manifold);

  void FilloutManifold(const CsoPoint& closestPoints, Manifold* manifold, Real skin);
  CsoPoint ComputeSupport(const SupportShape* shapeA, const SupportShape* shapeB, const Real2& direction);

  Real mProgressEpsilon;
  size_t mMaxIterations;

  size_t mSize;
  CsoPoint mSimplex[3];

private:

  Real2 Intialize(const SupportShape* shapeA, const SupportShape* shapeB);

  void ComputeClosestFeatureInfo(const Real2& q, Real2& searchDirection, Real2& closestPoint, size_t& newSize, int newIndices[3]);
  void Reduce(size_t& newSize, int newIndices[3]);
  bool CheckProgress(const CsoPoint& newPoint, Real2& closestPoint, Real2 searchDirection);
  void ComputeClosestPoint(CsoPoint& closestPoint);

  
  Real mSkin;
  Real2 mClosestCsoPoint;
  Real2 mSearchDirection;
  const SupportShape* mShapeA;
  const SupportShape* mShapeB;
};