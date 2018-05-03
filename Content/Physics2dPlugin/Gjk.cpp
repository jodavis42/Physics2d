#include "Physics2dPluginPrecompiled.hpp"
#include "VoronoiRegions.hpp"

Gjk::Gjk()
{
  mSize = 0;
  mProgressEpsilon = 0.001f;
  mMaxIterations = 100;
  mSkin = 0;
}

bool Gjk::Intersect(const SupportShape* shapeA, const SupportShape* shapeB)
{
  Real2 q = Real2::cZero;
  mSearchDirection = Intialize(shapeA, shapeB);
  if (mSearchDirection == q)
    return true;

  mSize = 1;
  mSimplex[0] = ComputeSupport(shapeA, shapeB, mSearchDirection);
  size_t newSize;
  int indices[3];
  for (size_t i = 0; i < mMaxIterations; ++i)
  {
    ComputeClosestFeatureInfo(q, mSearchDirection, mClosestCsoPoint, newSize, indices);
    Reduce(newSize, indices);
    mSearchDirection.AttemptNormalize();

    if (mClosestCsoPoint == q)
      return true;

    CsoPoint newPoint = ComputeSupport(shapeA, shapeB, mSearchDirection);
    if (!CheckProgress(newPoint, mClosestCsoPoint, mSearchDirection))
      break;

    mSimplex[mSize] = newPoint;
    ++mSize;
  }
  return false;
}

bool Gjk::Intersect(const SupportShape* shapeA, const SupportShape* shapeB, Manifold* manifold)
{
  bool result = Intersect(shapeA, shapeB);
  if (result)
    return result;

  CsoPoint closestPoints;
  closestPoints.mCsoPoint = mClosestCsoPoint;
  ComputeClosestPoint(closestPoints);
  return result;
}

bool Gjk::Intersect(const SupportShape* shapeA, const SupportShape* shapeB, Real skin, Manifold* manifold)
{
  Real oldProgressEpsilon = mProgressEpsilon;
  mSkin = skin;

  bool result = Intersect(shapeA, shapeB);
  if (result)
  {
    Epa2d epa;
    epa.Run(shapeA, shapeB, mSimplex, mSize, manifold);
    return result;
  }

  CsoPoint closestPoints;
  closestPoints.mCsoPoint = mClosestCsoPoint;
  ComputeClosestPoint(closestPoints);

  Real distance = Math::Distance(closestPoints.mPointA, closestPoints.mPointB);
  if (distance < mSkin)
  {
    FilloutManifold(closestPoints, manifold, mSkin);
    return true;
  }
  return false;
}

void Gjk::FilloutManifold(const CsoPoint& closestPoints, Manifold* manifold, Real skin)
{
  manifold->mContactCount = 1;
  Manifold::ContactPoint& contactPoint = manifold->mContactPoints[0];
  contactPoint.mPointA = closestPoints.mPointB;
  contactPoint.mPointB = closestPoints.mPointA;
  contactPoint.mNormal = mSearchDirection;

  Real2 dir = contactPoint.mPointB - contactPoint.mPointA;
  Real penetration = -Math::Dot(dir, contactPoint.mNormal);
  if (penetration < 0)
  {
    contactPoint.mNormal *= -1;
  }
  
  contactPoint.mPenetration = skin - Math::Abs(penetration);
}

Gjk::CsoPoint Gjk::ComputeSupport(const SupportShape* shapeA, const SupportShape* shapeB, const Real2& direction)
{
  CsoPoint result;
  result.mPointA = shapeA->Support(direction) - direction * mSkin / 2;
  result.mPointB = shapeB->Support(-direction) + direction * mSkin / 2;
  result.mCsoPoint = result.mPointA - result.mPointB;
  return result;
}

Real2 Gjk::Intialize(const SupportShape* shapeA, const SupportShape* shapeB)
{
  mShapeA = shapeA;
  mShapeB = shapeB;

  Real2 centerA = shapeA->GetCenter();
  Real2 centerB = shapeB->GetCenter();
  return centerA - centerB;
}

void Gjk::ComputeClosestFeatureInfo(const Real2& q, Real2& searchDirection, Real2& closestPoint, size_t& newSize, int newIndices[3])
{
  if (mSize == 1)
    IdentifyVoronoiRegion(q, mSimplex[0].mCsoPoint, newSize, newIndices, closestPoint, searchDirection);
  else if (mSize == 2)
    IdentifyVoronoiRegion(q, mSimplex[0].mCsoPoint, mSimplex[1].mCsoPoint, newSize, newIndices, closestPoint, searchDirection);
  else if (mSize == 3)
    IdentifyVoronoiRegion(q, mSimplex[0].mCsoPoint, mSimplex[1].mCsoPoint, mSimplex[2].mCsoPoint, newSize, newIndices, closestPoint, searchDirection);
}

void Gjk::Reduce(size_t& newSize, int newIndices[3])
{
  // Reduce to closest feature
  for (size_t i = 0; i < newSize; ++i)
    mSimplex[i] = mSimplex[newIndices[i]];
  mSize = newSize;
}

bool Gjk::CheckProgress(const CsoPoint& newPoint, Real2& closestPoint, Real2 searchDirection)
{
  Real distance = Math::Dot(newPoint.mCsoPoint - closestPoint, searchDirection);
  return distance > mProgressEpsilon;
}

void Gjk::ComputeClosestPoint(CsoPoint& closestPoint)
{
  if (mSize == 1)
  {
    closestPoint.mPointA = mSimplex[0].mPointA;
    closestPoint.mPointB = mSimplex[0].mPointB;
  }
  else if (mSize == 2)
  {
    float u, v;
    BarycentricCoordinates(closestPoint.mCsoPoint, mSimplex[0].mCsoPoint, mSimplex[1].mCsoPoint, u, v);
    closestPoint.mPointA = mSimplex[0].mPointA * u + mSimplex[1].mPointA * v;
    closestPoint.mPointB = mSimplex[0].mPointB * u + mSimplex[1].mPointB * v;
  }
}
