#include "Physics2dPluginPrecompiled.hpp"

Epa2d::Epa2d()
{
  mMaxIterations = 50;
}

bool Epa2d::Run(const SupportShape* shapeA, const SupportShape* shapeB, CsoPoint points[3], size_t pointCount, Manifold* manifold)
{
  mShapeA = shapeA;
  mShapeB = shapeB;

  mPoints.Clear();
  for (size_t i = 0; i < pointCount; ++i)
    mPoints.PushBack(points[i]);
  bool success = InitializeHull();
  if (!success)
    return false;

  Real progressEpsilon = 0.001f;
  for (size_t i = 0; i < mMaxIterations; ++i)
  {
    CsoFace closestFace = FindClosestFace();
    Real2 normal = closestFace.GetNormal();

    CsoPoint newPoint = ComputeSupport(normal);
    if (!CheckEnoughProgress(newPoint, closestFace.mPoint0, normal, progressEpsilon))
      break;

    AddPoint(newPoint);
  }

  Real2 q = Real2::cZero;
  CsoFace closestFace = FindClosestFace();
  Real u, v;
  BarycentricCoordinates(q, closestFace.mPoint0.mCsoPoint, closestFace.mPoint1.mCsoPoint, u, v);
  Real2 pointA = closestFace.mPoint0.mPointA * u + closestFace.mPoint1.mPointA * v;
  Real2 pointB = closestFace.mPoint0.mPointB * u + closestFace.mPoint1.mPointB * v;

  manifold->mContactCount = 1;
  Manifold::ContactPoint& contactPoint = manifold->mContactPoints[0];
  contactPoint.mPointA = pointA;
  contactPoint.mPointB = pointB;
  contactPoint.mNormal = closestFace.GetNormal();

  Real2 dir = contactPoint.mPointB - contactPoint.mPointA;
  // Penetration is negated since the vector should be A to B but they should be flipped since they're overlapping
  Real penetration = -Math::Dot(dir, contactPoint.mNormal);
  if (penetration < 0)
  {
    contactPoint.mNormal *= -1;
  }
  contactPoint.mPenetration = Math::Abs(penetration);
  return true;
}

Real2 Epa2d::CsoFace::GetNormal()
{
  Real2 edgeDir = mPoint1.mCsoPoint - mPoint0.mCsoPoint;
  Real2 normal = Cross2d(edgeDir, 1.0f);
  return Math::AttemptNormalized(normal);
}

Epa2d::CsoPoint Epa2d::ComputeSupport(const Real2& direction)
{
  CsoPoint result;
  result.mPointA = mShapeA->Support(direction);
  result.mPointB = mShapeB->Support(-direction);
  result.mCsoPoint = result.mPointA - result.mPointB;
  return result;
}

bool Epa2d::CheckEnoughProgress(CsoPoint& newPoint, CsoPoint& planePoint, Real2Param direction, Real progressEpsilon)
{
  Real dist = Math::Dot(newPoint.mCsoPoint - planePoint.mCsoPoint, direction);
  return dist > progressEpsilon;
}

bool Epa2d::InitializeHull()
{
  Real2 q = Real2::cZero;
  switch (mPoints.Size())
  {
    case 1:
    {
      Real2 dirs[5];
      dirs[0] = q - mPoints[0].mCsoPoint;
      dirs[1] = Real2(1, 0);
      dirs[2] = Real2(-1, 0);
      dirs[3] = Real2(0, 1);
      dirs[4] = Real2(0, -1);

      for (size_t i = 0; i < 5; ++i)
      {
        if(dirs[i] == q)
          continue;

        CsoPoint newPoint = ComputeSupport(dirs[i]);
        if(newPoint.mCsoPoint == mPoints[0].mCsoPoint)
          continue;;
        mPoints.PushBack(newPoint);
        break;
      }
    }
    case 2:
    {
      Real2 edge = mPoints[0].mCsoPoint - mPoints[1].mCsoPoint;
      Math::AttemptNormalize(edge);
      Real2 dirs[2];
      dirs[0] = Real2(-edge.y, edge.x);
      dirs[1] = Real2(edge.y, -edge.x);

      for (size_t i = 0; i < 2; ++i)
      {
        CsoPoint newPoint = ComputeSupport(dirs[i]);
        if(CheckEnoughProgress(newPoint, mPoints[0], edge))
          continue;

        mPoints.PushBack(newPoint);
        break;
      }
    }
  }
  if (mPoints.Size() != 3)
    return false;

  Real area = Cross2d(mPoints[1].mCsoPoint - mPoints[0].mCsoPoint, mPoints[2].mCsoPoint - mPoints[0].mCsoPoint);
  if (area < 0)
  {
    Math::Swap(mPoints[1], mPoints[2]);
  }
  return true;
}

Epa2d::CsoFace Epa2d::FindClosestFace()
{
  Real2 q = Real2::cZero;

  Real minDistance = Math::PositiveMax();
  CsoFace bestFace;
  for (size_t i = 0; i < mPoints.Size(); ++i)
  {
    size_t j = (i + 1) % mPoints.Size();
    CsoFace face;
    face.mPoint0 = mPoints[i];
    face.mPoint1 = mPoints[j];
    
    Real2 normal = face.GetNormal();
    Real dist = Math::Dot(normal, face.mPoint0.mCsoPoint - q);
    if (dist < minDistance)
    {
      minDistance = dist;
      bestFace = face;
    }
  }
  return bestFace;
}

void Epa2d::AddPoint(const CsoPoint& newPoint)
{
  int count = 0;
  for (size_t i = 0; i < mPoints.Size(); ++i)
  {
    size_t j = (i + 1) % mPoints.Size();
    CsoFace face;
    face.mPoint0 = mPoints[i];
    face.mPoint1 = mPoints[j];

    Real dist = Math::Dot(face.GetNormal(), newPoint.mCsoPoint - face.mPoint0.mCsoPoint);
    if (dist > 0)
    {
      ++count;
      mPoints.InsertAt(j, newPoint);
      break;
    }
  }
}
