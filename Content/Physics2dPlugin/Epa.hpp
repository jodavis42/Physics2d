#pragma once

class Gjk;

class Epa2d
{
public:
  typedef Gjk::CsoPoint CsoPoint;

  Epa2d();
  bool Run(const SupportShape* shapeA, const SupportShape* shapeB, CsoPoint points[3], size_t pointCount, Manifold* manifold);

  size_t mMaxIterations;
private:
  struct CsoFace
  {
    Real2 GetNormal();
    CsoPoint mPoint0;
    CsoPoint mPoint1;
  };

  CsoPoint ComputeSupport(const Real2& direction);
  bool CheckEnoughProgress(CsoPoint& newPoint, CsoPoint& planePoint, Real2Param direction, Real progressEpsilon = 0);
  bool InitializeHull();
  CsoFace FindClosestFace();
  void AddPoint(const CsoPoint& newPoint);
  
  Array<CsoPoint> mPoints;
  const SupportShape* mShapeA;
  const SupportShape* mShapeB;
};