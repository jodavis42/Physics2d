#pragma once

struct Sat
{
  static bool Intersect(PolygonSupportShape* shapeA, PolygonSupportShape* shapeB, Manifold* manifold);
  static Real TestAxes(PolygonSupportShape* shapeA, PolygonSupportShape* shapeB, Array<Real2>& axes, Real2& normal);
  static Real TestAxis(PolygonSupportShape* shapeA, PolygonSupportShape* shapeB, Real2Param axis);

  static Real2 GetProjectionInterval(PolygonSupportShape* shape, Real2Param axis);

  static bool IntersectPolygons(PolygonSupportShape* shapeA, PolygonSupportShape* shapeB, Manifold* manifold);
  static Real TestAxes(PolygonSupportShape* testShape, PolygonSupportShape* otherShape, Face2d& bestFace);
  static void GenerateContactInfo(Face2d& faceA, PolygonSupportShape* shapeB, Manifold* manifold, bool flipAB);
  static Real2 Clip(Face2d& face, Real2Param point, Face2d& inputFace, Real2& projPoint);
  static Face2d FindContactFace(Face2d& incidentFace, PolygonSupportShape* shape);
};
