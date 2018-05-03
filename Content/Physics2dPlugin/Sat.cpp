#include "Physics2dPluginPrecompiled.hpp"

bool Sat::Intersect(PolygonSupportShape* shapeA, PolygonSupportShape* shapeB, Manifold* manifold)
{
  Array<Real2> axesA, axesB;
  shapeA->GetFaceNormals(axesA);

  Real2 normalA;
  Real minProjA = TestAxes(shapeA, shapeB, axesA, normalA);
  if (minProjA < 0)
    return false;

  Real2 normalB;
  shapeB->GetFaceNormals(axesB);
  Real minProjB = TestAxes(shapeA, shapeB, axesB, normalB);
  if (minProjB < 0)
    return false;

  Real2 normal = normalA;
  if (minProjB < minProjA)
    normal = normalB;

  Real2 centerA = shapeA->GetCenter();
  Real2 centerB = shapeB->GetCenter();
  if (Math::Dot(normal, centerB - centerA) < 0)
    normal *= -1;

  manifold->mContactPoints[0].mNormal = normal;
  manifold->mContactPoints[0].mPenetration = Math::Min(minProjA, minProjB);
  manifold->mContactCount = 1;

  return true;
}

Real Sat::TestAxes(PolygonSupportShape* shapeA, PolygonSupportShape* shapeB, Array<Real2>& axes, Real2& normal)
{
  Real minPenetration = Math::PositiveMax();
  for (size_t i = 0; i < axes.Size(); ++i)
  {
    Real proj = TestAxis(shapeA, shapeB, axes[i]);
    if (proj < 0)
      return proj;

    if (proj < minPenetration)
    {
      minPenetration = proj;
      normal = axes[i];
    }
  }
  return minPenetration;
}

Real Sat::TestAxis(PolygonSupportShape* shapeA, PolygonSupportShape* shapeB, Real2Param axis)
{
  Real2 aInterval = GetProjectionInterval(shapeA, axis);
  Real2 bInterval = GetProjectionInterval(shapeB, axis);

  Real aMin = aInterval.x, aMax = aInterval.y;
  Real bMin = bInterval.x, bMax = bInterval.y;

  Real min = Math::Max(aMin, bMin);
  Real max = Math::Min(aMax, bMax);

  return max - min;
}

Real2 Sat::GetProjectionInterval(PolygonSupportShape* shape, Real2Param axis)
{
  Real2 min = shape->Support(-axis);
  Real2 max = shape->Support(axis);

  Real2 result;
  result.x = Math::Dot(min, axis);
  result.y = Math::Dot(max, axis);
  return result;
}

bool Sat::IntersectPolygons(PolygonSupportShape* shapeA, PolygonSupportShape* shapeB, Manifold* manifold)
{
  Real penetrationA, penetrationB;
  Face2d faceA, faceB;

  penetrationA = TestAxes(shapeA, shapeB, faceA);
  if (penetrationA < 0)
    return false;

  penetrationB = TestAxes(shapeB, shapeA, faceB);
  if (penetrationB < 0)
    return false;

  Real penetration = penetrationA;
  Face2d face = faceA;
  PolygonSupportShape* shape = shapeA;
  PolygonSupportShape* otherShape = shapeB;
  if (penetrationB < penetration)
  {
    penetration = penetrationB;
    face = faceB;
    shape = shapeB;
    otherShape = shapeA;
  }
  bool flipAB = otherShape == shapeA;
  GenerateContactInfo(face, otherShape, manifold, flipAB);

  return true;
}

Real Sat::TestAxes(PolygonSupportShape* testShape, PolygonSupportShape* otherShape, Face2d& bestFace)
{
  Real minPenetration = Math::PositiveMax();

  Array<Face2d> faces;
  testShape->GetFaces(faces);
  for (size_t i = 0; i < faces.Size(); ++i)
  {
    Face2d face = faces[i];
    Real2 normal = face.GetNormal();
    Real2 otherPoint = otherShape->Support(-normal);
    Real separation = Math::Dot(normal, otherPoint - face.mPoint0);
    Real penetration = -separation;
    if (penetration < 0)
      return penetration;

    if (penetration < minPenetration)
    {
      minPenetration = penetration;
      bestFace = face;
    }
  }
  return minPenetration;
}

void Sat::GenerateContactInfo(Face2d& faceA, PolygonSupportShape* shapeB, Manifold* manifold, bool flipAB)
{
  Real2 normalA = faceA.GetNormal();
  Face2d faceB = FindContactFace(faceA, shapeB);

  Real2 projP0, projP1;
  Real2 p0 = Clip(faceA, faceB.mPoint0, faceB, projP0);
  Real2 p1 = Clip(faceA, faceB.mPoint1, faceB, projP1);

  Real2 p0B = p0;
  Real2 p1B = p1;
  Real2 p0A = projP0;
  Real2 p1A = projP1;

  IntersectionType::Enum p0BState = PointPlane(p0B, faceA.mPoint0, normalA, 0);
  IntersectionType::Enum p1BState = PointPlane(p1B, faceA.mPoint0, normalA, 0);

  if (flipAB)
  {
    Math::Swap(p0A, p0B);
    Math::Swap(p1A, p1B);
    normalA *= -1;
  }

  manifold->mContactCount = 0;
  if (p0BState == IntersectionType::Outside)
  {
    int i = manifold->mContactCount;
    ++manifold->mContactCount;
    manifold->mContactPoints[i].mNormal = normalA;
    manifold->mContactPoints[i].mPointA = p0A;
    manifold->mContactPoints[i].mPointB = p0B;
    manifold->mContactPoints[i].mPenetration = Math::Dot(p0A - p0B, normalA);
  }

  if (p1BState == IntersectionType::Outside)
  {
    int i = manifold->mContactCount;
    ++manifold->mContactCount;
    manifold->mContactPoints[i].mNormal = normalA;
    manifold->mContactPoints[i].mPointA = p1A;
    manifold->mContactPoints[i].mPointB = p1B;
    manifold->mContactPoints[i].mPenetration = Math::Dot(p1A - p1B, normalA);
  }
}

Real2 Sat::Clip(Face2d& face, Real2Param point, Face2d& inputFace, Real2& projPoint)
{
  Real u, v;
  bool contained = BarycentricCoordinates(point, face.mPoint0, face.mPoint1, u, v);
  if (contained)
  {
    projPoint = u * face.mPoint0 + v * face.mPoint1;
    return point;
  }

  u = Math::Clamp(u);
  v = Math::Clamp(v);

  projPoint = u * face.mPoint0 + v * face.mPoint1;
  BarycentricCoordinates(projPoint, inputFace.mPoint0, inputFace.mPoint1, u, v);
  return u * inputFace.mPoint0 + v * inputFace.mPoint1;
}

Face2d Sat::FindContactFace(Face2d& incidentFace, PolygonSupportShape* shape)
{
  Face2d result;
  Real direction = 1;
  Real2 normal = incidentFace.GetNormal();
  Array<Face2d> faces;
  shape->GetFaces(faces);
  for (size_t i = 0; i < faces.Size(); ++i)
  {
    Face2d face = faces[i];
    Real2 faceNormal = face.GetNormal();
    Real dot = Math::Dot(faceNormal, normal);
    if (dot < direction)
    {
      direction = dot;
      result = face;
    }
  }
  return result;
}
