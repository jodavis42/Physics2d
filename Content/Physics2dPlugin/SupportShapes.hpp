#pragma once


class Collider2d;
class SphereCollider2d;
class BoxCollider2d;

struct Face2d
{
  Face2d() {}
  Face2d(Real2Param p0, Real2Param p1)
  {
    mPoint0 = p0;
    mPoint1 = p1;
  }
  Real2 GetNormal() const;

  Real2 mPoint0;
  Real2 mPoint1;
};

struct SupportShape
{
  virtual Real2 GetCenter() const = 0;
  virtual Real2 Support(Real2Param dir) const = 0;
};

struct PolygonSupportShape : public SupportShape
{
  virtual void GetFaceNormals(Array<Real2>& axes) = 0;
  virtual void GetFaces(Array<Face2d>& faces) = 0;
};

struct SphereSupportShape : public SupportShape
{
  SphereSupportShape(SphereCollider2d* collider);

  Real2 GetCenter() const override;
  Real2 Support(Real2Param dir) const override;

  Real2 mPosition;
  Real mRadius;
};

struct BoxSupportShape : public PolygonSupportShape
{
  BoxSupportShape(BoxCollider2d* collider);

  Real2 GetCenter() const override;
  Real2 Support(Real2Param dir) const override;

  void GetFaceNormals(Array<Real2>& axes) override;
  void GetFaces(Array<Face2d>& faces) override;

  BoxCollider2d* mCollider;

  Real2x2 mInvRotation;
  Real2x2 mRotation;
};