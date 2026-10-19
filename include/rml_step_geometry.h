#ifndef RML_STEP_GEOMETRY_H
#define RML_STEP_GEOMETRY_H

#include <array>
#include <functional>
#include <memory>
#include <vector>

#include "rml_step_file.h"

//! 3D vector used by STEP geometry.
struct RStepVector
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    RStepVector() = default;
    RStepVector(double x, double y, double z) : x(x), y(y), z(z) {}

    RStepVector operator+(const RStepVector &v) const { return RStepVector(x+v.x,y+v.y,z+v.z); }
    RStepVector operator-(const RStepVector &v) const { return RStepVector(x-v.x,y-v.y,z-v.z); }
    RStepVector operator*(double s) const { return RStepVector(x*s,y*s,z*s); }
    RStepVector operator-() const { return RStepVector(-x,-y,-z); }
    RStepVector &operator+=(const RStepVector &v) { x+=v.x; y+=v.y; z+=v.z; return *this; }

    double dot(const RStepVector &v) const { return x*v.x + y*v.y + z*v.z; }
    RStepVector cross(const RStepVector &v) const { return RStepVector(y*v.z-z*v.y,z*v.x-x*v.z,x*v.y-y*v.x); }
    double length() const;
    RStepVector normalized() const;
};

//! Rigid (optionally scaled) transformation p' = R * s * p + t.
struct RStepTransform
{
    //! Rotation matrix rows.
    std::array<RStepVector,3> r = {RStepVector(1,0,0),RStepVector(0,1,0),RStepVector(0,0,1)};
    //! Translation.
    RStepVector t;
    //! Uniform scale.
    double s = 1.0;

    //! Transform point.
    RStepVector apply(const RStepVector &p) const;

    //! Return composition (this * other) - other is applied first.
    RStepTransform operator*(const RStepTransform &other) const;

    //! Return inverse transformation.
    RStepTransform inverse() const;

    //! Return true if transformation is identity.
    bool isIdentity() const;

    //! Return true if transformation changes orientation.
    bool isMirror() const;
};

//! Coordinate system of elementary geometry (AXIS2_PLACEMENT_3D).
struct RStepPlacement
{
    RStepVector origin;
    RStepVector x = RStepVector(1,0,0);
    RStepVector y = RStepVector(0,1,0);
    RStepVector z = RStepVector(0,0,1);

    //! Return transformation from local to global coordinates.
    RStepTransform toTransform() const;
};

//! B-spline data shared by curves and surfaces.
struct RStepBSpline
{
    //! Degree.
    int degree = 0;
    //! Expanded knot vector.
    std::vector<double> knots;

    //! Return first parameter of valid domain.
    double getMin() const;

    //! Return last parameter of valid domain.
    double getMax() const;

    //! Find knot span.
    int findSpan(int nControlPoints, double t) const;

    //! Compute non-zero basis functions for given span.
    void basisFunctions(int span, double t, double *values) const;

    //! Return number of distinct knot spans in domain.
    int getNSpans() const;
};

//! Parametric curve.
class RStepCurve
{

    public:

        enum Kind
        {
            None = 0,
            Line,
            Circle,
            Ellipse,
            BSpline,
            Polyline
        };

    public:

        Kind kind;
        //! Line origin, circle / ellipse placement.
        RStepPlacement placement;
        //! Line direction (including magnitude).
        RStepVector direction;
        //! Radius / semi axes.
        double radius1;
        double radius2;
        //! B-spline curve.
        RStepBSpline bspline;
        //! Control or polyline points.
        std::vector<RStepVector> points;
        //! Weights of rational B-spline (empty if not rational).
        std::vector<double> weights;
        //! Closed curve.
        bool periodic;
        //! Period length.
        double period;

    public:

        //! Constructor.
        RStepCurve();

        //! Return true if curve is valid.
        bool isValid() const;

        //! Return parameter domain.
        double getMin() const;
        double getMax() const;

        //! Evaluate curve point.
        RStepVector eval(double t) const;

        //! Return parameter of nearest curve point.
        double project(const RStepVector &p) const;

        //! Return initial number of sampling intervals for parameter range.
        int getNSamples(double t1, double t2, double maxAngle) const;

        //! Build curve from STEP entity.
        //! Return false if curve type is not supported.
        static bool build(const RStepFile &file, unsigned int id, RStepCurve &curve);

};

//! Parametric surface.
class RStepSurface
{

    public:

        enum Kind
        {
            None = 0,
            Plane,
            Cylinder,
            Cone,
            Sphere,
            Torus,
            BSpline,
            Extrusion,
            Revolution,
            Offset
        };

    public:

        Kind kind;
        RStepPlacement placement;
        //! Radius (cylinder, cone, sphere) or major radius (torus).
        double radius1;
        //! Minor radius (torus).
        double radius2;
        //! Tangent of cone semi angle.
        double tanAngle;
        //! B-spline surface.
        RStepBSpline bsplineU;
        RStepBSpline bsplineV;
        int nu;
        int nv;
        //! Control points - nu x nv, row major.
        std::vector<RStepVector> points;
        std::vector<double> weights;
        //! Swept curve of extrusion / revolution.
        RStepCurve curve;
        //! Extrusion vector or revolution axis direction.
        RStepVector direction;
        //! Basis surface of offset surface.
        std::shared_ptr<RStepSurface> basis;
        //! Offset distance.
        double distance;
        //! Periodicity.
        bool periodic[2];
        double period[2];
        //! Bounded parameter domain.
        bool bounded[2];
        double domainMin[2];
        double domainMax[2];
        //! Control net extent (B-spline).
        double extent;
        //! Sample grid for projection (B-spline).
        std::vector<RStepVector> grid;
        std::vector<std::array<double,2>> gridUV;

    public:

        //! Constructor.
        RStepSurface();

        //! Return true if surface is valid.
        bool isValid() const;

        //! Evaluate surface point.
        RStepVector eval(double u, double v) const;

        //! Evaluate partial derivatives (numerically for free form surfaces).
        void derivatives(double u, double v, RStepVector &du, RStepVector &dv) const;

        //! Return surface normal (not oriented by face sense).
        RStepVector normal(double u, double v) const;

        //! Return parameters of nearest surface point.
        //! If hint is used, given parameters are used as initial guess.
        void project(const RStepVector &p, double &u, double &v, bool useHint = false) const;

        //! Refine parameters of nearest surface point by Gauss-Newton iterations.
        //! Return distance to surface.
        double refineProjection(const RStepVector &p, double &u, double &v) const;

        //! Return true if surface curvature can be neglected.
        bool isFlat() const;

        //! Return minimal curvature radius (analytic surfaces) or -1 if unknown.
        double getCurvatureRadius() const;

        //! Prepare projection acceleration data.
        void prepare();

        //! Build surface from STEP entity.
        //! Return false if surface type is not supported.
        static bool build(const RStepFile &file, unsigned int id, double angleScale, RStepSurface &surface);

};

//! STEP geometry helper functions.
namespace RStepGeometry
{

    //! Read CARTESIAN_POINT.
    bool readPoint(const RStepFile &file, unsigned int id, RStepVector &point);

    //! Read DIRECTION.
    bool readDirection(const RStepFile &file, unsigned int id, RStepVector &direction);

    //! Read VERTEX_POINT location.
    bool readVertex(const RStepFile &file, unsigned int id, RStepVector &point);

    //! Read AXIS2_PLACEMENT_3D (or AXIS1_PLACEMENT).
    bool readPlacement(const RStepFile &file, unsigned int id, RStepPlacement &placement);

    //! Read transformation from AXIS2_PLACEMENT_3D or CARTESIAN_TRANSFORMATION_OPERATOR_3D.
    bool readTransform(const RStepFile &file, unsigned int id, RStepTransform &transform);

    //! Minimize function on interval using sampling and golden section search.
    double minimize(const std::function<double(double)> &function, double a, double b, int nSamples);

}

#endif // RML_STEP_GEOMETRY_H
