#include <algorithm>
#include <cmath>
#include <limits>

#include "rml_step_geometry.h"

static const double stepPi = 3.14159265358979323846;
static const double stepTwoPi = 2.0 * stepPi;
static const double stepInfinity = std::numeric_limits<double>::infinity();

double RStepVector::length() const
{
    return std::sqrt(x*x + y*y + z*z);
}

RStepVector RStepVector::normalized() const
{
    double l = this->length();
    return (l > 0.0) ? (*this) * (1.0 / l) : RStepVector();
}

RStepVector RStepTransform::apply(const RStepVector &p) const
{
    return RStepVector(this->r[0].dot(p),this->r[1].dot(p),this->r[2].dot(p)) * this->s + this->t;
}

RStepTransform RStepTransform::operator*(const RStepTransform &other) const
{
    RStepTransform result;
    RStepVector c0(other.r[0].x,other.r[1].x,other.r[2].x);
    RStepVector c1(other.r[0].y,other.r[1].y,other.r[2].y);
    RStepVector c2(other.r[0].z,other.r[1].z,other.r[2].z);
    for (int i=0;i<3;i++)
    {
        result.r[i] = RStepVector(this->r[i].dot(c0),this->r[i].dot(c1),this->r[i].dot(c2));
    }
    result.s = this->s * other.s;
    result.t = RStepVector(this->r[0].dot(other.t),this->r[1].dot(other.t),this->r[2].dot(other.t)) * this->s + this->t;
    return result;
}

RStepTransform RStepTransform::inverse() const
{
    RStepTransform result;
    result.r[0] = RStepVector(this->r[0].x,this->r[1].x,this->r[2].x);
    result.r[1] = RStepVector(this->r[0].y,this->r[1].y,this->r[2].y);
    result.r[2] = RStepVector(this->r[0].z,this->r[1].z,this->r[2].z);
    result.s = (this->s != 0.0) ? 1.0 / this->s : 1.0;
    result.t = RStepVector(result.r[0].dot(this->t),result.r[1].dot(this->t),result.r[2].dot(this->t)) * (-result.s);
    return result;
}

bool RStepTransform::isIdentity() const
{
    const double eps = 1e-12;
    return (std::abs(this->r[0].x - 1.0) < eps && std::abs(this->r[1].y - 1.0) < eps && std::abs(this->r[2].z - 1.0) < eps &&
            std::abs(this->r[0].y) < eps && std::abs(this->r[0].z) < eps &&
            std::abs(this->r[1].x) < eps && std::abs(this->r[1].z) < eps &&
            std::abs(this->r[2].x) < eps && std::abs(this->r[2].y) < eps &&
            this->t.length() < eps && std::abs(this->s - 1.0) < eps);
}

bool RStepTransform::isMirror() const
{
    return (this->r[0].dot(this->r[1].cross(this->r[2])) * this->s < 0.0);
}

RStepTransform RStepPlacement::toTransform() const
{
    RStepTransform transform;
    transform.r[0] = RStepVector(this->x.x,this->y.x,this->z.x);
    transform.r[1] = RStepVector(this->x.y,this->y.y,this->z.y);
    transform.r[2] = RStepVector(this->x.z,this->y.z,this->z.z);
    transform.t = this->origin;
    return transform;
}

double RStepBSpline::getMin() const
{
    return this->knots.empty() ? 0.0 : this->knots[this->degree];
}

double RStepBSpline::getMax() const
{
    return this->knots.empty() ? 1.0 : this->knots[this->knots.size() - this->degree - 1];
}

int RStepBSpline::findSpan(int nControlPoints, double t) const
{
    int n = nControlPoints - 1;
    if (t >= this->knots[n+1])
    {
        // Last non-empty span.
        int span = n;
        while (span > this->degree && this->knots[span] >= this->knots[span+1])
        {
            span--;
        }
        return span;
    }
    if (t <= this->knots[this->degree])
    {
        int span = this->degree;
        while (span < n && this->knots[span] >= this->knots[span+1])
        {
            span++;
        }
        return span;
    }
    int low = this->degree;
    int high = n + 1;
    int mid = (low + high) / 2;
    while (t < this->knots[mid] || t >= this->knots[mid+1])
    {
        if (t < this->knots[mid])
        {
            high = mid;
        }
        else
        {
            low = mid;
        }
        mid = (low + high) / 2;
    }
    return mid;
}

void RStepBSpline::basisFunctions(int span, double t, double *values) const
{
    double left[32];
    double right[32];
    values[0] = 1.0;
    for (int j=1;j<=this->degree;j++)
    {
        left[j] = t - this->knots[span+1-j];
        right[j] = this->knots[span+j] - t;
        double saved = 0.0;
        for (int r=0;r<j;r++)
        {
            double denominator = right[r+1] + left[j-r];
            double temp = (denominator != 0.0) ? values[r] / denominator : 0.0;
            values[r] = saved + right[r+1] * temp;
            saved = left[j-r] * temp;
        }
        values[j] = saved;
    }
}

int RStepBSpline::getNSpans() const
{
    int nSpans = 0;
    double tMin = this->getMin();
    double tMax = this->getMax();
    for (size_t i=0;i+1<this->knots.size();i++)
    {
        if (this->knots[i+1] > this->knots[i] && this->knots[i] >= tMin && this->knots[i+1] <= tMax)
        {
            nSpans++;
        }
    }
    return std::max(nSpans,1);
}

namespace
{

//! Wrap parameter of periodic B-spline into its domain.
double wrapPeriodic(double t, double tMin, double period)
{
    if (period <= 0.0)
    {
        return t;
    }
    double f = std::fmod(t - tMin,period);
    if (f < 0.0)
    {
        f += period;
    }
    return tMin + f;
}

//! Build expanded knot vector.
bool buildKnots(const RStepValue &multiplicities, const RStepValue &knots, int degree, int nControlPoints, RStepBSpline &bspline)
{
    bspline.degree = degree;
    bspline.knots.clear();
    for (size_t i=0;i<knots.size() && i<multiplicities.size();i++)
    {
        int m = multiplicities.at(i).toInt();
        for (int j=0;j<m;j++)
        {
            bspline.knots.push_back(knots.at(i).toDouble());
        }
    }
    return (degree >= 1 && degree < 30 && int(bspline.knots.size()) == nControlPoints + degree + 1);
}

//! Build implicit knot vector of Bezier, uniform or quasi uniform B-spline.
void buildImplicitKnots(const QString &type, int degree, int nControlPoints, RStepBSpline &bspline)
{
    bspline.degree = degree;
    bspline.knots.clear();
    int nKnots = nControlPoints + degree + 1;
    if (type == "UNIFORM")
    {
        for (int i=0;i<nKnots;i++)
        {
            bspline.knots.push_back(double(i - degree));
        }
    }
    else
    {
        // Bezier and quasi uniform curves are clamped.
        int nInner = nKnots - 2 * (degree + 1);
        for (int i=0;i<=degree;i++)
        {
            bspline.knots.push_back(0.0);
        }
        for (int i=1;i<=nInner;i++)
        {
            bspline.knots.push_back(double(i));
        }
        for (int i=0;i<=degree;i++)
        {
            bspline.knots.push_back(double(nInner + 1));
        }
    }
}

//! Find implicit B-spline knot type in entity.
QString findImplicitKnotType(const RStepEntity &entity)
{
    if (entity.has("BEZIER_CURVE") || entity.has("BEZIER_SURFACE"))
    {
        return "BEZIER";
    }
    if (entity.has("UNIFORM_CURVE") || entity.has("UNIFORM_SURFACE"))
    {
        return "UNIFORM";
    }
    if (entity.has("QUASI_UNIFORM_CURVE") || entity.has("QUASI_UNIFORM_SURFACE"))
    {
        return "QUASI_UNIFORM";
    }
    return QString();
}

bool readPointList(const RStepFile &file, const RStepValue &list, std::vector<RStepVector> &points)
{
    points.clear();
    for (size_t i=0;i<list.size();i++)
    {
        RStepVector point;
        if (!RStepGeometry::readPoint(file,list.at(i).toId(),point))
        {
            return false;
        }
        points.push_back(point);
    }
    return true;
}

} // namespace

RStepCurve::RStepCurve()
    : kind(None)
    , radius1(0.0)
    , radius2(0.0)
    , periodic(false)
    , period(0.0)
{

}

bool RStepCurve::isValid() const
{
    return (this->kind != None);
}

double RStepCurve::getMin() const
{
    switch (this->kind)
    {
        case Circle:
        case Ellipse:
            return 0.0;
        case BSpline:
            return this->bspline.getMin();
        case Polyline:
            return 0.0;
        default:
            return -stepInfinity;
    }
}

double RStepCurve::getMax() const
{
    switch (this->kind)
    {
        case Circle:
        case Ellipse:
            return stepTwoPi;
        case BSpline:
            return this->bspline.getMax();
        case Polyline:
            return double(std::max(int(this->points.size()) - 1,0));
        default:
            return stepInfinity;
    }
}

RStepVector RStepCurve::eval(double t) const
{
    switch (this->kind)
    {
        case Line:
        {
            return this->placement.origin + this->direction * t;
        }
        case Circle:
        {
            return this->placement.origin + (this->placement.x * std::cos(t) + this->placement.y * std::sin(t)) * this->radius1;
        }
        case Ellipse:
        {
            return this->placement.origin + this->placement.x * (this->radius1 * std::cos(t)) + this->placement.y * (this->radius2 * std::sin(t));
        }
        case BSpline:
        {
            int n = int(this->points.size());
            t = this->periodic ? wrapPeriodic(t,this->bspline.getMin(),this->period) : std::clamp(t,this->bspline.getMin(),this->bspline.getMax());
            int span = this->bspline.findSpan(n,t);
            double basis[32];
            this->bspline.basisFunctions(span,t,basis);
            RStepVector point;
            double w = 0.0;
            for (int i=0;i<=this->bspline.degree;i++)
            {
                int k = span - this->bspline.degree + i;
                double b = basis[i] * (this->weights.empty() ? 1.0 : this->weights[k]);
                point += this->points[k] * b;
                w += b;
            }
            return (w != 0.0) ? point * (1.0 / w) : point;
        }
        case Polyline:
        {
            int n = int(this->points.size());
            if (n == 0)
            {
                return RStepVector();
            }
            t = std::clamp(t,0.0,double(n-1));
            int i = std::min(int(t),n-2);
            if (i < 0)
            {
                return this->points[0];
            }
            double f = t - double(i);
            return this->points[i] * (1.0 - f) + this->points[i+1] * f;
        }
        default:
        {
            return RStepVector();
        }
    }
}

double RStepCurve::project(const RStepVector &p) const
{
    switch (this->kind)
    {
        case Line:
        {
            double l2 = this->direction.dot(this->direction);
            return (l2 > 0.0) ? (p - this->placement.origin).dot(this->direction) / l2 : 0.0;
        }
        case Circle:
        case Ellipse:
        {
            RStepVector d = p - this->placement.origin;
            double a = (this->kind == Circle) ? 1.0 : this->radius1;
            double b = (this->kind == Circle) ? 1.0 : this->radius2;
            double t = std::atan2(d.dot(this->placement.y) / b,d.dot(this->placement.x) / a);
            return (t < 0.0) ? t + stepTwoPi : t;
        }
        case BSpline:
        {
            int nSamples = std::max(16,this->bspline.getNSpans() * (this->bspline.degree + 1) * 2);
            return RStepGeometry::minimize([&](double t) { RStepVector d = this->eval(t) - p; return d.dot(d); },
                                           this->bspline.getMin(),this->bspline.getMax(),nSamples);
        }
        case Polyline:
        {
            double bestT = 0.0;
            double bestDistance = stepInfinity;
            for (size_t i=0;i+1<this->points.size();i++)
            {
                RStepVector s = this->points[i+1] - this->points[i];
                double l2 = s.dot(s);
                double f = (l2 > 0.0) ? std::clamp((p - this->points[i]).dot(s) / l2,0.0,1.0) : 0.0;
                RStepVector d = this->points[i] + s * f - p;
                if (d.dot(d) < bestDistance)
                {
                    bestDistance = d.dot(d);
                    bestT = double(i) + f;
                }
            }
            return bestT;
        }
        default:
        {
            return 0.0;
        }
    }
}

int RStepCurve::getNSamples(double t1, double t2, double maxAngle) const
{
    double dt = std::abs(t2 - t1);
    switch (this->kind)
    {
        case Line:
        {
            return 1;
        }
        case Circle:
        case Ellipse:
        {
            return std::max(1,int(std::ceil(dt / maxAngle - 1e-9)));
        }
        case BSpline:
        {
            double tMin = std::min(t1,t2);
            double tMax = std::max(t1,t2);
            int nSpans = 1;
            for (size_t i=0;i<this->bspline.knots.size();i++)
            {
                if (this->bspline.knots[i] > tMin && this->bspline.knots[i] < tMax &&
                    (i == 0 || this->bspline.knots[i] != this->bspline.knots[i-1]))
                {
                    nSpans++;
                }
            }
            return std::max(2,nSpans * std::max(2,this->bspline.degree));
        }
        case Polyline:
        {
            return std::max(1,int(std::ceil(dt)));
        }
        default:
        {
            return 1;
        }
    }
}

bool RStepCurve::build(const RStepFile &file, unsigned int id, RStepCurve &curve)
{
    const RStepEntity *pEntity = file.find(id);
    if (!pEntity)
    {
        return false;
    }
    const RStepEntity &entity = *pEntity;
    const QString &keyword = entity.keyword();

    curve = RStepCurve();

    if (keyword == "LINE")
    {
        const RStepRecord &record = entity.records[0];
        const RStepEntity *pVector = file.find(record.param(2).toId());
        if (!pVector || pVector->keyword() != "VECTOR")
        {
            return false;
        }
        RStepVector direction;
        if (!RStepGeometry::readPoint(file,record.param(1).toId(),curve.placement.origin) ||
            !RStepGeometry::readDirection(file,pVector->records[0].param(1).toId(),direction))
        {
            return false;
        }
        double magnitude = pVector->records[0].param(2).toDouble(1.0);
        curve.direction = direction * ((magnitude > 0.0) ? magnitude : 1.0);
        curve.kind = Line;
        return true;
    }
    if (keyword == "CIRCLE" || keyword == "ELLIPSE")
    {
        const RStepRecord &record = entity.records[0];
        if (!RStepGeometry::readPlacement(file,record.param(1).toId(),curve.placement))
        {
            return false;
        }
        curve.radius1 = record.param(2).toDouble();
        curve.radius2 = (keyword == "ELLIPSE") ? record.param(3).toDouble() : curve.radius1;
        if (curve.radius1 <= 0.0 || curve.radius2 <= 0.0)
        {
            return false;
        }
        curve.kind = (keyword == "CIRCLE") ? Circle : Ellipse;
        curve.periodic = true;
        curve.period = stepTwoPi;
        return true;
    }
    if (keyword == "POLYLINE")
    {
        if (!readPointList(file,entity.records[0].param(1),curve.points) || curve.points.size() < 2)
        {
            return false;
        }
        curve.kind = Polyline;
        return true;
    }
    if (keyword == "TRIMMED_CURVE")
    {
        return RStepCurve::build(file,entity.records[0].param(1).toId(),curve);
    }
    if (keyword == "SURFACE_CURVE" || keyword == "SEAM_CURVE" || keyword == "INTERSECTION_CURVE" || keyword == "BOUNDED_SURFACE_CURVE")
    {
        return RStepCurve::build(file,entity.records[0].param(1).toId(),curve);
    }

    // B-spline curves - simple or complex instances.
    const RStepRecord *pWithKnots = entity.find("B_SPLINE_CURVE_WITH_KNOTS");
    QString implicitType = findImplicitKnotType(entity);
    if (pWithKnots || !implicitType.isEmpty())
    {
        int degree = 0;
        RStepValue controlPoints;
        bool closed = false;
        RStepValue multiplicities;
        RStepValue knots;

        if (!entity.isComplex())
        {
            const RStepRecord &record = entity.records[0];
            degree = record.param(1).toInt();
            controlPoints = record.param(2);
            closed = record.param(4).toBool(false);
            if (pWithKnots)
            {
                multiplicities = record.param(6);
                knots = record.param(7);
            }
        }
        else
        {
            const RStepRecord *pBase = entity.find("B_SPLINE_CURVE");
            if (!pBase)
            {
                return false;
            }
            degree = pBase->param(0).toInt();
            controlPoints = pBase->param(1);
            closed = pBase->param(3).toBool(false);
            if (pWithKnots)
            {
                multiplicities = pWithKnots->param(0);
                knots = pWithKnots->param(1);
            }
            if (const RStepRecord *pRational = entity.find("RATIONAL_B_SPLINE_CURVE"))
            {
                curve.weights = pRational->param(0).toDoubleList();
            }
        }

        if (!readPointList(file,controlPoints,curve.points) || int(curve.points.size()) <= degree || degree < 1)
        {
            return false;
        }
        if (pWithKnots)
        {
            if (!buildKnots(multiplicities,knots,degree,int(curve.points.size()),curve.bspline))
            {
                return false;
            }
        }
        else
        {
            buildImplicitKnots(implicitType,degree,int(curve.points.size()),curve.bspline);
        }
        if (!curve.weights.empty() && curve.weights.size() != curve.points.size())
        {
            return false;
        }
        curve.kind = BSpline;
        double tMin = curve.bspline.getMin();
        double tMax = curve.bspline.getMax();
        RStepVector d = curve.eval(tMin) - curve.eval(tMax);
        RStepVector extent = curve.eval(0.5 * (tMin + tMax)) - curve.eval(tMin);
        if (closed || d.length() <= 1e-9 * std::max(extent.length(),1e-12))
        {
            curve.periodic = (d.length() <= 1e-6 * std::max(extent.length(),1e-12));
            curve.period = tMax - tMin;
        }
        return true;
    }

    return false;
}

RStepSurface::RStepSurface()
    : kind(None)
    , radius1(0.0)
    , radius2(0.0)
    , tanAngle(0.0)
    , nu(0)
    , nv(0)
    , distance(0.0)
    , extent(0.0)
    , periodic{false,false}
    , period{0.0,0.0}
    , bounded{false,false}
    , domainMin{-stepInfinity,-stepInfinity}
    , domainMax{stepInfinity,stepInfinity}
{

}

bool RStepSurface::isValid() const
{
    return (this->kind != None);
}

RStepVector RStepSurface::eval(double u, double v) const
{
    const RStepPlacement &pl = this->placement;
    switch (this->kind)
    {
        case Plane:
        {
            return pl.origin + pl.x * u + pl.y * v;
        }
        case Cylinder:
        {
            return pl.origin + (pl.x * std::cos(u) + pl.y * std::sin(u)) * this->radius1 + pl.z * v;
        }
        case Cone:
        {
            return pl.origin + (pl.x * std::cos(u) + pl.y * std::sin(u)) * (this->radius1 + v * this->tanAngle) + pl.z * v;
        }
        case Sphere:
        {
            return pl.origin + (pl.x * std::cos(u) + pl.y * std::sin(u)) * (this->radius1 * std::cos(v)) + pl.z * (this->radius1 * std::sin(v));
        }
        case Torus:
        {
            return pl.origin + (pl.x * std::cos(u) + pl.y * std::sin(u)) * (this->radius1 + this->radius2 * std::cos(v)) + pl.z * (this->radius2 * std::sin(v));
        }
        case BSpline:
        {
            u = this->periodic[0] ? wrapPeriodic(u,this->domainMin[0],this->period[0]) : std::clamp(u,this->domainMin[0],this->domainMax[0]);
            v = this->periodic[1] ? wrapPeriodic(v,this->domainMin[1],this->period[1]) : std::clamp(v,this->domainMin[1],this->domainMax[1]);
            int spanU = this->bsplineU.findSpan(this->nu,u);
            int spanV = this->bsplineV.findSpan(this->nv,v);
            double basisU[32];
            double basisV[32];
            this->bsplineU.basisFunctions(spanU,u,basisU);
            this->bsplineV.basisFunctions(spanV,v,basisV);
            RStepVector point;
            double w = 0.0;
            for (int i=0;i<=this->bsplineU.degree;i++)
            {
                int ki = spanU - this->bsplineU.degree + i;
                for (int j=0;j<=this->bsplineV.degree;j++)
                {
                    int k = ki * this->nv + spanV - this->bsplineV.degree + j;
                    double b = basisU[i] * basisV[j] * (this->weights.empty() ? 1.0 : this->weights[k]);
                    point += this->points[k] * b;
                    w += b;
                }
            }
            return (w != 0.0) ? point * (1.0 / w) : point;
        }
        case Extrusion:
        {
            return this->curve.eval(u) + this->direction * v;
        }
        case Revolution:
        {
            RStepVector p = this->curve.eval(v) - pl.origin;
            const RStepVector &k = this->direction;
            double c = std::cos(u);
            double s = std::sin(u);
            return pl.origin + p * c + k.cross(p) * s + k * (k.dot(p) * (1.0 - c));
        }
        case Offset:
        {
            return this->basis->eval(u,v) + this->basis->normal(u,v) * this->distance;
        }
        default:
        {
            return RStepVector();
        }
    }
}

void RStepSurface::derivatives(double u, double v, RStepVector &du, RStepVector &dv) const
{
    const RStepPlacement &pl = this->placement;
    switch (this->kind)
    {
        case Plane:
        {
            du = pl.x;
            dv = pl.y;
            return;
        }
        case Cylinder:
        {
            du = (pl.y * std::cos(u) - pl.x * std::sin(u)) * this->radius1;
            dv = pl.z;
            return;
        }
        case Cone:
        {
            du = (pl.y * std::cos(u) - pl.x * std::sin(u)) * (this->radius1 + v * this->tanAngle);
            dv = (pl.x * std::cos(u) + pl.y * std::sin(u)) * this->tanAngle + pl.z;
            return;
        }
        case Sphere:
        {
            du = (pl.y * std::cos(u) - pl.x * std::sin(u)) * (this->radius1 * std::cos(v));
            dv = (pl.x * std::cos(u) + pl.y * std::sin(u)) * (-this->radius1 * std::sin(v)) + pl.z * (this->radius1 * std::cos(v));
            return;
        }
        case Torus:
        {
            du = (pl.y * std::cos(u) - pl.x * std::sin(u)) * (this->radius1 + this->radius2 * std::cos(v));
            dv = (pl.x * std::cos(u) + pl.y * std::sin(u)) * (-this->radius2 * std::sin(v)) + pl.z * (this->radius2 * std::cos(v));
            return;
        }
        default:
        {
            double h[2];
            for (int d=0;d<2;d++)
            {
                double range = (this->bounded[d] ? (this->domainMax[d] - this->domainMin[d]) : (this->periodic[d] ? this->period[d] : 1.0));
                h[d] = 1e-6 * std::max(range,1e-9);
            }
            double u1 = u - h[0], u2 = u + h[0];
            double v1 = v - h[1], v2 = v + h[1];
            // One sided differences at domain boundaries.
            if (this->bounded[0] && !this->periodic[0])
            {
                u1 = std::max(u1,this->domainMin[0]);
                u2 = std::min(u2,this->domainMax[0]);
            }
            if (this->bounded[1] && !this->periodic[1])
            {
                v1 = std::max(v1,this->domainMin[1]);
                v2 = std::min(v2,this->domainMax[1]);
            }
            du = (this->eval(u2,v) - this->eval(u1,v)) * (1.0 / (u2 - u1));
            dv = (this->eval(u,v2) - this->eval(u,v1)) * (1.0 / (v2 - v1));
            return;
        }
    }
}

RStepVector RStepSurface::normal(double u, double v) const
{
    RStepVector du, dv;
    this->derivatives(u,v,du,dv);
    RStepVector n = du.cross(dv);
    if (n.length() > 0.0)
    {
        return n.normalized();
    }
    // Singular point - use nearby normal.
    double eps = 1e-6;
    double u2 = u + ((this->bounded[0] && u + eps > this->domainMax[0]) ? -eps : eps);
    double v2 = v + ((this->bounded[1] && v + eps > this->domainMax[1]) ? -eps : eps);
    this->derivatives(u2,v2,du,dv);
    return du.cross(dv).normalized();
}

void RStepSurface::project(const RStepVector &p, double &u, double &v, bool useHint) const
{
    const RStepPlacement &pl = this->placement;
    RStepVector d = p - pl.origin;
    switch (this->kind)
    {
        case Plane:
        {
            u = d.dot(pl.x);
            v = d.dot(pl.y);
            return;
        }
        case Cylinder:
        case Cone:
        case Sphere:
        case Torus:
        {
            double dx = d.dot(pl.x);
            double dy = d.dot(pl.y);
            double dz = d.dot(pl.z);
            u = std::atan2(dy,dx);
            if (u < 0.0)
            {
                u += stepTwoPi;
            }
            if (this->kind == Cylinder)
            {
                v = dz;
            }
            else if (this->kind == Cone)
            {
                // Nearest point on generator line.
                double rho = std::sqrt(dx*dx + dy*dy);
                double c = 1.0 / std::sqrt(1.0 + this->tanAngle * this->tanAngle);
                double s = this->tanAngle * c;
                // Generator: (R + v tan) radial + v axial; unit direction (s, c) scaled by 1/c per unit v.
                v = ((rho - this->radius1) * s + dz * c) * c;
            }
            else if (this->kind == Sphere)
            {
                double rho = std::sqrt(dx*dx + dy*dy);
                v = std::atan2(dz,rho);
            }
            else
            {
                double rho = std::sqrt(dx*dx + dy*dy) - this->radius1;
                v = std::atan2(dz,rho);
                if (v < 0.0)
                {
                    v += stepTwoPi;
                }
            }
            return;
        }
        case Extrusion:
        {
            RStepVector k = this->direction.normalized();
            u = RStepGeometry::minimize([&](double t)
            {
                RStepVector e = this->curve.eval(t) - p;
                e = e - k * e.dot(k);
                return e.dot(e);
            },this->curve.getMin(),this->curve.getMax(),std::max(16,this->curve.getNSamples(this->curve.getMin(),this->curve.getMax(),stepPi/16.0) * 4));
            double l2 = this->direction.dot(this->direction);
            v = (l2 > 0.0) ? (p - this->curve.eval(u)).dot(this->direction) / l2 : 0.0;
            return;
        }
        case Revolution:
        {
            const RStepVector &k = this->direction;
            double a = d.dot(k);
            RStepVector rp = d - k * a;
            double r = rp.length();
            v = RStepGeometry::minimize([&](double t)
            {
                RStepVector e = this->curve.eval(t) - pl.origin;
                double ac = e.dot(k);
                double rc = (e - k * ac).length();
                return (ac - a) * (ac - a) + (rc - r) * (rc - r);
            },this->curve.getMin(),this->curve.getMax(),std::max(16,this->curve.getNSamples(this->curve.getMin(),this->curve.getMax(),stepPi/16.0) * 4));
            RStepVector e = this->curve.eval(v) - pl.origin;
            RStepVector rc = e - k * e.dot(k);
            u = std::atan2(k.dot(rc.cross(rp)),rc.dot(rp));
            if (u < 0.0)
            {
                u += stepTwoPi;
            }
            return;
        }
        case Offset:
        {
            this->basis->project(p,u,v,useHint);
            return;
        }
        case BSpline:
        {
            double tolerance = 1e-5 * this->extent;
            double hintDistance = stepInfinity;
            double hintU = u, hintV = v;
            if (useHint)
            {
                hintDistance = this->refineProjection(p,hintU,hintV);
                if (hintDistance <= tolerance)
                {
                    u = hintU;
                    v = hintV;
                    return;
                }
            }
            // Several nearest grid samples followed by Gauss-Newton iterations.
            std::vector<std::pair<double,size_t>> seeds(this->grid.size());
            for (size_t i=0;i<this->grid.size();i++)
            {
                RStepVector e = this->grid[i] - p;
                seeds[i] = {e.dot(e),i};
            }
            size_t nSeeds = std::min(seeds.size(),size_t(6));
            std::partial_sort(seeds.begin(),seeds.begin() + nSeeds,seeds.end());
            double bestDistance = stepInfinity;
            u = 0.5 * (this->domainMin[0] + this->domainMax[0]);
            v = 0.5 * (this->domainMin[1] + this->domainMax[1]);
            for (size_t k=0;k<nSeeds;k++)
            {
                double su = this->gridUV[seeds[k].second][0];
                double sv = this->gridUV[seeds[k].second][1];
                double distance = this->refineProjection(p,su,sv);
                if (distance < bestDistance)
                {
                    bestDistance = distance;
                    u = su;
                    v = sv;
                }
                if (distance <= tolerance)
                {
                    break;
                }
            }
            // Keep continuity with hint unless global search is clearly better.
            if (useHint && !(bestDistance < 0.5 * hintDistance))
            {
                u = hintU;
                v = hintV;
            }
            return;
        }
        default:
        {
            u = v = 0.0;
            return;
        }
    }
}

double RStepSurface::refineProjection(const RStepVector &p, double &u, double &v) const
{
    for (int iteration=0;iteration<30;iteration++)
    {
        RStepVector su, sv;
        this->derivatives(u,v,su,sv);
        RStepVector r = this->eval(u,v) - p;
        double a11 = su.dot(su), a12 = su.dot(sv), a22 = sv.dot(sv);
        double b1 = -su.dot(r), b2 = -sv.dot(r);
        double det = a11 * a22 - a12 * a12;
        if (std::abs(det) < 1e-30)
        {
            break;
        }
        double du = (b1 * a22 - b2 * a12) / det;
        double dv = (a11 * b2 - a12 * b1) / det;
        double uNew = std::clamp(u + du,this->domainMin[0],this->domainMax[0]);
        double vNew = std::clamp(v + dv,this->domainMin[1],this->domainMax[1]);
        bool converged = (std::abs(uNew - u) <= 1e-12 * (this->domainMax[0] - this->domainMin[0]) &&
                          std::abs(vNew - v) <= 1e-12 * (this->domainMax[1] - this->domainMin[1]));
        u = uNew;
        v = vNew;
        if (converged)
        {
            break;
        }
    }
    return (this->eval(u,v) - p).length();
}

bool RStepSurface::isFlat() const
{
    if (this->kind == Plane)
    {
        return true;
    }
    if (this->kind == Extrusion && this->curve.kind == RStepCurve::Line)
    {
        return true;
    }
    return false;
}

double RStepSurface::getCurvatureRadius() const
{
    switch (this->kind)
    {
        case Cylinder:
        case Sphere:
            return this->radius1;
        case Torus:
            return (this->radius1 > this->radius2) ? std::min(this->radius2,this->radius1 - this->radius2) : this->radius2;
        default:
            return -1.0;
    }
}

void RStepSurface::prepare()
{
    if (this->kind != BSpline)
    {
        return;
    }
    // Dense enough to seed projection on surfaces wrapping many times (threads, helices).
    int nSamplesU = std::clamp(this->bsplineU.getNSpans() * (this->bsplineU.degree + 1) * 2 + 1,9,201);
    int nSamplesV = std::clamp(this->bsplineV.getNSpans() * (this->bsplineV.degree + 1) * 2 + 1,9,201);
    const double maxSamples = 40000.0;
    if (double(nSamplesU) * double(nSamplesV) > maxSamples)
    {
        double factor = std::sqrt(maxSamples / (double(nSamplesU) * double(nSamplesV)));
        nSamplesU = std::max(9,int(nSamplesU * factor));
        nSamplesV = std::max(9,int(nSamplesV * factor));
    }
    RStepVector lo(stepInfinity,stepInfinity,stepInfinity);
    RStepVector hi(-stepInfinity,-stepInfinity,-stepInfinity);
    for (const RStepVector &point : this->points)
    {
        lo = RStepVector(std::min(lo.x,point.x),std::min(lo.y,point.y),std::min(lo.z,point.z));
        hi = RStepVector(std::max(hi.x,point.x),std::max(hi.y,point.y),std::max(hi.z,point.z));
    }
    this->extent = (hi - lo).length();
    this->grid.clear();
    this->gridUV.clear();
    this->grid.reserve(nSamplesU * nSamplesV);
    this->gridUV.reserve(nSamplesU * nSamplesV);
    for (int i=0;i<nSamplesU;i++)
    {
        double u = this->domainMin[0] + (this->domainMax[0] - this->domainMin[0]) * double(i) / double(nSamplesU - 1);
        for (int j=0;j<nSamplesV;j++)
        {
            double v = this->domainMin[1] + (this->domainMax[1] - this->domainMin[1]) * double(j) / double(nSamplesV - 1);
            this->grid.push_back(this->eval(u,v));
            this->gridUV.push_back({u,v});
        }
    }
}

bool RStepSurface::build(const RStepFile &file, unsigned int id, double angleScale, RStepSurface &surface)
{
    const RStepEntity *pEntity = file.find(id);
    if (!pEntity)
    {
        return false;
    }
    const RStepEntity &entity = *pEntity;
    const QString &keyword = entity.keyword();

    surface = RStepSurface();

    if (keyword == "PLANE" || keyword == "CYLINDRICAL_SURFACE" || keyword == "CONICAL_SURFACE" ||
        keyword == "SPHERICAL_SURFACE" || keyword == "TOROIDAL_SURFACE" || keyword == "DEGENERATE_TOROIDAL_SURFACE")
    {
        const RStepRecord &record = entity.records[0];
        if (!RStepGeometry::readPlacement(file,record.param(1).toId(),surface.placement))
        {
            return false;
        }
        if (keyword == "PLANE")
        {
            surface.kind = Plane;
            return true;
        }
        surface.radius1 = record.param(2).toDouble();
        surface.periodic[0] = true;
        surface.period[0] = stepTwoPi;
        surface.domainMin[0] = 0.0;
        surface.domainMax[0] = stepTwoPi;
        surface.bounded[0] = true;
        if (keyword == "CYLINDRICAL_SURFACE")
        {
            surface.kind = Cylinder;
            return (surface.radius1 > 0.0);
        }
        if (keyword == "CONICAL_SURFACE")
        {
            surface.kind = Cone;
            surface.tanAngle = std::tan(record.param(3).toDouble() * angleScale);
            if (std::abs(surface.tanAngle) < 1e-12)
            {
                surface.kind = Cylinder;
                return (surface.radius1 > 0.0);
            }
            double apex = -surface.radius1 / surface.tanAngle;
            if (surface.tanAngle > 0.0)
            {
                surface.domainMin[1] = apex;
            }
            else
            {
                surface.domainMax[1] = apex;
            }
            return true;
        }
        if (keyword == "SPHERICAL_SURFACE")
        {
            surface.kind = Sphere;
            surface.domainMin[1] = -0.5 * stepPi;
            surface.domainMax[1] = 0.5 * stepPi;
            surface.bounded[1] = true;
            return (surface.radius1 > 0.0);
        }
        surface.kind = Torus;
        surface.radius2 = record.param(3).toDouble();
        surface.periodic[1] = true;
        surface.period[1] = stepTwoPi;
        surface.domainMin[1] = 0.0;
        surface.domainMax[1] = stepTwoPi;
        surface.bounded[1] = true;
        return (surface.radius2 > 0.0);
    }
    if (keyword == "RECTANGULAR_TRIMMED_SURFACE" || keyword == "CURVE_BOUNDED_SURFACE")
    {
        return RStepSurface::build(file,entity.records[0].param(1).toId(),angleScale,surface);
    }
    if (keyword == "SURFACE_OF_LINEAR_EXTRUSION")
    {
        const RStepRecord &record = entity.records[0];
        const RStepEntity *pVector = file.find(record.param(2).toId());
        if (!pVector || pVector->keyword() != "VECTOR" || !RStepCurve::build(file,record.param(1).toId(),surface.curve))
        {
            return false;
        }
        RStepVector direction;
        if (!RStepGeometry::readDirection(file,pVector->records[0].param(1).toId(),direction))
        {
            return false;
        }
        double magnitude = pVector->records[0].param(2).toDouble(1.0);
        surface.direction = direction * ((magnitude > 0.0) ? magnitude : 1.0);
        surface.kind = Extrusion;
        surface.periodic[0] = surface.curve.periodic;
        surface.period[0] = surface.curve.period;
        surface.domainMin[0] = surface.curve.getMin();
        surface.domainMax[0] = surface.curve.getMax();
        surface.bounded[0] = std::isfinite(surface.domainMin[0]) && std::isfinite(surface.domainMax[0]);
        return true;
    }
    if (keyword == "SURFACE_OF_REVOLUTION")
    {
        const RStepRecord &record = entity.records[0];
        if (!RStepCurve::build(file,record.param(1).toId(),surface.curve) ||
            !RStepGeometry::readPlacement(file,record.param(2).toId(),surface.placement))
        {
            return false;
        }
        surface.direction = surface.placement.z;
        surface.kind = Revolution;
        surface.periodic[0] = true;
        surface.period[0] = stepTwoPi;
        surface.domainMin[0] = 0.0;
        surface.domainMax[0] = stepTwoPi;
        surface.bounded[0] = true;
        surface.periodic[1] = surface.curve.periodic;
        surface.period[1] = surface.curve.period;
        surface.domainMin[1] = surface.curve.getMin();
        surface.domainMax[1] = surface.curve.getMax();
        surface.bounded[1] = std::isfinite(surface.domainMin[1]) && std::isfinite(surface.domainMax[1]);
        return true;
    }
    if (keyword == "OFFSET_SURFACE")
    {
        const RStepRecord &record = entity.records[0];
        auto basis = std::make_shared<RStepSurface>();
        if (!RStepSurface::build(file,record.param(1).toId(),angleScale,*basis))
        {
            return false;
        }
        surface.kind = Offset;
        surface.distance = record.param(2).toDouble();
        for (int d=0;d<2;d++)
        {
            surface.periodic[d] = basis->periodic[d];
            surface.period[d] = basis->period[d];
            surface.bounded[d] = basis->bounded[d];
            surface.domainMin[d] = basis->domainMin[d];
            surface.domainMax[d] = basis->domainMax[d];
        }
        surface.basis = basis;
        return true;
    }

    // B-spline surfaces - simple or complex instances.
    const RStepRecord *pWithKnots = entity.find("B_SPLINE_SURFACE_WITH_KNOTS");
    QString implicitType = findImplicitKnotType(entity);
    if (pWithKnots || !implicitType.isEmpty())
    {
        int degreeU = 0, degreeV = 0;
        RStepValue controlPoints;
        bool closed[2] = {false,false};
        RStepValue multiplicitiesU, multiplicitiesV, knotsU, knotsV;
        RStepValue weights;

        if (!entity.isComplex())
        {
            const RStepRecord &record = entity.records[0];
            degreeU = record.param(1).toInt();
            degreeV = record.param(2).toInt();
            controlPoints = record.param(3);
            closed[0] = record.param(5).toBool(false);
            closed[1] = record.param(6).toBool(false);
            if (pWithKnots)
            {
                multiplicitiesU = record.param(8);
                multiplicitiesV = record.param(9);
                knotsU = record.param(10);
                knotsV = record.param(11);
            }
        }
        else
        {
            const RStepRecord *pBase = entity.find("B_SPLINE_SURFACE");
            if (!pBase)
            {
                return false;
            }
            degreeU = pBase->param(0).toInt();
            degreeV = pBase->param(1).toInt();
            controlPoints = pBase->param(2);
            closed[0] = pBase->param(4).toBool(false);
            closed[1] = pBase->param(5).toBool(false);
            if (pWithKnots)
            {
                multiplicitiesU = pWithKnots->param(0);
                multiplicitiesV = pWithKnots->param(1);
                knotsU = pWithKnots->param(2);
                knotsV = pWithKnots->param(3);
            }
            if (const RStepRecord *pRational = entity.find("RATIONAL_B_SPLINE_SURFACE"))
            {
                weights = pRational->param(0);
            }
        }

        surface.nu = int(controlPoints.size());
        surface.nv = (surface.nu > 0) ? int(controlPoints.at(0).size()) : 0;
        if (surface.nu <= degreeU || surface.nv <= degreeV || degreeU < 1 || degreeV < 1)
        {
            return false;
        }
        surface.points.reserve(surface.nu * surface.nv);
        for (int i=0;i<surface.nu;i++)
        {
            const RStepValue &row = controlPoints.at(i);
            if (int(row.size()) != surface.nv)
            {
                return false;
            }
            for (int j=0;j<surface.nv;j++)
            {
                RStepVector point;
                if (!RStepGeometry::readPoint(file,row.at(j).toId(),point))
                {
                    return false;
                }
                surface.points.push_back(point);
            }
        }
        if (weights.isList())
        {
            for (int i=0;i<surface.nu;i++)
            {
                for (int j=0;j<surface.nv;j++)
                {
                    surface.weights.push_back(weights.at(i).at(j).toDouble(1.0));
                }
            }
        }
        if (pWithKnots)
        {
            if (!buildKnots(multiplicitiesU,knotsU,degreeU,surface.nu,surface.bsplineU) ||
                !buildKnots(multiplicitiesV,knotsV,degreeV,surface.nv,surface.bsplineV))
            {
                return false;
            }
        }
        else
        {
            buildImplicitKnots(implicitType,degreeU,surface.nu,surface.bsplineU);
            buildImplicitKnots(implicitType,degreeV,surface.nv,surface.bsplineV);
        }
        surface.kind = BSpline;
        surface.domainMin[0] = surface.bsplineU.getMin();
        surface.domainMax[0] = surface.bsplineU.getMax();
        surface.domainMin[1] = surface.bsplineV.getMin();
        surface.domainMax[1] = surface.bsplineV.getMax();
        surface.bounded[0] = surface.bounded[1] = true;

        // Closed directions are treated as periodic when the opposite boundaries coincide.
        for (int d=0;d<2;d++)
        {
            if (!closed[d])
            {
                continue;
            }
            bool coincide = true;
            double extent = 0.0;
            for (int k=0;k<=4;k++)
            {
                double t = surface.domainMin[1-d] + (surface.domainMax[1-d] - surface.domainMin[1-d]) * double(k) / 4.0;
                RStepVector p1 = (d == 0) ? surface.eval(surface.domainMin[0],t) : surface.eval(t,surface.domainMin[1]);
                RStepVector p2 = (d == 0) ? surface.eval(surface.domainMax[0],t) : surface.eval(t,surface.domainMax[1]);
                RStepVector pm = (d == 0) ? surface.eval(0.5 * (surface.domainMin[0] + surface.domainMax[0]),t) : surface.eval(t,0.5 * (surface.domainMin[1] + surface.domainMax[1]));
                extent = std::max(extent,(pm - p1).length());
                if ((p2 - p1).length() > 1e-6 * std::max(extent,1e-12))
                {
                    coincide = false;
                }
            }
            if (coincide)
            {
                surface.periodic[d] = true;
                surface.period[d] = surface.domainMax[d] - surface.domainMin[d];
            }
        }
        surface.prepare();
        return true;
    }

    return false;
}

bool RStepGeometry::readPoint(const RStepFile &file, unsigned int id, RStepVector &point)
{
    const RStepEntity *pEntity = file.find(id);
    if (!pEntity || pEntity->keyword() != "CARTESIAN_POINT")
    {
        return false;
    }
    const RStepValue &coordinates = pEntity->records[0].param(1);
    if (coordinates.size() < 2)
    {
        return false;
    }
    point = RStepVector(coordinates.at(0).toDouble(),coordinates.at(1).toDouble(),coordinates.at(2).toDouble());
    return true;
}

bool RStepGeometry::readDirection(const RStepFile &file, unsigned int id, RStepVector &direction)
{
    const RStepEntity *pEntity = file.find(id);
    if (!pEntity || pEntity->keyword() != "DIRECTION")
    {
        return false;
    }
    const RStepValue &ratios = pEntity->records[0].param(1);
    if (ratios.size() < 2)
    {
        return false;
    }
    direction = RStepVector(ratios.at(0).toDouble(),ratios.at(1).toDouble(),ratios.at(2).toDouble()).normalized();
    return (direction.length() > 0.0);
}

bool RStepGeometry::readVertex(const RStepFile &file, unsigned int id, RStepVector &point)
{
    const RStepEntity *pEntity = file.find(id);
    if (!pEntity || pEntity->keyword() != "VERTEX_POINT")
    {
        return false;
    }
    return RStepGeometry::readPoint(file,pEntity->records[0].param(1).toId(),point);
}

bool RStepGeometry::readPlacement(const RStepFile &file, unsigned int id, RStepPlacement &placement)
{
    const RStepEntity *pEntity = file.find(id);
    if (!pEntity)
    {
        return false;
    }
    const QString &keyword = pEntity->keyword();
    const RStepRecord &record = pEntity->records[0];

    placement = RStepPlacement();

    if (keyword == "AXIS2_PLACEMENT_3D" || keyword == "AXIS1_PLACEMENT" || keyword == "AXIS2_PLACEMENT_2D")
    {
        if (!RStepGeometry::readPoint(file,record.param(1).toId(),placement.origin))
        {
            return false;
        }
        RStepVector z(0.0,0.0,1.0);
        RStepVector x(1.0,0.0,0.0);
        if (keyword == "AXIS2_PLACEMENT_2D")
        {
            RStepGeometry::readDirection(file,record.param(2).toId(),x);
        }
        else
        {
            RStepGeometry::readDirection(file,record.param(2).toId(),z);
            if (keyword == "AXIS2_PLACEMENT_3D")
            {
                RStepGeometry::readDirection(file,record.param(3).toId(),x);
            }
        }
        x = x - z * x.dot(z);
        if (x.length() < 1e-9)
        {
            // Reference direction parallel with axis - pick any perpendicular.
            x = (std::abs(z.x) < 0.9) ? RStepVector(1.0,0.0,0.0) : RStepVector(0.0,1.0,0.0);
            x = x - z * x.dot(z);
        }
        placement.z = z;
        placement.x = x.normalized();
        placement.y = z.cross(placement.x);
        return true;
    }
    return false;
}

bool RStepGeometry::readTransform(const RStepFile &file, unsigned int id, RStepTransform &transform)
{
    const RStepEntity *pEntity = file.find(id);
    if (!pEntity)
    {
        return false;
    }
    const QString &keyword = pEntity->keyword();

    if (keyword == "AXIS2_PLACEMENT_3D")
    {
        RStepPlacement placement;
        if (!RStepGeometry::readPlacement(file,id,placement))
        {
            return false;
        }
        transform = placement.toTransform();
        return true;
    }
    if (keyword == "CARTESIAN_TRANSFORMATION_OPERATOR_3D" || keyword == "CARTESIAN_TRANSFORMATION_OPERATOR_3D_NON_UNIFORM")
    {
        const RStepRecord &record = pEntity->records[0];
        RStepPlacement placement;
        RStepVector x(1.0,0.0,0.0);
        RStepVector z(0.0,0.0,1.0);
        RStepGeometry::readDirection(file,record.param(2).toId(),x);
        RStepGeometry::readDirection(file,record.param(6).toId(),z);
        if (!RStepGeometry::readPoint(file,record.param(4).toId(),placement.origin))
        {
            return false;
        }
        x = x - z * x.dot(z);
        if (x.length() < 1e-9)
        {
            x = (std::abs(z.x) < 0.9) ? RStepVector(1.0,0.0,0.0) : RStepVector(0.0,1.0,0.0);
            x = x - z * x.dot(z);
        }
        placement.z = z;
        placement.x = x.normalized();
        placement.y = z.cross(placement.x);
        transform = placement.toTransform();
        transform.s = record.param(5).toDouble(1.0);
        return true;
    }
    return false;
}

double RStepGeometry::minimize(const std::function<double (double)> &function, double a, double b, int nSamples)
{
    if (!std::isfinite(a) || !std::isfinite(b) || b <= a)
    {
        return std::isfinite(a) ? a : 0.0;
    }
    nSamples = std::max(nSamples,2);
    std::vector<double> values(nSamples + 1);
    for (int i=0;i<=nSamples;i++)
    {
        values[i] = function(a + (b - a) * double(i) / double(nSamples));
    }

    // Every local minimum of samples is refined - curves may pass close to the point several times.
    double bestT = a;
    double bestValue = stepInfinity;
    const double ratio = 0.5 * (std::sqrt(5.0) - 1.0);
    for (int k=0;k<=nSamples;k++)
    {
        if ((k > 0 && values[k] > values[k-1]) || (k < nSamples && values[k] > values[k+1]))
        {
            continue;
        }
        double lo = a + (b - a) * double(std::max(k - 1,0)) / double(nSamples);
        double hi = a + (b - a) * double(std::min(k + 1,nSamples)) / double(nSamples);
        double x1 = hi - ratio * (hi - lo);
        double x2 = lo + ratio * (hi - lo);
        double f1 = function(x1);
        double f2 = function(x2);
        for (int i=0;i<80 && (hi - lo) > 1e-15 * std::max(1.0,std::abs(hi));i++)
        {
            if (f1 < f2)
            {
                hi = x2;
                x2 = x1;
                f2 = f1;
                x1 = hi - ratio * (hi - lo);
                f1 = function(x1);
            }
            else
            {
                lo = x1;
                x1 = x2;
                f1 = f2;
                x2 = lo + ratio * (hi - lo);
                f2 = function(x2);
            }
        }
        double t = 0.5 * (lo + hi);
        double value = function(t);
        if (values[k] < value)
        {
            t = a + (b - a) * double(k) / double(nSamples);
            value = values[k];
        }
        if (value < bestValue)
        {
            bestValue = value;
            bestT = t;
        }
    }
    return bestT;
}
