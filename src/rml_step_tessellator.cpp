#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <tuple>

#include "rml_step_tessellator.h"
#include "rml_constrained_delaunay.h"

static const double stepPi = 3.14159265358979323846;
static const double stepInfinity = std::numeric_limits<double>::infinity();

void RStepMesh::append(const RStepMesh &mesh, const RStepTransform &transform, bool flip)
{
    unsigned int offset = unsigned(this->nodes.size());
    bool identity = transform.isIdentity();
    this->nodes.reserve(this->nodes.size() + mesh.nodes.size());
    for (const RStepVector &node : mesh.nodes)
    {
        this->nodes.push_back(identity ? node : transform.apply(node));
    }
    // Mirroring transformation reverses orientation.
    if (transform.isMirror())
    {
        flip = !flip;
    }
    this->triangles.reserve(this->triangles.size() + mesh.triangles.size());
    for (const auto &triangle : mesh.triangles)
    {
        if (flip)
        {
            this->triangles.push_back({triangle[0] + offset,triangle[2] + offset,triangle[1] + offset});
        }
        else
        {
            this->triangles.push_back({triangle[0] + offset,triangle[1] + offset,triangle[2] + offset});
        }
    }
}

RStepTessellator::RStepTessellator(const RStepFile &file, double angleScale)
    : file(file)
    , angleScale(angleScale)
    , maxAngle(stepPi / 12.0)
    , chordTolerance(1e-3)
    , nUnsupportedEdges(0)
{

}

void RStepTessellator::setTolerances(double maxAngle, double chordTolerance)
{
    this->maxAngle = maxAngle;
    this->chordTolerance = chordTolerance;
}

RStepTessellator::ItemType RStepTessellator::getItemType(const RStepEntity &entity)
{
    static const char *brepKeywords[] = {
        "MANIFOLD_SOLID_BREP", "BREP_WITH_VOIDS", "FACETED_BREP", "SHELL_BASED_SURFACE_MODEL",
        "FACE_BASED_SURFACE_MODEL", "CLOSED_SHELL", "OPEN_SHELL", "ORIENTED_CLOSED_SHELL",
        "ORIENTED_OPEN_SHELL", "CONNECTED_FACE_SET", "ADVANCED_FACE", "FACE_SURFACE", "ORIENTED_FACE"
    };
    static const char *tessellatedKeywords[] = {
        "TESSELLATED_SOLID", "TESSELLATED_SHELL", "TRIANGULATED_FACE", "COMPLEX_TRIANGULATED_FACE",
        "TRIANGULATED_SURFACE_SET", "COMPLEX_TRIANGULATED_SURFACE_SET", "TESSELLATED_GEOMETRIC_SET"
    };
    for (const RStepRecord &record : entity.records)
    {
        for (const char *keyword : brepKeywords)
        {
            if (record.keyword == keyword)
            {
                return ItemBRep;
            }
        }
        for (const char *keyword : tessellatedKeywords)
        {
            if (record.keyword == keyword)
            {
                return ItemTessellated;
            }
        }
    }
    return ItemNone;
}

QString RStepTessellator::getEntityName(const RStepEntity &entity)
{
    if (!entity.isComplex())
    {
        return (entity.records.empty()) ? QString() : entity.records[0].param(0).text;
    }
    const RStepRecord *pRecord = entity.find("REPRESENTATION_ITEM");
    return pRecord ? pRecord->param(0).text : QString();
}

double RStepTessellator::estimateSize(unsigned int itemId) const
{
    RStepVector lo(stepInfinity,stepInfinity,stepInfinity);
    RStepVector hi(-stepInfinity,-stepInfinity,-stepInfinity);
    auto addPoint = [&](const RStepVector &p)
    {
        lo = RStepVector(std::min(lo.x,p.x),std::min(lo.y,p.y),std::min(lo.z,p.z));
        hi = RStepVector(std::max(hi.x,p.x),std::max(hi.y,p.y),std::max(hi.z,p.z));
    };

    std::set<unsigned int> visited;
    std::vector<unsigned int> stack = {itemId};
    while (!stack.empty())
    {
        unsigned int id = stack.back();
        stack.pop_back();
        if (!visited.insert(id).second)
        {
            continue;
        }
        const RStepEntity *pEntity = this->file.find(id);
        if (!pEntity)
        {
            continue;
        }
        const QString &keyword = pEntity->keyword();
        if (keyword == "CARTESIAN_POINT")
        {
            RStepVector p;
            if (RStepGeometry::readPoint(this->file,id,p))
            {
                addPoint(p);
            }
            continue;
        }
        if (keyword == "COORDINATES_LIST")
        {
            const RStepValue &list = pEntity->records[0].param(2);
            for (size_t i=0;i<list.size();i++)
            {
                const RStepValue &xyz = list.at(i);
                addPoint(RStepVector(xyz.at(0).toDouble(),xyz.at(1).toDouble(),xyz.at(2).toDouble()));
            }
            continue;
        }
        // Do not descend into face and edge geometry - control points may lie far from the shape.
        std::function<void(const RStepValue &)> collect = [&](const RStepValue &value)
        {
            if (value.isReference())
            {
                stack.push_back(value.id);
            }
            else if (value.isList())
            {
                for (const RStepValue &item : value.items)
                {
                    collect(item);
                }
            }
        };
        for (const RStepRecord &record : pEntity->records)
        {
            for (size_t i=0;i<record.params.size();i++)
            {
                if ((record.keyword == "ADVANCED_FACE" || record.keyword == "FACE_SURFACE") && i == 2)
                {
                    continue;
                }
                if (record.keyword == "EDGE_CURVE" && i == 3)
                {
                    continue;
                }
                if (record.keyword == "TRIANGULATED_FACE" || record.keyword == "COMPLEX_TRIANGULATED_FACE")
                {
                    // Skip geometric link.
                    if (i == 4)
                    {
                        continue;
                    }
                }
                collect(record.params[i]);
            }
        }
    }

    if (!(hi.x >= lo.x))
    {
        return 0.0;
    }
    return (hi - lo).length();
}

bool RStepTessellator::tessellateItem(unsigned int itemId, RStepMesh &mesh)
{
    const RStepEntity *pEntity = this->file.find(itemId);
    if (!pEntity)
    {
        return false;
    }
    const RStepEntity &entity = *pEntity;

    if (const RStepRecord *pRecord = entity.find("MANIFOLD_SOLID_BREP"))
    {
        // Complex instances do not carry name in partial entity record.
        unsigned int outer = pRecord->param(entity.isComplex() ? 0 : 1).toId();
        this->tessellateShell(outer,true,mesh);
        if (const RStepRecord *pVoids = entity.find("BREP_WITH_VOIDS"))
        {
            for (unsigned int voidId : pVoids->param(entity.isComplex() ? 0 : 2).toIdList())
            {
                this->tessellateShell(voidId,true,mesh);
            }
        }
        return true;
    }

    const QString &keyword = entity.keyword();
    const RStepRecord &record = entity.records[0];

    if (keyword == "BREP_WITH_VOIDS")
    {
        this->tessellateShell(record.param(1).toId(),true,mesh);
        for (unsigned int voidId : record.param(2).toIdList())
        {
            this->tessellateShell(voidId,true,mesh);
        }
        return true;
    }
    if (keyword == "FACETED_BREP")
    {
        this->tessellateShell(record.param(1).toId(),true,mesh);
        return true;
    }
    if (keyword == "SHELL_BASED_SURFACE_MODEL")
    {
        for (unsigned int shellId : record.param(1).toIdList())
        {
            this->tessellateShell(shellId,true,mesh);
        }
        return true;
    }
    if (keyword == "FACE_BASED_SURFACE_MODEL")
    {
        for (unsigned int faceSetId : record.param(1).toIdList())
        {
            this->tessellateShell(faceSetId,true,mesh);
        }
        return true;
    }
    if (keyword == "CLOSED_SHELL" || keyword == "OPEN_SHELL" || keyword == "CONNECTED_FACE_SET" ||
        keyword == "ORIENTED_CLOSED_SHELL" || keyword == "ORIENTED_OPEN_SHELL")
    {
        this->tessellateShell(itemId,true,mesh);
        return true;
    }
    if (keyword == "ADVANCED_FACE" || keyword == "FACE_SURFACE" || keyword == "ORIENTED_FACE")
    {
        if (!this->tessellateFace(itemId,true,mesh))
        {
            this->failedFaces.push_back(itemId);
        }
        return true;
    }
    if (keyword == "TESSELLATED_SOLID" || keyword == "TESSELLATED_SHELL" || keyword == "TESSELLATED_GEOMETRIC_SET")
    {
        for (unsigned int childId : record.param(1).toIdList())
        {
            const RStepEntity *pChild = this->file.find(childId);
            if (pChild)
            {
                if (pChild->keyword() == "TESSELLATED_SHELL" || pChild->keyword() == "TESSELLATED_SOLID" || pChild->keyword() == "TESSELLATED_GEOMETRIC_SET")
                {
                    this->tessellateItem(childId,mesh);
                }
                else
                {
                    this->tessellateTriangulated(*pChild,mesh);
                }
            }
        }
        return true;
    }
    if (keyword == "TRIANGULATED_FACE" || keyword == "COMPLEX_TRIANGULATED_FACE" ||
        keyword == "TRIANGULATED_SURFACE_SET" || keyword == "COMPLEX_TRIANGULATED_SURFACE_SET")
    {
        return this->tessellateTriangulated(entity,mesh);
    }
    return false;
}

unsigned int RStepTessellator::getNFailedFaces() const
{
    return unsigned(this->failedFaces.size());
}

const std::vector<unsigned int> &RStepTessellator::getFailedFaces() const
{
    return this->failedFaces;
}

unsigned int RStepTessellator::getNUnsupportedEdges() const
{
    return this->nUnsupportedEdges;
}

void RStepTessellator::tessellateShell(unsigned int shellId, bool orientation, RStepMesh &mesh)
{
    const RStepEntity *pEntity = this->file.find(shellId);
    if (!pEntity)
    {
        return;
    }
    const QString &keyword = pEntity->keyword();
    const RStepRecord &record = pEntity->records[0];

    if (keyword == "ORIENTED_CLOSED_SHELL" || keyword == "ORIENTED_OPEN_SHELL")
    {
        bool shellOrientation = record.param(3).toBool(true);
        this->tessellateShell(record.param(2).toId(),orientation == shellOrientation,mesh);
        return;
    }
    if (keyword == "CLOSED_SHELL" || keyword == "OPEN_SHELL" || keyword == "CONNECTED_FACE_SET")
    {
        for (unsigned int faceId : record.param(1).toIdList())
        {
            if (!this->tessellateFace(faceId,orientation,mesh))
            {
                this->failedFaces.push_back(faceId);
            }
        }
    }
}

bool RStepTessellator::tessellateFace(unsigned int faceId, bool orientation, RStepMesh &mesh)
{
    const RStepEntity *pEntity = this->file.find(faceId);
    if (!pEntity)
    {
        return false;
    }
    const QString &keyword = pEntity->keyword();
    const RStepRecord &record = pEntity->records[0];

    if (keyword == "ORIENTED_FACE")
    {
        bool faceOrientation = record.param(3).toBool(true);
        return this->tessellateFace(record.param(2).toId(),orientation == faceOrientation,mesh);
    }
    if (keyword != "ADVANCED_FACE" && keyword != "FACE_SURFACE" && keyword != "FACE")
    {
        return false;
    }

    std::vector<std::vector<RStepVector>> loops;
    std::vector<bool> outerFlags;
    for (unsigned int boundId : record.param(1).toIdList())
    {
        const RStepEntity *pBound = this->file.find(boundId);
        if (!pBound || (pBound->keyword() != "FACE_BOUND" && pBound->keyword() != "FACE_OUTER_BOUND"))
        {
            continue;
        }
        std::vector<RStepVector> points;
        if (!this->readLoop(pBound->records[0].param(1).toId(),points))
        {
            return false;
        }
        if (points.size() < 2)
        {
            // Vertex loop.
            continue;
        }
        if (!pBound->records[0].param(2).toBool(true))
        {
            std::reverse(points.begin(),points.end());
        }
        loops.push_back(points);
        outerFlags.push_back(pBound->keyword() == "FACE_OUTER_BOUND");
    }

    bool sameSense = true;
    std::shared_ptr<RStepSurface> surface;
    if (keyword != "FACE")
    {
        surface = this->getSurface(record.param(2).toId());
        sameSense = record.param(3).toBool(true);
    }

    if (!surface)
    {
        // Face without geometry or with unsupported geometry - planar approximation if loops are planar.
        if (loops.empty())
        {
            return false;
        }
        size_t outerLoop = 0;
        for (size_t i=0;i<loops.size();i++)
        {
            if (outerFlags[i])
            {
                outerLoop = i;
                break;
            }
        }
        RStepVector normal;
        RStepVector center;
        size_t nPoints = 0;
        const std::vector<RStepVector> &outer = loops[outerLoop];
        for (size_t i=0;i<outer.size();i++)
        {
            const RStepVector &a = outer[i];
            const RStepVector &b = outer[(i+1)%outer.size()];
            normal += RStepVector((a.y - b.y) * (a.z + b.z),(a.z - b.z) * (a.x + b.x),(a.x - b.x) * (a.y + b.y));
        }
        for (const std::vector<RStepVector> &loop : loops)
        {
            for (const RStepVector &p : loop)
            {
                center += p;
                nPoints++;
            }
        }
        if (normal.length() <= 0.0 || nPoints == 0)
        {
            return false;
        }
        center = center * (1.0 / double(nPoints));
        normal = normal.normalized();
        double deviation = 0.0;
        double extent = 0.0;
        for (const std::vector<RStepVector> &loop : loops)
        {
            for (const RStepVector &p : loop)
            {
                deviation = std::max(deviation,std::abs((p - center).dot(normal)));
                extent = std::max(extent,(p - center).length());
            }
        }
        if (deviation > std::max(this->chordTolerance,1e-6 * extent))
        {
            return false;
        }
        auto plane = std::make_shared<RStepSurface>();
        plane->kind = RStepSurface::Plane;
        plane->placement.origin = center;
        plane->placement.z = normal;
        RStepVector x = (std::abs(normal.x) < 0.9) ? RStepVector(1.0,0.0,0.0) : RStepVector(0.0,1.0,0.0);
        plane->placement.x = (x - normal * x.dot(normal)).normalized();
        plane->placement.y = normal.cross(plane->placement.x);
        surface = plane;
        // Plane normal follows outer loop orientation.
        sameSense = true;
    }

    RStepMesh faceMesh;
    if (!this->triangulateFace(*surface,loops,outerFlags,sameSense,faceMesh))
    {
        return false;
    }
    mesh.append(faceMesh,RStepTransform(),!orientation);
    return true;
}

bool RStepTessellator::tessellateTriangulated(const RStepEntity &entity, RStepMesh &mesh) const
{
    const QString &keyword = entity.keyword();
    const RStepRecord &record = entity.records[0];

    bool isFace = (keyword == "TRIANGULATED_FACE" || keyword == "COMPLEX_TRIANGULATED_FACE");
    bool isComplex = (keyword == "COMPLEX_TRIANGULATED_FACE" || keyword == "COMPLEX_TRIANGULATED_SURFACE_SET");
    if (!isFace && keyword != "TRIANGULATED_SURFACE_SET" && keyword != "COMPLEX_TRIANGULATED_SURFACE_SET")
    {
        return false;
    }

    const RStepEntity *pCoordinates = this->file.find(record.param(1).toId());
    if (!pCoordinates || pCoordinates->keyword() != "COORDINATES_LIST")
    {
        return false;
    }
    const RStepValue &coordinates = pCoordinates->records[0].param(2);
    std::vector<RStepVector> points;
    points.reserve(coordinates.size());
    for (size_t i=0;i<coordinates.size();i++)
    {
        const RStepValue &xyz = coordinates.at(i);
        points.push_back(RStepVector(xyz.at(0).toDouble(),xyz.at(1).toDouble(),xyz.at(2).toDouble()));
    }

    // Parameter positions differ for faces (extra geometric link).
    size_t pnIndexPosition = isFace ? 5 : 4;
    const RStepValue &pnIndex = record.param(pnIndexPosition);

    unsigned int offset = unsigned(mesh.nodes.size());
    mesh.nodes.insert(mesh.nodes.end(),points.begin(),points.end());

    auto nodeId = [&](int index, unsigned int &id) -> bool
    {
        // Indexes are 1-based.
        if (pnIndex.size() > 0)
        {
            if (index < 1 || size_t(index) > pnIndex.size())
            {
                return false;
            }
            index = pnIndex.at(index-1).toInt();
        }
        if (index < 1 || size_t(index) > points.size())
        {
            return false;
        }
        id = offset + unsigned(index - 1);
        return true;
    };

    auto addTriangle = [&](int a, int b, int c)
    {
        unsigned int na, nb, nc;
        if (a == b || b == c || a == c || !nodeId(a,na) || !nodeId(b,nb) || !nodeId(c,nc))
        {
            return;
        }
        mesh.triangles.push_back({na,nb,nc});
    };

    if (!isComplex)
    {
        const RStepValue &triangles = record.param(pnIndexPosition + 1);
        for (size_t i=0;i<triangles.size();i++)
        {
            const RStepValue &t = triangles.at(i);
            addTriangle(t.at(0).toInt(),t.at(1).toInt(),t.at(2).toInt());
        }
    }
    else
    {
        const RStepValue &strips = record.param(pnIndexPosition + 1);
        const RStepValue &fans = record.param(pnIndexPosition + 2);
        for (size_t i=0;i<strips.size();i++)
        {
            const RStepValue &strip = strips.at(i);
            for (size_t j=0;j+2<strip.size();j++)
            {
                if (j % 2 == 0)
                {
                    addTriangle(strip.at(j).toInt(),strip.at(j+1).toInt(),strip.at(j+2).toInt());
                }
                else
                {
                    addTriangle(strip.at(j+1).toInt(),strip.at(j).toInt(),strip.at(j+2).toInt());
                }
            }
        }
        for (size_t i=0;i<fans.size();i++)
        {
            const RStepValue &fan = fans.at(i);
            for (size_t j=1;j+1<fan.size();j++)
            {
                addTriangle(fan.at(0).toInt(),fan.at(j).toInt(),fan.at(j+1).toInt());
            }
        }
    }
    return true;
}

const std::vector<RStepVector> &RStepTessellator::sampleEdge(unsigned int edgeId)
{
    auto iter = this->edgeCache.find(edgeId);
    if (iter != this->edgeCache.end())
    {
        return iter->second;
    }
    std::vector<RStepVector> &points = this->edgeCache[edgeId];

    const RStepEntity *pEntity = this->file.find(edgeId);
    if (!pEntity || (pEntity->keyword() != "EDGE_CURVE" && pEntity->keyword() != "EDGE"))
    {
        return points;
    }
    const RStepRecord &record = pEntity->records[0];

    RStepVector p1, p2;
    if (!RStepGeometry::readVertex(this->file,record.param(1).toId(),p1) ||
        !RStepGeometry::readVertex(this->file,record.param(2).toId(),p2))
    {
        return points;
    }

    RStepCurve curve;
    if (pEntity->keyword() == "EDGE" || !RStepCurve::build(this->file,record.param(3).toId(),curve))
    {
        if (pEntity->keyword() != "EDGE")
        {
            this->nUnsupportedEdges++;
        }
        points = {p1,p2};
        return points;
    }
    bool sameSense = record.param(4).toBool(true);
    bool closed = (record.param(1).toId() == record.param(2).toId()) || ((p1 - p2).length() <= 1e-12 * std::max(p1.length(),1.0));

    if (curve.kind == RStepCurve::Line)
    {
        points = {p1,p2};
        return points;
    }

    double t1 = curve.project(p1);
    double t2 = curve.project(p2);
    if (curve.periodic)
    {
        double period = curve.period;
        if (closed)
        {
            t2 = sameSense ? t1 + period : t1 - period;
        }
        else if (sameSense)
        {
            while (t2 <= t1)
            {
                t2 += period;
            }
            while (t2 - t1 > period)
            {
                t2 -= period;
            }
        }
        else
        {
            while (t2 >= t1)
            {
                t2 -= period;
            }
            while (t1 - t2 > period)
            {
                t2 += period;
            }
        }
    }
    else if (closed)
    {
        t1 = sameSense ? curve.getMin() : curve.getMax();
        t2 = sameSense ? curve.getMax() : curve.getMin();
    }

    int nSamples = curve.getNSamples(t1,t2,this->maxAngle);
    std::function<void(double,const RStepVector&,double,const RStepVector&,int)> refine =
        [&](double ta, const RStepVector &pa, double tb, const RStepVector &pb, int depth)
    {
        double tm = 0.5 * (ta + tb);
        RStepVector pm = curve.eval(tm);
        RStepVector s = pb - pa;
        double l2 = s.dot(s);
        double f = (l2 > 0.0) ? std::clamp((pm - pa).dot(s) / l2,0.0,1.0) : 0.0;
        double deviation = (pa + s * f - pm).length();
        if (depth < 10 && deviation > this->chordTolerance)
        {
            refine(ta,pa,tm,pm,depth+1);
            points.push_back(pm);
            refine(tm,pm,tb,pb,depth+1);
        }
    };

    points.push_back(p1);
    RStepVector pa = curve.eval(t1);
    for (int i=1;i<=nSamples;i++)
    {
        double tb = t1 + (t2 - t1) * double(i) / double(nSamples);
        RStepVector pb = curve.eval(tb);
        refine(t1 + (t2 - t1) * double(i-1) / double(nSamples),pa,tb,pb,0);
        if (i < nSamples)
        {
            points.push_back(pb);
        }
        pa = pb;
    }
    points.push_back(p2);
    return points;
}

bool RStepTessellator::readLoop(unsigned int loopId, std::vector<RStepVector> &points)
{
    points.clear();
    const RStepEntity *pEntity = this->file.find(loopId);
    if (!pEntity)
    {
        return false;
    }
    const QString &keyword = pEntity->keyword();
    const RStepRecord &record = pEntity->records[0];

    auto samePoint = [](const RStepVector &a, const RStepVector &b)
    {
        return (a - b).length() <= 1e-12 * std::max(1.0,a.length());
    };

    if (keyword == "VERTEX_LOOP")
    {
        RStepVector p;
        if (RStepGeometry::readVertex(this->file,record.param(1).toId(),p))
        {
            points.push_back(p);
        }
        return true;
    }
    if (keyword == "POLY_LOOP")
    {
        for (unsigned int pointId : record.param(1).toIdList())
        {
            RStepVector p;
            if (!RStepGeometry::readPoint(this->file,pointId,p))
            {
                return false;
            }
            if (points.empty() || !samePoint(points.back(),p))
            {
                points.push_back(p);
            }
        }
    }
    else if (keyword == "EDGE_LOOP")
    {
        for (unsigned int orientedEdgeId : record.param(1).toIdList())
        {
            const RStepEntity *pOrientedEdge = this->file.find(orientedEdgeId);
            if (!pOrientedEdge)
            {
                return false;
            }
            unsigned int edgeId = orientedEdgeId;
            bool edgeOrientation = true;
            if (pOrientedEdge->keyword() == "ORIENTED_EDGE")
            {
                edgeId = pOrientedEdge->records[0].param(3).toId();
                edgeOrientation = pOrientedEdge->records[0].param(4).toBool(true);
            }
            std::vector<RStepVector> edgePoints = this->sampleEdge(edgeId);
            if (edgePoints.empty())
            {
                return false;
            }
            if (!edgeOrientation)
            {
                std::reverse(edgePoints.begin(),edgePoints.end());
            }
            for (const RStepVector &p : edgePoints)
            {
                if (points.empty() || !samePoint(points.back(),p))
                {
                    points.push_back(p);
                }
            }
        }
    }
    else
    {
        return false;
    }

    while (points.size() > 1 && samePoint(points.front(),points.back()))
    {
        points.pop_back();
    }
    return true;
}

std::shared_ptr<RStepSurface> RStepTessellator::getSurface(unsigned int surfaceId)
{
    auto iter = this->surfaceCache.find(surfaceId);
    if (iter != this->surfaceCache.end())
    {
        return iter->second;
    }
    auto surface = std::make_shared<RStepSurface>();
    if (!RStepSurface::build(this->file,surfaceId,this->angleScale,*surface))
    {
        surface.reset();
    }
    this->surfaceCache[surfaceId] = surface;
    return surface;
}

namespace
{

double wrapNearest(double x, double reference, double period)
{
    return x + period * std::round((reference - x) / period);
}

double signedArea(const std::vector<std::array<double,2>> &polygon)
{
    double area = 0.0;
    for (size_t i=0;i<polygon.size();i++)
    {
        const auto &a = polygon[i];
        const auto &b = polygon[(i+1)%polygon.size()];
        area += a[0] * b[1] - b[0] * a[1];
    }
    return 0.5 * area;
}

} // namespace

bool RStepTessellator::buildUVLoop(const RStepSurface &surface,
                                   const std::vector<RStepVector> &points,
                                   bool outer,
                                   bool sameSense,
                                   double size,
                                   UVLoop &loop) const
{
    int n = int(points.size());
    std::vector<std::array<double,2>> raw(n);
    std::vector<int> freeDim(n,-1);

    double extent[2];
    for (int d=0;d<2;d++)
    {
        extent[d] = surface.periodic[d] ? surface.period[d] : (surface.bounded[d] ? (surface.domainMax[d] - surface.domainMin[d]) : 1.0);
    }

    bool canBeSingular = (surface.kind != RStepSurface::Plane && surface.kind != RStepSurface::Cylinder);
    for (int i=0;i<n;i++)
    {
        // Free form surfaces are projected starting from previous loop point.
        bool useHint = (i > 0 && (surface.kind == RStepSurface::BSpline || surface.kind == RStepSurface::Offset));
        if (useHint)
        {
            raw[i] = raw[i-1];
        }
        surface.project(points[i],raw[i][0],raw[i][1],useHint);
        if (canBeSingular)
        {
            RStepVector du, dv;
            surface.derivatives(raw[i][0],raw[i][1],du,dv);
            if (du.length() * extent[0] < 1e-6 * size)
            {
                freeDim[i] = 0;
            }
            else if (dv.length() * extent[1] < 1e-6 * size)
            {
                freeDim[i] = 1;
            }
        }
    }

    std::vector<int> ns;
    for (int i=0;i<n;i++)
    {
        if (freeDim[i] < 0)
        {
            ns.push_back(i);
        }
    }
    int m = int(ns.size());
    if (m < 2)
    {
        return false;
    }

    // Gaps of singular points after each non-singular point.
    std::vector<int> gapFreeDim(m,-1);
    std::vector<int> crossings;
    for (int k=0;k<m;k++)
    {
        int next = (k + 1 < m) ? ns[k+1] : ns[0] + n;
        if (next - ns[k] > 1)
        {
            gapFreeDim[k] = freeDim[(ns[k] + 1) % n];
            crossings.push_back(k);
        }
    }

    // Unwrap with given extra period shifts at singular crossings.
    auto unwrap = [&](const std::vector<int> &extras, std::vector<std::array<double,2>> &cur, double net[2])
    {
        cur.assign(n,{0.0,0.0});
        cur[ns[0]] = raw[ns[0]];
        int c = 0;
        for (int k=1;k<=m;k++)
        {
            std::array<double,2> value = raw[ns[k % m]];
            const std::array<double,2> &previous = cur[ns[k-1]];
            for (int d=0;d<2;d++)
            {
                if (surface.periodic[d])
                {
                    value[d] = wrapNearest(value[d],previous[d],surface.period[d]);
                }
            }
            if (gapFreeDim[k-1] >= 0)
            {
                int fd = gapFreeDim[k-1];
                if (surface.periodic[fd] && c < int(extras.size()))
                {
                    value[fd] += double(extras[c]) * surface.period[fd];
                }
                c++;
            }
            if (k < m)
            {
                cur[ns[k]] = value;
            }
            else
            {
                for (int d=0;d<2;d++)
                {
                    net[d] = surface.periodic[d] ? surface.period[d] * std::round((value[d] - cur[ns[0]][d]) / surface.period[d]) : 0.0;
                }
            }
        }
    };

    int nAngular = std::max(4,int(std::ceil(2.0 * stepPi / this->maxAngle)));

    // Expand singular points into segments along singular parameter line.
    auto expand = [&](const std::vector<std::array<double,2>> &cur, const double net[2], std::vector<UVPoint> &result)
    {
        result.clear();
        for (int k=0;k<m;k++)
        {
            int index = ns[k];
            result.push_back({{cur[index][0],cur[index][1]},points[index],true,false});
            if (gapFreeDim[k] < 0)
            {
                continue;
            }
            int s = (index + 1) % n;
            int fd = gapFreeDim[k];
            int od = 1 - fd;
            std::array<double,2> previous = cur[index];
            std::array<double,2> next = (k + 1 < m) ? cur[ns[k+1]] : std::array<double,2>{cur[ns[0]][0] + net[0],cur[ns[0]][1] + net[1]};
            double singularValue = raw[s][od];
            if (surface.periodic[od])
            {
                singularValue = wrapNearest(singularValue,previous[od],surface.period[od]);
            }
            double a = previous[fd];
            double b = next[fd];
            int nSegments = 1;
            double reference = surface.periodic[fd] ? surface.period[fd] : extent[fd];
            if (reference > 0.0)
            {
                nSegments = std::clamp(int(std::ceil(std::abs(b - a) / reference * nAngular)),1,1000);
            }
            for (int j=0;j<=nSegments;j++)
            {
                if (j > 0 && std::abs(b - a) <= 0.0)
                {
                    break;
                }
                UVPoint point;
                point.uv[fd] = a + (b - a) * double(j) / double(nSegments);
                point.uv[od] = singularValue;
                point.p = points[s];
                point.hasPoint = true;
                point.singular = true;
                result.push_back(point);
            }
        }
    };

    std::vector<std::array<double,2>> cur;
    double net[2] = {0.0,0.0};

    std::vector<int> bestExtras(crossings.size(),0);
    bool needChoice = false;
    for (int k : crossings)
    {
        if (surface.periodic[gapFreeDim[k]])
        {
            needChoice = true;
        }
    }
    if (needChoice && crossings.size() <= 3)
    {
        double expectedSign = (outer == sameSense) ? 1.0 : -1.0;
        int nCombinations = 1;
        for (size_t i=0;i<crossings.size();i++)
        {
            nCombinations *= 3;
        }
        int bestScore = std::numeric_limits<int>::max();
        for (int combination=0;combination<nCombinations;combination++)
        {
            std::vector<int> extras(crossings.size());
            int code = combination;
            int cost = 0;
            for (size_t i=0;i<crossings.size();i++)
            {
                extras[i] = (code % 3) - 1;
                code /= 3;
                cost += std::abs(extras[i]);
            }
            double candidateNet[2];
            unwrap(extras,cur,candidateNet);
            std::vector<UVPoint> expanded;
            expand(cur,candidateNet,expanded);
            std::vector<std::array<double,2>> polygon;
            for (const UVPoint &point : expanded)
            {
                polygon.push_back({point.uv[0],point.uv[1]});
            }
            int score = cost;
            if (candidateNet[0] != 0.0 || candidateNet[1] != 0.0)
            {
                score += 1000;
            }
            if (signedArea(polygon) * expectedSign <= 0.0)
            {
                score += 100;
            }
            if (score < bestScore)
            {
                bestScore = score;
                bestExtras = extras;
            }
        }
    }

    unwrap(bestExtras,cur,net);
    expand(cur,net,loop.points);
    loop.net[0] = net[0];
    loop.net[1] = net[1];
    loop.outer = outer;
    return true;
}

bool RStepTessellator::triangulateFace(const RStepSurface &surface,
                                       const std::vector<std::vector<RStepVector>> &loops3D,
                                       const std::vector<bool> &outerFlags,
                                       bool sameSense,
                                       RStepMesh &mesh) const
{
    // Face size.
    RStepVector lo(stepInfinity,stepInfinity,stepInfinity);
    RStepVector hi(-stepInfinity,-stepInfinity,-stepInfinity);
    for (const std::vector<RStepVector> &loop : loops3D)
    {
        for (const RStepVector &p : loop)
        {
            lo = RStepVector(std::min(lo.x,p.x),std::min(lo.y,p.y),std::min(lo.z,p.z));
            hi = RStepVector(std::max(hi.x,p.x),std::max(hi.y,p.y),std::max(hi.z,p.z));
        }
    }
    double size = (hi.x >= lo.x) ? (hi - lo).length() : 0.0;
    if (size <= 0.0)
    {
        size = std::max(2.0 * std::max(surface.radius1,surface.radius2),this->chordTolerance * 1000.0);
    }

    // Loops in parameter space.
    bool singleLoop = (loops3D.size() == 1);
    std::vector<UVLoop> uvLoops;
    bool removedSeams = false;
    for (size_t i=0;i<loops3D.size();i++)
    {
        UVLoop loop;
        if (!this->buildUVLoop(surface,loops3D[i],outerFlags[i] || singleLoop,sameSense,size,loop))
        {
            continue;
        }
        if (loop.net[0] == 0.0 && loop.net[1] == 0.0)
        {
            // Zero area loops (edges traversed forth and back) are internal seams, not boundaries.
            double area = 0.0;
            double uvLo[2] = {stepInfinity,stepInfinity};
            double uvHi[2] = {-stepInfinity,-stepInfinity};
            for (size_t j=0;j<loop.points.size();j++)
            {
                const double *a = loop.points[j].uv;
                const double *b = loop.points[(j+1)%loop.points.size()].uv;
                area += 0.5 * (a[0] * b[1] - b[0] * a[1]);
                for (int d=0;d<2;d++)
                {
                    uvLo[d] = std::min(uvLo[d],a[d]);
                    uvHi[d] = std::max(uvHi[d],a[d]);
                }
            }
            double scale0 = 1.0, scale1 = 1.0;
            {
                RStepVector du, dv;
                surface.derivatives(loop.points[0].uv[0],loop.points[0].uv[1],du,dv);
                scale0 = std::max(du.length(),1e-300);
                scale1 = std::max(dv.length(),1e-300);
            }
            double diagonal2 = std::pow((uvHi[0] - uvLo[0]) * scale0,2) + std::pow((uvHi[1] - uvLo[1]) * scale1,2);
            if (std::abs(area) * scale0 * scale1 <= 1e-6 * diagonal2)
            {
                removedSeams = true;
                continue;
            }
        }
        uvLoops.push_back(loop);
    }
    if (!loops3D.empty() && uvLoops.empty())
    {
        bool fullDomain = removedSeams && ((surface.periodic[0] || surface.bounded[0]) && (surface.periodic[1] || surface.bounded[1]));
        if (!fullDomain)
        {
            // Degenerate face.
            return true;
        }
    }

    // Parameter scaling to approximate isotropic metric.
    double scale[2] = {0.0,0.0};
    {
        int nSamples = 0;
        auto addSample = [&](double u, double v)
        {
            RStepVector du, dv;
            surface.derivatives(u,v,du,dv);
            scale[0] += du.length();
            scale[1] += dv.length();
            nSamples++;
        };
        for (const UVLoop &loop : uvLoops)
        {
            for (const UVPoint &point : loop.points)
            {
                if (!point.singular)
                {
                    addSample(point.uv[0],point.uv[1]);
                }
            }
        }
        if (nSamples == 0)
        {
            double u = surface.bounded[0] ? 0.5 * (surface.domainMin[0] + surface.domainMax[0]) : 0.0;
            double v = surface.bounded[1] ? 0.5 * (surface.domainMin[1] + surface.domainMax[1]) : 0.0;
            addSample(u,v);
        }
        for (int d=0;d<2;d++)
        {
            scale[d] = (scale[d] > 0.0) ? scale[d] / double(nSamples) : 1.0;
        }
    }

    // Parameter bounding box (unscaled).
    double uvMin[2] = {stepInfinity,stepInfinity};
    double uvMax[2] = {-stepInfinity,-stepInfinity};
    for (const UVLoop &loop : uvLoops)
    {
        for (const UVPoint &point : loop.points)
        {
            for (int d=0;d<2;d++)
            {
                uvMin[d] = std::min(uvMin[d],point.uv[d]);
                uvMax[d] = std::max(uvMax[d],point.uv[d]);
            }
        }
    }
    for (int d=0;d<2;d++)
    {
        if (uvLoops.empty() || uvMin[d] > uvMax[d])
        {
            uvMin[d] = surface.domainMin[d];
            uvMax[d] = surface.periodic[d] ? surface.domainMin[d] + surface.period[d] : surface.domainMax[d];
        }
    }

    // Target interior spacing from curvature.
    double h = stepInfinity;
    if (!surface.isFlat())
    {
        double radius = surface.getCurvatureRadius();
        if (radius <= 0.0 && std::isfinite(uvMin[0]) && std::isfinite(uvMax[0]) && std::isfinite(uvMin[1]) && std::isfinite(uvMax[1]))
        {
            const int nGrid = 17;
            std::vector<RStepVector> gridPoints(nGrid * nGrid);
            std::vector<RStepVector> gridNormals(nGrid * nGrid);
            for (int i=0;i<nGrid;i++)
            {
                for (int j=0;j<nGrid;j++)
                {
                    double u = uvMin[0] + (uvMax[0] - uvMin[0]) * double(i) / double(nGrid - 1);
                    double v = uvMin[1] + (uvMax[1] - uvMin[1]) * double(j) / double(nGrid - 1);
                    gridPoints[i * nGrid + j] = surface.eval(u,v);
                    gridNormals[i * nGrid + j] = surface.normal(u,v);
                }
            }
            double curvature = 0.0;
            for (int i=0;i<nGrid;i++)
            {
                for (int j=0;j<nGrid;j++)
                {
                    for (int k=0;k<2;k++)
                    {
                        int i2 = i + (k == 0 ? 1 : 0);
                        int j2 = j + (k == 1 ? 1 : 0);
                        if (i2 >= nGrid || j2 >= nGrid)
                        {
                            continue;
                        }
                        const RStepVector &n1 = gridNormals[i * nGrid + j];
                        const RStepVector &n2 = gridNormals[i2 * nGrid + j2];
                        double distance = (gridPoints[i * nGrid + j] - gridPoints[i2 * nGrid + j2]).length();
                        if (distance > 1e-9 * size && n1.length() > 0.0 && n2.length() > 0.0)
                        {
                            double angle = std::acos(std::clamp(n1.dot(n2),-1.0,1.0));
                            curvature = std::max(curvature,angle / distance);
                        }
                    }
                }
            }
            radius = (curvature > 1e-6 / size) ? 1.0 / curvature : -1.0;
        }
        if (radius > 0.0)
        {
            h = std::min(radius * this->maxAngle,std::sqrt(8.0 * radius * this->chordTolerance));
        }
    }

    // Convert to scaled coordinates.
    for (UVLoop &loop : uvLoops)
    {
        for (UVPoint &point : loop.points)
        {
            point.uv[0] *= scale[0];
            point.uv[1] *= scale[1];
        }
        loop.net[0] *= scale[0];
        loop.net[1] *= scale[1];
    }
    double period[2] = {surface.period[0] * scale[0],surface.period[1] * scale[1]};
    double domainMin[2] = {surface.domainMin[0] * scale[0],surface.domainMin[1] * scale[1]};
    double domainMax[2] = {surface.domainMax[0] * scale[0],surface.domainMax[1] * scale[1]};
    {
        double faceSize = std::hypot((uvMax[0] - uvMin[0]) * scale[0],(uvMax[1] - uvMin[1]) * scale[1]);
        if (std::isfinite(faceSize) && faceSize > 0.0)
        {
            h = std::max(h,faceSize / 100.0);
        }
    }

    int nAngular = std::max(4,int(std::ceil(2.0 * stepPi / this->maxAngle)));

    // Append constructed points between a and b (both excluded).
    auto appendSegment = [&](std::vector<UVPoint> &ring, const UVPoint &a, const UVPoint &b)
    {
        double du = b.uv[0] - a.uv[0];
        double dv = b.uv[1] - a.uv[1];
        int nSegments = 1;
        if (std::isfinite(h))
        {
            nSegments = std::max(nSegments,int(std::ceil(std::hypot(du,dv) / h)));
        }
        if (surface.periodic[0] && period[0] > 0.0)
        {
            nSegments = std::max(nSegments,int(std::ceil(std::abs(du) / period[0] * nAngular)));
        }
        if (surface.periodic[1] && period[1] > 0.0)
        {
            nSegments = std::max(nSegments,int(std::ceil(std::abs(dv) / period[1] * nAngular)));
        }
        nSegments = std::min(nSegments,1000);
        for (int i=1;i<nSegments;i++)
        {
            double f = double(i) / double(nSegments);
            UVPoint point;
            point.uv[0] = a.uv[0] + du * f;
            point.uv[1] = a.uv[1] + dv * f;
            point.hasPoint = false;
            point.singular = false;
            ring.push_back(point);
        }
    };

    auto makePoint = [](double u, double v)
    {
        UVPoint point;
        point.uv[0] = u;
        point.uv[1] = v;
        point.hasPoint = false;
        point.singular = false;
        return point;
    };

    auto liftPath = [](const UVLoop &loop, size_t start)
    {
        std::vector<UVPoint> path;
        size_t m = loop.points.size();
        for (size_t k=0;k<=m;k++)
        {
            UVPoint point = loop.points[(start + k) % m];
            if (start + k >= m)
            {
                point.uv[0] += loop.net[0];
                point.uv[1] += loop.net[1];
            }
            path.push_back(point);
        }
        return path;
    };

    auto pointInRing = [](const std::vector<UVPoint> &ring, double x, double y)
    {
        bool inside = false;
        for (size_t i=0, j=ring.size()-1;i<ring.size();j=i++)
        {
            const double *a = ring[i].uv;
            const double *b = ring[j].uv;
            if ((a[1] > y) != (b[1] > y))
            {
                double xc = a[0] + (y - a[1]) * (b[0] - a[0]) / (b[1] - a[1]);
                if (x < xc)
                {
                    inside = !inside;
                }
            }
        }
        return inside;
    };

    auto ringArea = [](const std::vector<UVPoint> &ring)
    {
        double area = 0.0;
        for (size_t i=0;i<ring.size();i++)
        {
            const double *a = ring[i].uv;
            const double *b = ring[(i+1)%ring.size()].uv;
            area += a[0] * b[1] - b[0] * a[1];
        }
        return 0.5 * area;
    };

    // Assemble outer polygon and holes.
    std::vector<int> spanning;
    std::vector<int> closedLoops;
    for (size_t i=0;i<uvLoops.size();i++)
    {
        if (uvLoops[i].net[0] != 0.0 && uvLoops[i].net[1] != 0.0)
        {
            return false;
        }
        if (uvLoops[i].net[0] != 0.0 || uvLoops[i].net[1] != 0.0)
        {
            spanning.push_back(int(i));
        }
        else
        {
            closedLoops.push_back(int(i));
        }
    }

    std::vector<UVPoint> outer;
    std::vector<std::vector<UVPoint>> holes;

    if (spanning.empty())
    {
        if (closedLoops.empty())
        {
            // Whole surface domain.
            double lo2[2], hi2[2];
            for (int d=0;d<2;d++)
            {
                if (surface.periodic[d])
                {
                    lo2[d] = domainMin[d];
                    hi2[d] = domainMin[d] + period[d];
                }
                else if (surface.bounded[d])
                {
                    lo2[d] = domainMin[d];
                    hi2[d] = domainMax[d];
                }
                else
                {
                    return false;
                }
            }
            UVPoint corners[4] = {makePoint(lo2[0],lo2[1]),makePoint(hi2[0],lo2[1]),makePoint(hi2[0],hi2[1]),makePoint(lo2[0],hi2[1])};
            for (int i=0;i<4;i++)
            {
                outer.push_back(corners[i]);
                appendSegment(outer,corners[i],corners[(i+1)%4]);
            }
        }
        else
        {
            int outerLoop = -1;
            int nFlagged = 0;
            for (int i : closedLoops)
            {
                if (uvLoops[i].outer && !singleLoop)
                {
                    nFlagged++;
                    outerLoop = i;
                }
            }
            if (nFlagged != 1)
            {
                double maxArea = -1.0;
                for (int i : closedLoops)
                {
                    double area = std::abs(ringArea(uvLoops[i].points));
                    if (area > maxArea)
                    {
                        maxArea = area;
                        outerLoop = i;
                    }
                }
            }
            bool closedSurface = ((surface.periodic[0] || surface.bounded[0]) && (surface.periodic[1] || surface.bounded[1]) &&
                                  (surface.periodic[0] || surface.periodic[1]));
            if (closedSurface && (ringArea(uvLoops[outerLoop].points) > 0.0) != sameSense)
            {
                // Loop orientation says the face lies outside of all loops - whole surface domain with loops as holes.
                // Domain is cut in periodic directions where no loop crosses.
                double lo2[2], hi2[2];
                for (int d=0;d<2;d++)
                {
                    if (!surface.periodic[d])
                    {
                        lo2[d] = domainMin[d];
                        hi2[d] = domainMax[d];
                        continue;
                    }
                    std::vector<std::pair<double,double>> ranges;
                    for (int i : closedLoops)
                    {
                        double rMin = stepInfinity, rMax = -stepInfinity;
                        for (const UVPoint &point : uvLoops[i].points)
                        {
                            rMin = std::min(rMin,point.uv[d]);
                            rMax = std::max(rMax,point.uv[d]);
                        }
                        ranges.push_back({rMin,rMax});
                    }
                    double cut = domainMin[d];
                    for (const auto &candidate : ranges)
                    {
                        double c = candidate.second + 1e-6 * period[d];
                        bool free = true;
                        for (const auto &range : ranges)
                        {
                            double shifted = wrapNearest(c,0.5 * (range.first + range.second),period[d]);
                            if (shifted > range.first && shifted < range.second)
                            {
                                free = false;
                                break;
                            }
                        }
                        if (free)
                        {
                            cut = c;
                            break;
                        }
                    }
                    lo2[d] = cut;
                    hi2[d] = cut + period[d];
                }
                UVPoint corners[4] = {makePoint(lo2[0],lo2[1]),makePoint(hi2[0],lo2[1]),makePoint(hi2[0],hi2[1]),makePoint(lo2[0],hi2[1])};
                for (int i=0;i<4;i++)
                {
                    outer.push_back(corners[i]);
                    appendSegment(outer,corners[i],corners[(i+1)%4]);
                }
                for (int i : closedLoops)
                {
                    holes.push_back(uvLoops[i].points);
                }
            }
            else
            {
                outer = uvLoops[outerLoop].points;
                for (int i : closedLoops)
                {
                    if (i != outerLoop)
                    {
                        holes.push_back(uvLoops[i].points);
                    }
                }
            }
        }
    }
    else if (spanning.size() == 1)
    {
        const UVLoop &loop = uvLoops[spanning[0]];
        int d = (loop.net[0] != 0.0) ? 0 : 1;
        int o = 1 - d;
        bool positive = (loop.net[d] > 0.0);
        bool wantMax = (d == 0) ? (positive == sameSense) : (positive != sameSense);
        double pole = wantMax ? domainMax[o] : domainMin[o];
        if (!std::isfinite(pole) || surface.periodic[o])
        {
            return false;
        }
        // Start at the vertex closest to the pole - connectors to the pole cannot cross the loop.
        size_t start = 0;
        for (size_t i=1;i<loop.points.size();i++)
        {
            if ((wantMax && loop.points[i].uv[o] > loop.points[start].uv[o]) ||
                (!wantMax && loop.points[i].uv[o] < loop.points[start].uv[o]))
            {
                start = i;
            }
        }
        std::vector<UVPoint> path = liftPath(loop,start);
        outer.insert(outer.end(),path.begin(),path.end());
        UVPoint c1, c2;
        c1.uv[d] = path.back().uv[d];
        c1.uv[o] = pole;
        c2.uv[d] = path.front().uv[d];
        c2.uv[o] = pole;
        c1.hasPoint = c2.hasPoint = false;
        c1.singular = c2.singular = false;
        appendSegment(outer,path.back(),c1);
        outer.push_back(c1);
        appendSegment(outer,c1,c2);
        outer.push_back(c2);
        appendSegment(outer,c2,path.front());
        for (int i : closedLoops)
        {
            holes.push_back(uvLoops[i].points);
        }
    }
    else if (spanning.size() == 2)
    {
        UVLoop loopA = uvLoops[spanning[0]];
        UVLoop loopB = uvLoops[spanning[1]];
        int d = (loopA.net[0] != 0.0) ? 0 : 1;
        int o = 1 - d;
        if (surface.periodic[o])
        {
            // Band between two loops may run either way around periodic direction - face lies
            // left of its loops, so the lower loop has to run in positive direction (for same sense).
            double meanA = 0.0, meanB = 0.0;
            for (const UVPoint &point : loopA.points)
            {
                meanA += point.uv[o] / double(loopA.points.size());
            }
            for (const UVPoint &point : loopB.points)
            {
                meanB += point.uv[o] / double(loopB.points.size());
            }
            UVLoop &lower = (meanA < meanB) ? loopA : loopB;
            bool expectedPositive = (d == 0) ? sameSense : !sameSense;
            if ((lower.net[d] > 0.0) != expectedPositive)
            {
                for (UVPoint &point : lower.points)
                {
                    point.uv[o] += period[o];
                }
            }
        }
        if ((loopB.net[d] == 0.0))
        {
            return false;
        }
        auto reverseLoop = [](UVLoop &loop)
        {
            std::reverse(loop.points.begin(),loop.points.end());
            loop.net[0] = -loop.net[0];
            loop.net[1] = -loop.net[1];
        };
        if (loopA.net[d] < 0.0)
        {
            std::swap(loopA,loopB);
        }
        if (loopA.net[d] < 0.0)
        {
            reverseLoop(loopA);
        }
        if (loopB.net[d] > 0.0)
        {
            reverseLoop(loopB);
        }
        double p = loopA.net[d];

        // Connector candidates - for every vertex of loop A the nearest vertex of loop B.
        std::vector<std::tuple<double,size_t,size_t>> pairs;
        for (size_t ia=0;ia<loopA.points.size();ia++)
        {
            double target = loopA.points[ia].uv[d] + p;
            double bestLength = stepInfinity;
            size_t bestB = 0;
            for (size_t ib=0;ib<loopB.points.size();ib++)
            {
                double du = wrapNearest(loopB.points[ib].uv[d],target,p) - target;
                double dv = loopB.points[ib].uv[o] - loopA.points[ia].uv[o];
                double length = std::hypot(du,dv);
                if (length < bestLength)
                {
                    bestLength = length;
                    bestB = ib;
                }
            }
            pairs.push_back({bestLength,ia,bestB});
        }
        std::sort(pairs.begin(),pairs.end());

        auto crosses = [](const double *a, const double *b, const double *c, const double *e)
        {
            auto orient = [](const double *p, const double *q, const double *r) { return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0]); };
            double o1 = orient(a,b,c), o2 = orient(a,b,e), o3 = orient(c,e,a), o4 = orient(c,e,b);
            return (((o1 > 0.0 && o2 < 0.0) || (o1 < 0.0 && o2 > 0.0)) && ((o3 > 0.0 && o4 < 0.0) || (o3 < 0.0 && o4 > 0.0)));
        };

        std::vector<UVPoint> pathA, pathB;
        size_t nTries = std::min(pairs.size(),size_t(64));
        for (size_t k=0;k<nTries;k++)
        {
            size_t ia = std::get<1>(pairs[k]);
            size_t ib = std::get<2>(pairs[k]);
            UVLoop shiftedB = loopB;
            double target = loopA.points[ia].uv[d] + p;
            double shift = wrapNearest(loopB.points[ib].uv[d],target,p) - loopB.points[ib].uv[d];
            for (UVPoint &point : shiftedB.points)
            {
                point.uv[d] += shift;
            }
            pathA = liftPath(loopA,ia);
            pathB = liftPath(shiftedB,ib);
            // Connector must not cross any boundary segment - holes are tested in all periodic positions.
            const double *connector[2] = {pathA.back().uv,pathB.front().uv};
            bool valid = true;
            for (const std::vector<UVPoint> *pPath : {&pathA,&pathB})
            {
                for (size_t i=0;i+1<pPath->size() && valid;i++)
                {
                    for (int s=-1;s<=0 && valid;s++)
                    {
                        // Second connector is the first one shifted by one period.
                        double a0[2] = {connector[0][0],connector[0][1]};
                        double a1[2] = {connector[1][0],connector[1][1]};
                        a0[d] += s * p;
                        a1[d] += s * p;
                        valid = !crosses(a0,a1,(*pPath)[i].uv,(*pPath)[i+1].uv);
                    }
                }
            }
            for (size_t h=0;h<closedLoops.size() && valid;h++)
            {
                const std::vector<UVPoint> &hole = uvLoops[closedLoops[h]].points;
                for (int s=-2;s<=2 && valid;s++)
                {
                    for (size_t i=0;i<hole.size() && valid;i++)
                    {
                        double h0[2] = {hole[i].uv[0],hole[i].uv[1]};
                        double h1[2] = {hole[(i+1)%hole.size()].uv[0],hole[(i+1)%hole.size()].uv[1]};
                        h0[d] += s * p;
                        h1[d] += s * p;
                        valid = !crosses(connector[0],connector[1],h0,h1);
                    }
                }
            }
            if (valid)
            {
                break;
            }
        }
        outer.insert(outer.end(),pathA.begin(),pathA.end());
        appendSegment(outer,pathA.back(),pathB.front());
        outer.insert(outer.end(),pathB.begin(),pathB.end());
        appendSegment(outer,pathB.back(),pathA.front());
        for (int i : closedLoops)
        {
            holes.push_back(uvLoops[i].points);
        }
    }
    else
    {
        return false;
    }

    if (outer.size() < 3)
    {
        return true;
    }

    // Holes on periodic surfaces may be shifted by period.
    for (std::vector<UVPoint> &hole : holes)
    {
        if (hole.empty() || pointInRing(outer,hole[0].uv[0],hole[0].uv[1]))
        {
            continue;
        }
        bool found = false;
        for (int ku=-2;ku<=2 && !found;ku++)
        {
            for (int kv=-2;kv<=2 && !found;kv++)
            {
                if ((ku != 0 && !surface.periodic[0]) || (kv != 0 && !surface.periodic[1]))
                {
                    continue;
                }
                double su = ku * period[0];
                double sv = kv * period[1];
                if (pointInRing(outer,hole[0].uv[0] + su,hole[0].uv[1] + sv))
                {
                    for (UVPoint &point : hole)
                    {
                        point.uv[0] += su;
                        point.uv[1] += sv;
                    }
                    found = true;
                }
            }
        }
    }

    // Collect points and constraints.
    std::vector<UVPoint> uvPoints;
    std::vector<RConstrainedDelaunay::Point> cdtPoints;
    std::vector<std::pair<unsigned int,unsigned int>> constraints;
    std::vector<const std::vector<UVPoint>*> rings = {&outer};
    for (const std::vector<UVPoint> &hole : holes)
    {
        if (hole.size() >= 2)
        {
            rings.push_back(&hole);
        }
    }
    for (const std::vector<UVPoint> *pRing : rings)
    {
        unsigned int first = unsigned(uvPoints.size());
        unsigned int count = unsigned(pRing->size());
        for (unsigned int i=0;i<count;i++)
        {
            uvPoints.push_back((*pRing)[i]);
            cdtPoints.push_back({(*pRing)[i].uv[0],(*pRing)[i].uv[1]});
            constraints.push_back({first + i,first + (i + 1) % count});
        }
    }

    // Interior points on triangular lattice.
    if (std::isfinite(h) && h > 0.0)
    {
        double xMin = stepInfinity, xMax = -stepInfinity, yMin = stepInfinity, yMax = -stepInfinity;
        for (const UVPoint &point : outer)
        {
            xMin = std::min(xMin,point.uv[0]);
            xMax = std::max(xMax,point.uv[0]);
            yMin = std::min(yMin,point.uv[1]);
            yMax = std::max(yMax,point.uv[1]);
        }
        double width = xMax - xMin;
        double height = yMax - yMin;
        const double maxPoints = 40000.0;
        if ((width / h) * (height / h) > maxPoints)
        {
            h = std::sqrt(width * height / maxPoints);
        }
        double dy = h * std::sqrt(3.0) / 2.0;
        double minDistance = 0.6 * h;

        // Segment buckets for distance queries.
        int nx = std::max(1,int(std::ceil(width / h)));
        int ny = std::max(1,int(std::ceil(height / h)));
        std::vector<std::vector<unsigned int>> buckets(size_t(nx) * size_t(ny));
        auto bucketIndex = [&](double x, double y, int &ix, int &iy)
        {
            ix = std::clamp(int((x - xMin) / h),0,nx-1);
            iy = std::clamp(int((y - yMin) / h),0,ny-1);
        };
        for (unsigned int s=0;s<constraints.size();s++)
        {
            const RConstrainedDelaunay::Point &a = cdtPoints[constraints[s].first];
            const RConstrainedDelaunay::Point &b = cdtPoints[constraints[s].second];
            int ix1, iy1, ix2, iy2;
            bucketIndex(std::min(a.x,b.x),std::min(a.y,b.y),ix1,iy1);
            bucketIndex(std::max(a.x,b.x),std::max(a.y,b.y),ix2,iy2);
            for (int ix=ix1;ix<=ix2;ix++)
            {
                for (int iy=iy1;iy<=iy2;iy++)
                {
                    buckets[size_t(iy) * nx + ix].push_back(s);
                }
            }
        }

        int nRows = int(std::floor(height / dy));
        for (int row=1;row<=nRows;row++)
        {
            double y = yMin + row * dy;
            // Row crossings for even-odd inside test.
            std::vector<double> crossingsX;
            for (const auto &constraint : constraints)
            {
                const RConstrainedDelaunay::Point &a = cdtPoints[constraint.first];
                const RConstrainedDelaunay::Point &b = cdtPoints[constraint.second];
                if ((a.y > y) != (b.y > y))
                {
                    crossingsX.push_back(a.x + (y - a.y) * (b.x - a.x) / (b.y - a.y));
                }
            }
            std::sort(crossingsX.begin(),crossingsX.end());
            double offset = (row % 2 == 0) ? 0.0 : 0.5 * h;
            for (double x=xMin + offset + 0.5 * h;x<xMax;x+=h)
            {
                size_t nLeft = std::lower_bound(crossingsX.begin(),crossingsX.end(),x) - crossingsX.begin();
                if (nLeft % 2 == 0)
                {
                    continue;
                }
                int ix, iy;
                bucketIndex(x,y,ix,iy);
                bool tooClose = false;
                for (int jx=std::max(ix-1,0);jx<=std::min(ix+1,nx-1) && !tooClose;jx++)
                {
                    for (int jy=std::max(iy-1,0);jy<=std::min(iy+1,ny-1) && !tooClose;jy++)
                    {
                        for (unsigned int s : buckets[size_t(jy) * nx + jx])
                        {
                            const RConstrainedDelaunay::Point &a = cdtPoints[constraints[s].first];
                            const RConstrainedDelaunay::Point &b = cdtPoints[constraints[s].second];
                            double sx = b.x - a.x, sy = b.y - a.y;
                            double l2 = sx * sx + sy * sy;
                            double f = (l2 > 0.0) ? std::clamp(((x - a.x) * sx + (y - a.y) * sy) / l2,0.0,1.0) : 0.0;
                            if (std::hypot(a.x + sx * f - x,a.y + sy * f - y) < minDistance)
                            {
                                tooClose = true;
                                break;
                            }
                        }
                    }
                }
                if (!tooClose)
                {
                    uvPoints.push_back(makePoint(x,y));
                    cdtPoints.push_back({x,y});
                }
            }
        }
    }

    RConstrainedDelaunay cdt;
    std::vector<RConstrainedDelaunay::Triangle> triangles;
    if (!cdt.triangulate(cdtPoints,constraints,triangles))
    {
        return false;
    }

    // Map to 3D.
    std::vector<int> nodeMap(uvPoints.size(),-1);
    for (const auto &triangle : triangles)
    {
        unsigned int ids[3];
        for (int k=0;k<3;k++)
        {
            unsigned int i = triangle[k];
            if (nodeMap[i] < 0)
            {
                const UVPoint &point = uvPoints[i];
                nodeMap[i] = int(mesh.nodes.size());
                mesh.nodes.push_back(point.hasPoint ? point.p : surface.eval(point.uv[0] / scale[0],point.uv[1] / scale[1]));
            }
            ids[k] = unsigned(nodeMap[i]);
        }
        if (sameSense)
        {
            mesh.triangles.push_back({ids[0],ids[1],ids[2]});
        }
        else
        {
            mesh.triangles.push_back({ids[0],ids[2],ids[1]});
        }
    }
    return true;
}
