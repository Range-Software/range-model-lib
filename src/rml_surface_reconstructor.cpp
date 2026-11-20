#include <algorithm>
#include <cmath>
#include <deque>
#include <set>
#include <unordered_map>

#include <rbl_error.h>
#include <rbl_logger.h>
#include <rbl_utils.h>

#include "rml_constrained_delaunay.h"
#include "rml_intersection_search.h"
#include "rml_surface_reconstructor.h"

typedef RSurfaceReconstructor::Vec3 Vec3;

namespace
{

inline Vec3 sub(const Vec3 &a, const Vec3 &b)
{
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

inline Vec3 add(const Vec3 &a, const Vec3 &b)
{
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

inline Vec3 scale(const Vec3 &a, double s)
{
    return {a.x * s, a.y * s, a.z * s};
}

inline double dot(const Vec3 &a, const Vec3 &b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline Vec3 cross(const Vec3 &a, const Vec3 &b)
{
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

inline double norm(const Vec3 &a)
{
    return std::sqrt(dot(a,a));
}

inline Vec3 normalized(const Vec3 &a)
{
    double l = norm(a);
    return (l > 0.0) ? scale(a,1.0/l) : Vec3{0.0,0.0,0.0};
}

//! Distance of point p from line given by points a and b.
inline double distanceToLine(const Vec3 &p, const Vec3 &a, const Vec3 &b)
{
    Vec3 d = sub(b,a);
    double l = norm(d);
    if (l <= 0.0)
    {
        return norm(sub(p,a));
    }
    return norm(cross(sub(p,a),d)) / l;
}

//! Check if triangle contains vertex.
inline bool triHas(const int v[3], int a)
{
    return (v[0] == a || v[1] == a || v[2] == a);
}

//! Check if triangle traverses directed edge a->b.
inline bool triTraverses(const int v[3], int a, int b)
{
    for (int i=0;i<3;i++)
    {
        if (v[i] == a && v[(i+1)%3] == b)
        {
            return true;
        }
    }
    return false;
}

//! Remove value from vector (order is not preserved).
inline void eraseValue(std::vector<int> &values, int value)
{
    for (size_t i=0;i<values.size();i++)
    {
        if (values[i] == value)
        {
            values[i] = values.back();
            values.pop_back();
            return;
        }
    }
}

} // namespace


RSurfaceReconstructor::Options::Options()
    : angleTolerance(R_DEG_TO_RAD(1.0))
    , distanceTolerance(0.0)
    , minEdgeLength(0.0)
    , sliverAngle(R_DEG_TO_RAD(5.0))
    , delaunay(false)
    , refine(false)
    , qualityAngle(R_DEG_TO_RAD(20.0))
    , maxEdgeLength(0.0)
{
}


RSurfaceReconstructor::Statistics::Statistics()
    : nInputTriangles(0)
    , nOutputTriangles(0)
    , nRemovedNodes(0)
    , nCollapsedEdges(0)
    , nFlippedEdges(0)
    , nPlanarRegions(0)
    , nRetriangulatedRegions(0)
    , nFailedRegions(0)
    , nFrozenTriangles(0)
    , nDelaunayFlips(0)
    , nInsertedNodes(0)
    , nSmoothedNodes(0)
{
}


RSurfaceReconstructor::RSurfaceReconstructor(const std::vector<RNode> &nodes,
                                             const std::vector<Triangle> &triangles,
                                             const std::vector<bool> &lockedNodes,
                                             const std::vector<std::pair<uint, uint>> &lockedEdges,
                                             const Options &options)
    : nodes(nodes)
    , inputTriangles(triangles)
    , inputLockedNodes(lockedNodes)
    , inputLockedEdges(lockedEdges)
    , options(options)
{
    for (const Triangle &triangle : triangles)
    {
        for (uint nodeID : triangle.nodes)
        {
            if (nodeID >= nodes.size())
            {
                throw RError(RError::Type::InvalidInput,R_ERROR_REF,"Triangle node ID %u is out of range (%u nodes).",nodeID,uint(nodes.size()));
            }
        }
    }

    // Resolve automatic tolerances.
    bool first = true;
    Vec3 lower = {0.0,0.0,0.0};
    Vec3 upper = {0.0,0.0,0.0};
    for (const Triangle &triangle : triangles)
    {
        for (uint nodeID : triangle.nodes)
        {
            Vec3 p = {nodes[nodeID].getX(),nodes[nodeID].getY(),nodes[nodeID].getZ()};
            if (first)
            {
                lower = upper = p;
                first = false;
            }
            lower = {std::min(lower.x,p.x),std::min(lower.y,p.y),std::min(lower.z,p.z)};
            upper = {std::max(upper.x,p.x),std::max(upper.y,p.y),std::max(upper.z,p.z)};
        }
    }
    double diagonal = norm(sub(upper,lower));
    if (diagonal <= 0.0)
    {
        diagonal = 1.0;
    }

    this->options.angleTolerance = std::clamp(this->options.angleTolerance,0.0,RConstants::pi/2.0);
    this->options.sliverAngle = std::clamp(this->options.sliverAngle,0.0,RConstants::pi/3.0);
    this->options.qualityAngle = std::clamp(this->options.qualityAngle,0.0,R_DEG_TO_RAD(34.0));
    if (this->options.maxEdgeLength < 0.0)
    {
        this->options.maxEdgeLength = 0.0;
    }
    if (this->options.distanceTolerance <= 0.0)
    {
        this->options.distanceTolerance = 1.0e-5 * diagonal;
    }
    if (this->options.minEdgeLength <= 0.0)
    {
        this->options.minEdgeLength = 10.0 * this->options.distanceTolerance;
    }
    this->sinAngleTolerance = std::sin(this->options.angleTolerance);
    this->cosAngleTolerance = std::cos(this->options.angleTolerance);
}


const RSurfaceReconstructor::Options &RSurfaceReconstructor::getOptions() const
{
    return this->options;
}


const std::vector<RNode> &RSurfaceReconstructor::getNewNodes() const
{
    return this->newNodes;
}


std::vector<RSurfaceReconstructor::Triangle> RSurfaceReconstructor::reconstruct(Statistics &statistics)
{
    const uint nInput = uint(this->inputTriangles.size());

    RLogger::info("Reconstructing surface (%u triangles)\n",nInput);
    RLogger::indent();
    RLogger::info("Angle tolerance:    %g deg\n",R_RAD_TO_DEG(this->options.angleTolerance));
    RLogger::info("Distance tolerance: %g\n",this->options.distanceTolerance);
    RLogger::info("Min. edge length:   %g\n",this->options.minEdgeLength);
    RLogger::info("Sliver angle:       %g deg\n",R_RAD_TO_DEG(this->options.sliverAngle));
    if (this->options.refine)
    {
        RLogger::info("Quality angle:      %g deg\n",R_RAD_TO_DEG(this->options.qualityAngle));
        if (this->options.maxEdgeLength > 0.0)
        {
            RLogger::info("Max. edge length:   %g\n",this->options.maxEdgeLength);
        }
    }

    // Input triangles (node sets) - unchanged triangles are not checked for intersections,
    // so that intersections already present in the input are left alone.
    auto triangleKey = [](const Triangle &triangle) -> std::array<uint,4>
    {
        std::array<uint,4> key = {triangle.nodes[0],triangle.nodes[1],triangle.nodes[2],triangle.surfaceID};
        std::sort(key.begin(),key.begin()+3);
        return key;
    };
    std::set<std::array<uint,4>> inputKeys;
    for (const Triangle &triangle : this->inputTriangles)
    {
        inputKeys.insert(triangleKey(triangle));
    }

    auto findBounds = [&](const std::vector<RNode> &nodes, const Triangle &triangle) -> std::array<double,6>
    {
        std::array<double,6> bounds = {0.0,0.0,0.0,0.0,0.0,0.0};
        for (uint k=0;k<3;k++)
        {
            const RNode &node = nodes[triangle.nodes[k]];
            const double p[3] = {node.getX(),node.getY(),node.getZ()};
            for (uint l=0;l<3;l++)
            {
                bounds[l]   = (k == 0) ? p[l] : std::min(bounds[l],p[l]);
                bounds[l+3] = (k == 0) ? p[l] : std::max(bounds[l+3],p[l]);
            }
        }
        return bounds;
    };

    std::vector<std::array<double,6>> inputBounds(nInput);
    std::array<double,6> modelBounds = {0.0,0.0,0.0,0.0,0.0,0.0};
    for (uint i=0;i<nInput;i++)
    {
        inputBounds[i] = findBounds(this->nodes,this->inputTriangles[i]);
        for (uint l=0;l<3;l++)
        {
            modelBounds[l]   = (i == 0) ? inputBounds[i][l]   : std::min(modelBounds[l],inputBounds[i][l]);
            modelBounds[l+3] = (i == 0) ? inputBounds[i][l+3] : std::max(modelBounds[l+3],inputBounds[i][l+3]);
        }
    }
    const double diagonal = std::hypot(modelBounds[3]-modelBounds[0],modelBounds[4]-modelBounds[1],modelBounds[5]-modelBounds[2]);

    std::vector<bool> frozenTriangles(nInput,false);
    std::vector<Triangle> triangles;
    // Model nodes extended by new nodes.
    std::vector<RNode> allNodes;

    for (uint iteration=1;;iteration++)
    {
        this->initialize(frozenTriangles);
        triangles = this->run(statistics);

        // Frozen triangles are passed unchanged.
        uint nFrozen = 0;
        for (uint i=0;i<nInput;i++)
        {
            if (frozenTriangles[i])
            {
                triangles.push_back(this->inputTriangles[i]);
                nFrozen++;
            }
        }
        statistics.nFrozenTriangles = nFrozen;

        const std::vector<RNode> *currentNodes = &this->nodes;
        if (!this->newNodes.empty())
        {
            allNodes = this->nodes;
            allNodes.insert(allNodes.end(),this->newNodes.begin(),this->newNodes.end());
            currentNodes = &allNodes;
        }

        // Find new triangles which intersect other triangles.
        std::vector<RElement> elements(triangles.size(),RElement(R_ELEMENT_TRI1));
        std::vector<uint> elementIDs(triangles.size());
        for (uint i=0;i<triangles.size();i++)
        {
            for (uint k=0;k<3;k++)
            {
                elements[i].setNodeId(k,triangles[i].nodes[k]);
            }
            elementIDs[i] = i;
        }
        std::vector<unsigned char> intersected = RIntersectionSearch(elements,*currentNodes,elementIDs).detect();

        std::vector<std::array<double,6>> intersectedBounds;
        for (uint i=0;i<triangles.size();i++)
        {
            if (!intersected[i] || inputKeys.find(triangleKey(triangles[i])) != inputKeys.end())
            {
                continue;
            }
            intersectedBounds.push_back(findBounds(*currentNodes,triangles[i]));
        }
        if (intersectedBounds.empty())
        {
            break;
        }

        // Freeze input triangles in the neighborhood of new intersections and repeat.
        // The neighborhood starts with the bounding box of the intersected triangle and
        // grows only if no further input triangle could be frozen.
        uint nNewFrozen = 0;
        double inflation = 0.0;
        while (nNewFrozen == 0 && inflation <= diagonal)
        {
            for (uint i=0;i<nInput;i++)
            {
                if (frozenTriangles[i])
                {
                    continue;
                }
                const std::array<double,6> &inputBound = inputBounds[i];
                for (const std::array<double,6> &bounds : intersectedBounds)
                {
                    if (inputBound[0] <= bounds[3] + inflation && inputBound[3] >= bounds[0] - inflation &&
                        inputBound[1] <= bounds[4] + inflation && inputBound[4] >= bounds[1] - inflation &&
                        inputBound[2] <= bounds[5] + inflation && inputBound[5] >= bounds[2] - inflation)
                    {
                        frozenTriangles[i] = true;
                        nNewFrozen++;
                        break;
                    }
                }
            }
            inflation = (inflation > 0.0) ? 4.0 * inflation : this->options.distanceTolerance;
        }
        RLogger::info("New intersections: %u triangles, freezing %u more input triangles (iteration %u)\n",
                      uint(intersectedBounds.size()),nNewFrozen,iteration);
        if (nNewFrozen == 0)
        {
            RLogger::warning("Could not avoid %u new intersections.\n",uint(intersectedBounds.size()));
            break;
        }
    }

    statistics.nInputTriangles = nInput;
    statistics.nOutputTriangles = uint(triangles.size());

    // Removed nodes.
    std::vector<bool> usedBefore(this->nodes.size() + this->newNodes.size(),false);
    std::vector<bool> usedAfter(this->nodes.size() + this->newNodes.size(),false);
    for (const Triangle &triangle : this->inputTriangles)
    {
        for (uint nodeID : triangle.nodes)
        {
            usedBefore[nodeID] = true;
        }
    }
    for (const Triangle &triangle : triangles)
    {
        for (uint nodeID : triangle.nodes)
        {
            usedAfter[nodeID] = true;
        }
    }
    statistics.nRemovedNodes = 0;
    for (size_t i=0;i<usedBefore.size();i++)
    {
        if (usedBefore[i] && !usedAfter[i])
        {
            statistics.nRemovedNodes++;
        }
    }

    RLogger::info("Triangles: %u -> %u, removed nodes: %u, inserted nodes: %u, frozen triangles: %u\n",
                  statistics.nInputTriangles,statistics.nOutputTriangles,statistics.nRemovedNodes,statistics.nInsertedNodes,statistics.nFrozenTriangles);
    RLogger::unindent();

    return triangles;
}


void RSurfaceReconstructor::initialize(const std::vector<bool> &frozenTriangles)
{
    this->points.clear();
    this->vertexNodeIDs.clear();
    this->lockedVertices.clear();
    this->lockedEdges.clear();
    this->tris.clear();
    this->segments.clear();
    this->patches.clear();
    this->vertexSegments.clear();
    this->insertedVertices.clear();
    this->newNodes.clear();

    std::vector<int> nodeVertex(this->nodes.size(),-1);

    auto findVertex = [&](uint nodeID) -> int
    {
        if (nodeVertex[nodeID] < 0)
        {
            nodeVertex[nodeID] = int(this->points.size());
            const RNode &node = this->nodes[nodeID];
            this->points.push_back({node.getX(),node.getY(),node.getZ()});
            this->vertexNodeIDs.push_back(nodeID);
            this->lockedVertices.push_back(nodeID < this->inputLockedNodes.size() && this->inputLockedNodes[nodeID]);
        }
        return nodeVertex[nodeID];
    };

    // Nodes and edges of frozen triangles are locked.
    std::vector<bool> frozenNodes(this->nodes.size(),false);
    std::vector<std::pair<uint,uint>> frozenEdges;
    for (size_t i=0;i<this->inputTriangles.size();i++)
    {
        if (!frozenTriangles[i])
        {
            continue;
        }
        const Triangle &triangle = this->inputTriangles[i];
        for (uint k=0;k<3;k++)
        {
            frozenNodes[triangle.nodes[k]] = true;
            frozenEdges.push_back({triangle.nodes[k],triangle.nodes[(k+1)%3]});
        }
    }

    // Exact duplicates within the same surface are dropped.
    std::set<std::array<uint,4>> triangleKeys;

    this->tris.reserve(this->inputTriangles.size());
    for (size_t i=0;i<this->inputTriangles.size();i++)
    {
        if (frozenTriangles[i])
        {
            continue;
        }
        const Triangle &triangle = this->inputTriangles[i];
        if (triangle.nodes[0] == triangle.nodes[1] || triangle.nodes[1] == triangle.nodes[2] || triangle.nodes[2] == triangle.nodes[0])
        {
            // Degenerate - removed.
            continue;
        }

        std::array<uint,4> key = {triangle.nodes[0],triangle.nodes[1],triangle.nodes[2],0};
        std::sort(key.begin(),key.begin()+3);
        key[3] = triangle.surfaceID;
        if (!triangleKeys.insert(key).second)
        {
            continue;
        }

        Tri tri;
        for (uint k=0;k<3;k++)
        {
            tri.v[k] = findVertex(triangle.nodes[k]);
        }
        tri.surfaceID = triangle.surfaceID;
        tri.alive = true;
        tri.patch = -1;
        this->tris.push_back(tri);
    }

    for (size_t v=0;v<this->vertexNodeIDs.size();v++)
    {
        if (frozenNodes[this->vertexNodeIDs[v]])
        {
            this->lockedVertices[v] = true;
        }
    }

    for (const std::vector<std::pair<uint,uint>> *edges : {&this->inputLockedEdges,&frozenEdges})
    {
        for (const std::pair<uint,uint> &edge : *edges)
        {
            if (edge.first >= this->nodes.size() || edge.second >= this->nodes.size())
            {
                continue;
            }
            int a = nodeVertex[edge.first];
            int b = nodeVertex[edge.second];
            if (a >= 0 && b >= 0 && a != b)
            {
                this->lockedEdges.insert(RSurfaceReconstructor::edgeKey(a,b));
            }
        }
    }

    this->rebuildVertexTris();
}


std::vector<RSurfaceReconstructor::Triangle> RSurfaceReconstructor::run(Statistics &statistics)
{
    statistics = Statistics();

    this->collapseShortEdges(statistics);
    this->retriangulatePlanarRegions(statistics);
    this->removeSlivers(statistics);
    this->flipToDelaunay(statistics);
    this->refineMesh(statistics);

    // New vertices get node IDs following model nodes.
    this->newNodes.clear();
    for (const Tri &tri : this->tris)
    {
        if (!tri.alive)
        {
            continue;
        }
        for (int k=0;k<3;k++)
        {
            uint &nodeID = this->vertexNodeIDs[tri.v[k]];
            if (nodeID == RConstants::eod)
            {
                nodeID = uint(this->nodes.size() + this->newNodes.size());
                const Vec3 &p = this->points[tri.v[k]];
                this->newNodes.push_back(RNode(p.x,p.y,p.z));
            }
        }
    }
    statistics.nInsertedNodes = uint(this->newNodes.size());

    std::vector<Triangle> triangles;
    triangles.reserve(this->tris.size());
    for (const Tri &tri : this->tris)
    {
        if (!tri.alive)
        {
            continue;
        }
        Triangle triangle;
        triangle.nodes = {this->vertexNodeIDs[tri.v[0]],this->vertexNodeIDs[tri.v[1]],this->vertexNodeIDs[tri.v[2]]};
        triangle.surfaceID = tri.surfaceID;
        triangles.push_back(triangle);
    }
    return triangles;
}


void RSurfaceReconstructor::collapseShortEdges(Statistics &statistics)
{
    uint nCollapsed = 0;

    for (uint pass=0;pass<10;pass++)
    {
        struct ShortEdge
        {
            double length;
            int a, b;
        };
        std::vector<ShortEdge> shortEdges;
        std::unordered_set<uint64_t> edgeKeys;

        for (const Tri &tri : this->tris)
        {
            if (!tri.alive)
            {
                continue;
            }
            for (int i=0;i<3;i++)
            {
                int a = tri.v[i];
                int b = tri.v[(i+1)%3];
                double length = norm(sub(this->points[b],this->points[a]));
                if (length <= this->options.minEdgeLength && edgeKeys.insert(RSurfaceReconstructor::edgeKey(a,b)).second)
                {
                    shortEdges.push_back({length,a,b});
                }
            }
        }
        if (shortEdges.empty())
        {
            break;
        }

        std::stable_sort(shortEdges.begin(),shortEdges.end(),[](const ShortEdge &e1, const ShortEdge &e2) { return e1.length < e2.length; });

        uint nPass = 0;
        for (const ShortEdge &shortEdge : shortEdges)
        {
            if (this->vertexTris[shortEdge.a].empty() || this->vertexTris[shortEdge.b].empty())
            {
                continue;
            }
            if (this->canCollapse(shortEdge.a,shortEdge.b,true))
            {
                this->collapse(shortEdge.a,shortEdge.b);
            }
            else if (this->canCollapse(shortEdge.b,shortEdge.a,true))
            {
                this->collapse(shortEdge.b,shortEdge.a);
            }
            else
            {
                continue;
            }
            nPass++;
        }
        nCollapsed += nPass;
        if (nPass == 0)
        {
            break;
        }
    }

    statistics.nCollapsedEdges += nCollapsed;
    RLogger::info("Collapsed short edges: %u\n",nCollapsed);
}


void RSurfaceReconstructor::retriangulatePlanarRegions(Statistics &statistics)
{
    this->rebuildVertexTris();

    const int nV = int(this->points.size());
    const double distanceTolerance = this->options.distanceTolerance;

    std::vector<int> triList;
    for (int t=0;t<int(this->tris.size());t++)
    {
        if (this->tris[t].alive)
        {
            triList.push_back(t);
        }
    }
    if (triList.empty())
    {
        return;
    }

    // Edge to triangle map - first two triangles and number of triangles.
    struct EdgeTris
    {
        int t[2];
        uint count;
    };
    std::unordered_map<uint64_t,EdgeTris> edgeMap;
    edgeMap.reserve(2*triList.size());
    for (int t : triList)
    {
        for (int i=0;i<3;i++)
        {
            EdgeTris &edgeTris = edgeMap[RSurfaceReconstructor::edgeKey(this->tris[t].v[i],this->tris[t].v[(i+1)%3])];
            if (edgeTris.count < 2)
            {
                edgeTris.t[edgeTris.count] = t;
            }
            edgeTris.count++;
        }
    }

    // Triangle normals.
    std::vector<Vec3> triNormals(this->tris.size(),{0.0,0.0,0.0});
    std::vector<double> triAreas(this->tris.size(),0.0);
    std::vector<char> triReliable(this->tris.size(),0);
    for (int t : triList)
    {
        Vec3 n = this->findTriNormal(t);
        triAreas[t] = norm(n);
        triNormals[t] = normalized(n);
        triReliable[t] = this->hasReliableNormal(t);
    }

    // Grow planar regions starting from largest triangles.
    struct Region
    {
        Vec3 normal;
        Vec3 origin;
        uint surfaceID;
        std::vector<int> tris;
    };
    std::vector<Region> regions;
    std::vector<int> triRegion(this->tris.size(),-1);

    std::vector<int> order(triList);
    std::stable_sort(order.begin(),order.end(),[&](int t1, int t2) { return triAreas[t1] > triAreas[t2]; });

    for (int seed : order)
    {
        if (triRegion[seed] >= 0)
        {
            continue;
        }
        const Tri &seedTri = this->tris[seed];

        Region region;
        region.surfaceID = seedTri.surfaceID;
        region.normal = (triAreas[seed] > 0.0) ? triNormals[seed] : Vec3{0.0,0.0,1.0};
        region.origin = scale(add(add(this->points[seedTri.v[0]],this->points[seedTri.v[1]]),this->points[seedTri.v[2]]),1.0/3.0);

        int r = int(regions.size());
        triRegion[seed] = r;
        region.tris.push_back(seed);

        // Thin seed has no reliable plane - region is not grown.
        for (size_t q=0;triReliable[seed] && q<region.tris.size();q++)
        {
            int t = region.tris[q];
            for (int i=0;i<3;i++)
            {
                int a = this->tris[t].v[i];
                int b = this->tris[t].v[(i+1)%3];
                const EdgeTris &edgeTris = edgeMap[RSurfaceReconstructor::edgeKey(a,b)];
                if (edgeTris.count != 2 || this->isLockedEdge(a,b))
                {
                    continue;
                }
                int u = (edgeTris.t[0] == t) ? edgeTris.t[1] : edgeTris.t[0];
                if (triRegion[u] >= 0 || this->tris[u].surfaceID != region.surfaceID)
                {
                    continue;
                }
                bool coplanar = true;
                for (int k=0;k<3 && coplanar;k++)
                {
                    coplanar = (std::abs(dot(sub(this->points[this->tris[u].v[k]],region.origin),region.normal)) <= distanceTolerance);
                }
                if (!coplanar)
                {
                    continue;
                }
                // Thin triangles have unreliable normals - only their orientation must match.
                double c = dot(triNormals[u],region.normal);
                if (triReliable[u] ? (c < this->cosAngleTolerance) : (c < 0.0))
                {
                    continue;
                }
                triRegion[u] = r;
                region.tris.push_back(u);
            }
        }

        // Region normal (orientation) is given by area weighted normals of its triangles.
        Vec3 normalSum = {0.0,0.0,0.0};
        for (int t : region.tris)
        {
            normalSum = add(normalSum,this->findTriNormal(t));
        }
        region.normal = normalized(normalSum);

        if (region.tris.size() > 1)
        {
            statistics.nPlanarRegions++;
        }
        regions.push_back(std::move(region));
    }

    // Feature edges - region boundaries, open/non-manifold and locked edges.
    std::vector<std::vector<int>> featureNeighbors(nV);
    for (const auto &edge : edgeMap)
    {
        int a = int(edge.first >> 32);
        int b = int(edge.first & 0xffffffffu);
        const EdgeTris &edgeTris = edge.second;
        if (edgeTris.count != 2 || this->isLockedEdge(a,b) || triRegion[edgeTris.t[0]] != triRegion[edgeTris.t[1]])
        {
            featureNeighbors[a].push_back(b);
            featureNeighbors[b].push_back(a);
        }
    }
    for (std::vector<int> &neighbors : featureNeighbors)
    {
        std::sort(neighbors.begin(),neighbors.end());
    }

    std::vector<int> edgeTrisBuffer;
    auto findEdgeRegions = [&](int a, int b, std::vector<int> &edgeRegions)
    {
        this->findEdgeTris(a,b,edgeTrisBuffer);
        edgeRegions.clear();
        for (int t : edgeTrisBuffer)
        {
            if (std::find(edgeRegions.begin(),edgeRegions.end(),triRegion[t]) == edgeRegions.end())
            {
                edgeRegions.push_back(triRegion[t]);
            }
        }
        std::sort(edgeRegions.begin(),edgeRegions.end());
    };

    // Vertices which may be removed if not fixed:
    //  - interior vertices of regions,
    //  - vertices on exactly two (manifold) feature edges separating the same regions.
    std::vector<char> baseCandidates(nV,0);
    {
        std::vector<int> regions0, regions1;
        for (int v=0;v<nV;v++)
        {
            if (this->vertexTris[v].empty())
            {
                continue;
            }
            const std::vector<int> &neighbors = featureNeighbors[v];
            if (neighbors.empty())
            {
                baseCandidates[v] = 1;
            }
            else if (neighbors.size() == 2)
            {
                this->findEdgeTris(v,neighbors[0],edgeTrisBuffer);
                if (edgeTrisBuffer.size() > 2)
                {
                    continue;
                }
                this->findEdgeTris(v,neighbors[1],edgeTrisBuffer);
                if (edgeTrisBuffer.size() > 2)
                {
                    continue;
                }
                findEdgeRegions(v,neighbors[0],regions0);
                findEdgeRegions(v,neighbors[1],regions1);
                baseCandidates[v] = (regions0 == regions1);
            }
        }
    }

    std::vector<char> fixedVertices(nV,0);
    for (int v=0;v<nV;v++)
    {
        fixedVertices[v] = this->lockedVertices[v];
    }

    std::vector<std::vector<std::array<int,3>>> regionResults(regions.size());
    std::vector<char> regionRetriangulate(regions.size(),0);
    std::vector<char> regionSucceeded(regions.size(),0);
    std::vector<int> cdtIndex(nV,-1);

    uint nRounds = 0;
    while (true)
    {
        nRounds++;

        std::vector<char> removed(nV,0);
        std::vector<char> anchors(nV,0);
        for (int v=0;v<nV;v++)
        {
            bool candidate = baseCandidates[v] && !fixedVertices[v];
            anchors[v] = !featureNeighbors[v].empty() && !candidate;
            if (candidate && featureNeighbors[v].empty())
            {
                removed[v] = 1;
            }
        }

        // Simplify feature lines - chains of feature edges between anchors.
        // Region segments - end vertices and number of region triangles traversing the segment forward and backward.
        std::vector<std::vector<std::array<int,4>>> regionSegments(regions.size());
        std::unordered_set<uint64_t> visitedEdges;
        std::vector<int> chainRegions;

        auto isSegmentValid = [&](const std::vector<int> &chain, size_t s, size_t j) -> bool
        {
            const Vec3 &pa = this->points[chain[s]];
            const Vec3 &pb = this->points[chain[j]];
            Vec3 d = sub(pb,pa);
            double length = norm(d);
            if (length <= 0.0)
            {
                return false;
            }
            for (size_t k=s;k<j;k++)
            {
                Vec3 e = sub(this->points[chain[k+1]],this->points[chain[k]]);
                if (dot(e,d) < norm(e) * length * this->cosAngleTolerance)
                {
                    return false;
                }
            }
            for (size_t k=s+1;k<j;k++)
            {
                if (distanceToLine(this->points[chain[k]],pa,pb) > distanceTolerance)
                {
                    return false;
                }
            }
            return true;
        };

        auto processChain = [&](const std::vector<int> &chain)
        {
            size_t m = chain.size() - 1;
            std::vector<size_t> kept = {0};
            size_t s = 0;
            size_t j = 2;
            while (j <= m)
            {
                if (isSegmentValid(chain,s,j))
                {
                    j++;
                }
                else
                {
                    kept.push_back(j-1);
                    s = j-1;
                    j = s+2;
                }
            }
            kept.push_back(m);

            for (size_t k=1;k<kept.size();k++)
            {
                for (size_t l=kept[k-1]+1;l<kept[k];l++)
                {
                    removed[chain[l]] = 1;
                }
            }

            // Regions on both sides are the same along the whole chain.
            findEdgeRegions(chain[0],chain[1],chainRegions);
            // Number of region triangles traversing the chain forward and backward
            // (orientation is the same along the whole chain).
            for (int r : chainRegions)
            {
                int nForward = 0;
                int nBackward = 0;
                for (int t : edgeTrisBuffer)
                {
                    if (triRegion[t] == r)
                    {
                        (triTraverses(this->tris[t].v,chain[0],chain[1]) ? nForward : nBackward)++;
                    }
                }
                for (size_t k=1;k<kept.size();k++)
                {
                    regionSegments[r].push_back({chain[kept[k-1]],chain[kept[k]],nForward,nBackward});
                }
            }
        };

        auto walkChain = [&](int start, int next)
        {
            std::vector<int> chain = {start};
            int prev = start;
            int cur = next;
            while (true)
            {
                visitedEdges.insert(RSurfaceReconstructor::edgeKey(prev,cur));
                chain.push_back(cur);
                if (anchors[cur])
                {
                    break;
                }
                // Non-anchor feature vertex has exactly two feature neighbors.
                const std::vector<int> &neighbors = featureNeighbors[cur];
                int following = (neighbors[0] == prev) ? neighbors[1] : neighbors[0];
                prev = cur;
                cur = following;
            }
            processChain(chain);
        };

        for (int v=0;v<nV;v++)
        {
            if (!anchors[v])
            {
                continue;
            }
            for (int n : featureNeighbors[v])
            {
                if (visitedEdges.find(RSurfaceReconstructor::edgeKey(v,n)) == visitedEdges.end())
                {
                    walkChain(v,n);
                }
            }
        }
        // Closed loops without anchor - vertex with the sharpest turn becomes an anchor.
        for (int v=0;v<nV;v++)
        {
            if (anchors[v] || featureNeighbors[v].empty())
            {
                continue;
            }
            if (visitedEdges.find(RSurfaceReconstructor::edgeKey(v,featureNeighbors[v][0])) != visitedEdges.end())
            {
                continue;
            }
            int best = v;
            double bestTurn = 2.0;
            int prev = featureNeighbors[v][1];
            int cur = v;
            do
            {
                const std::vector<int> &neighbors = featureNeighbors[cur];
                int following = (neighbors[0] == prev) ? neighbors[1] : neighbors[0];
                // Cosine of the angle between incoming and outgoing direction.
                double turn = dot(normalized(sub(this->points[cur],this->points[prev])),normalized(sub(this->points[following],this->points[cur])));
                if (turn < bestTurn)
                {
                    bestTurn = turn;
                    best = cur;
                }
                prev = cur;
                cur = following;
            }
            while (cur != v);
            anchors[best] = 1;
            walkChain(best,featureNeighbors[best][0]);
        }

        // Triangulate regions.
        std::vector<int> failedRegions;
        for (size_t r=0;r<regions.size();r++)
        {
            const Region &region = regions[r];

            regionResults[r].clear();
            regionSucceeded[r] = 0;

            bool hasRemoved = false;
            for (int t : region.tris)
            {
                for (int k=0;k<3;k++)
                {
                    hasRemoved = hasRemoved || removed[this->tris[t].v[k]];
                }
            }
            regionRetriangulate[r] = (hasRemoved || region.tris.size() > 1);
            if (!regionRetriangulate[r])
            {
                continue;
            }

            // Points.
            std::vector<int> regionPoints;
            for (int t : region.tris)
            {
                for (int k=0;k<3;k++)
                {
                    int v = this->tris[t].v[k];
                    if (!removed[v] && cdtIndex[v] < 0)
                    {
                        cdtIndex[v] = int(regionPoints.size());
                        regionPoints.push_back(v);
                    }
                }
            }

            // Local 2D coordinate system (e1 x e2 = normal).
            const Vec3 &n = region.normal;
            Vec3 e1 = normalized((std::abs(n.x) < 0.9) ? cross(n,Vec3{1.0,0.0,0.0}) : cross(n,Vec3{0.0,1.0,0.0}));
            Vec3 e2 = cross(n,e1);

            std::vector<RConstrainedDelaunay::Point> cdtPoints;
            cdtPoints.reserve(regionPoints.size());
            double uMin = 0.0, uMax = 0.0, vMin = 0.0, vMax = 0.0;
            for (size_t i=0;i<regionPoints.size();i++)
            {
                Vec3 d = sub(this->points[regionPoints[i]],region.origin);
                RConstrainedDelaunay::Point point = {dot(d,e1),dot(d,e2)};
                if (i == 0)
                {
                    uMin = uMax = point.x;
                    vMin = vMax = point.y;
                }
                uMin = std::min(uMin,point.x);
                uMax = std::max(uMax,point.x);
                vMin = std::min(vMin,point.y);
                vMax = std::max(vMax,point.y);
                cdtPoints.push_back(point);
            }
            double size = std::max(uMax - uMin,vMax - vMin);

            // Zero region normal - region orientation can not be determined.
            bool succeeded = (regionPoints.size() >= 3 && size > 0.0 && norm(n) > 0.0);

            std::vector<std::pair<unsigned int,unsigned int>> constraints;
            double perimeter = 0.0;
            for (const std::array<int,4> &segment : regionSegments[r])
            {
                int ia = cdtIndex[segment[0]];
                int ib = cdtIndex[segment[1]];
                if (ia < 0 || ib < 0)
                {
                    succeeded = false;
                    break;
                }
                constraints.push_back({unsigned(ia),unsigned(ib)});
                perimeter += norm(sub(this->points[segment[1]],this->points[segment[0]]));
            }

            std::vector<RConstrainedDelaunay::Triangle> cdtTriangles;
            if (succeeded)
            {
                RConstrainedDelaunay cdt;
                succeeded = cdt.triangulate(cdtPoints,constraints,cdtTriangles) && !cdtTriangles.empty();
            }

            // Validate - same area, no degenerated triangles, all points used, segments used
            // exactly as many times as in the original region and all other edges exactly twice
            // (the new patch is a manifold with the same boundary - neighbors stay conforming).
            if (succeeded)
            {
                double originalArea = 0.0;
                for (int t : region.tris)
                {
                    originalArea += 0.5 * dot(this->findTriNormal(t),n);
                }

                auto directedKey = [](int a, int b) -> uint64_t
                {
                    return (uint64_t(uint32_t(a)) << 32) | uint64_t(uint32_t(b));
                };
                std::unordered_map<uint64_t,int> newEdges;
                std::vector<char> usedPoints(regionPoints.size(),0);
                double newArea = 0.0;
                for (const RConstrainedDelaunay::Triangle &cdtTriangle : cdtTriangles)
                {
                    const RConstrainedDelaunay::Point &p0 = cdtPoints[cdtTriangle[0]];
                    const RConstrainedDelaunay::Point &p1 = cdtPoints[cdtTriangle[1]];
                    const RConstrainedDelaunay::Point &p2 = cdtPoints[cdtTriangle[2]];
                    double area2 = (p1.x - p0.x) * (p2.y - p0.y) - (p1.y - p0.y) * (p2.x - p0.x);
                    int v0 = regionPoints[cdtTriangle[0]];
                    int v1 = regionPoints[cdtTriangle[1]];
                    int v2 = regionPoints[cdtTriangle[2]];
                    if (area2 <= 1.0e-12 * size * size || dot(this->findTriNormal(v0,v1,v2),n) <= 0.0)
                    {
                        succeeded = false;
                        break;
                    }
                    newArea += 0.5 * area2;
                    for (int k=0;k<3;k++)
                    {
                        usedPoints[cdtTriangle[k]] = 1;
                        newEdges[directedKey(regionPoints[cdtTriangle[k]],regionPoints[cdtTriangle[(k+1)%3]])]++;
                    }
                    regionResults[r].push_back({v0,v1,v2});
                }
                if (succeeded && std::abs(newArea - originalArea) > 1.0e-9 * std::abs(originalArea) + distanceTolerance * perimeter)
                {
                    succeeded = false;
                }
                for (size_t i=0;succeeded && i<usedPoints.size();i++)
                {
                    succeeded = usedPoints[i];
                }
                if (succeeded)
                {
                    // Segments - directed uses must match the original region.
                    std::unordered_map<uint64_t,int> expectedUses;
                    for (const std::array<int,4> &segment : regionSegments[r])
                    {
                        expectedUses[directedKey(segment[0],segment[1])] += segment[2];
                        expectedUses[directedKey(segment[1],segment[0])] += segment[3];
                    }
                    for (const auto &expectedUse : expectedUses)
                    {
                        auto iter = newEdges.find(expectedUse.first);
                        int nUses = (iter == newEdges.end()) ? 0 : iter->second;
                        if (nUses != expectedUse.second)
                        {
                            succeeded = false;
                            break;
                        }
                    }
                    // Other edges - interior, used once in each direction.
                    for (auto iter = newEdges.begin();succeeded && iter != newEdges.end();++iter)
                    {
                        // New edges must not coincide with locked edges (edges of elements outside the working mesh).
                        int a = int(iter->first >> 32);
                        int b = int(iter->first & 0xffffffffu);
                        if (this->isLockedEdge(a,b))
                        {
                            this->findEdgeTris(a,b,edgeTrisBuffer);
                            if (edgeTrisBuffer.empty())
                            {
                                succeeded = false;
                                break;
                            }
                        }
                        if (expectedUses.find(iter->first) != expectedUses.end())
                        {
                            continue;
                        }
                        uint64_t reverseKey = (iter->first << 32) | (iter->first >> 32);
                        auto reverse = newEdges.find(reverseKey);
                        succeeded = (iter->second == 1 && reverse != newEdges.end() && reverse->second == 1);
                    }
                }
            }

            for (int v : regionPoints)
            {
                cdtIndex[v] = -1;
            }

            regionSucceeded[r] = succeeded;
            if (!succeeded)
            {
                regionResults[r].clear();
                failedRegions.push_back(int(r));
            }
        }

        // Global check - new triangles of neighboring regions must not share edges more times
        // than in the original mesh (e.g. two regions cutting the same ear at a kinked feature
        // line would create a doubled triangle).
        {
            std::unordered_map<uint64_t,uint> edgeUses;
            edgeUses.reserve(edgeMap.size());
            std::unordered_map<uint64_t,std::vector<int>> edgeNewRegions;
            for (size_t r=0;r<regions.size();r++)
            {
                const bool isNew = (regionRetriangulate[r] && regionSucceeded[r]);
                auto countTriangle = [&](int v0, int v1, int v2)
                {
                    const int v[3] = {v0,v1,v2};
                    for (int k=0;k<3;k++)
                    {
                        uint64_t key = RSurfaceReconstructor::edgeKey(v[k],v[(k+1)%3]);
                        edgeUses[key]++;
                        if (isNew)
                        {
                            std::vector<int> &edgeRegions = edgeNewRegions[key];
                            if (edgeRegions.empty() || edgeRegions.back() != int(r))
                            {
                                edgeRegions.push_back(int(r));
                            }
                        }
                    }
                };
                if (isNew)
                {
                    for (const std::array<int,3> &triangle : regionResults[r])
                    {
                        countTriangle(triangle[0],triangle[1],triangle[2]);
                    }
                }
                else
                {
                    for (int t : regions[r].tris)
                    {
                        countTriangle(this->tris[t].v[0],this->tris[t].v[1],this->tris[t].v[2]);
                    }
                }
            }
            for (const auto &edgeUse : edgeUses)
            {
                auto iter = edgeMap.find(edgeUse.first);
                uint allowedUses = std::max(2u,(iter == edgeMap.end()) ? 0u : iter->second.count);
                if (edgeUse.second <= allowedUses)
                {
                    continue;
                }
                auto newIter = edgeNewRegions.find(edgeUse.first);
                if (newIter == edgeNewRegions.end())
                {
                    continue;
                }
                for (int r : newIter->second)
                {
                    if (regionSucceeded[r])
                    {
                        regionSucceeded[r] = 0;
                        regionResults[r].clear();
                        failedRegions.push_back(r);
                    }
                }
            }
        }

        // Failed regions keep original triangles - all their vertices must be preserved.
        bool newFixed = false;
        for (int r : failedRegions)
        {
            for (int t : regions[r].tris)
            {
                for (int k=0;k<3;k++)
                {
                    int v = this->tris[t].v[k];
                    if (!fixedVertices[v])
                    {
                        fixedVertices[v] = 1;
                        newFixed = true;
                    }
                }
            }
        }
        if (!newFixed)
        {
            statistics.nFailedRegions += uint(failedRegions.size());
            break;
        }
    }

    // Replace triangles of retriangulated regions.
    uint nRetriangulated = 0;
    for (size_t r=0;r<regions.size();r++)
    {
        if (!regionRetriangulate[r] || !regionSucceeded[r])
        {
            continue;
        }
        for (int t : regions[r].tris)
        {
            this->killTri(t);
        }
        for (const std::array<int,3> &triangle : regionResults[r])
        {
            this->addTri(triangle[0],triangle[1],triangle[2],regions[r].surfaceID);
        }
        nRetriangulated++;
    }
    statistics.nRetriangulatedRegions += nRetriangulated;

    RLogger::info("Planar regions: %u, retriangulated: %u, failed: %u (rounds: %u)\n",
                  statistics.nPlanarRegions,nRetriangulated,statistics.nFailedRegions,nRounds);
}


void RSurfaceReconstructor::removeSlivers(Statistics &statistics)
{
    uint nCollapsed = 0;
    uint nFlipped = 0;

    for (uint pass=0;pass<10;pass++)
    {
        uint nChanged = 0;

        for (int t=0;t<int(this->tris.size());t++)
        {
            if (!this->tris[t].alive)
            {
                continue;
            }
            int v[3] = {this->tris[t].v[0],this->tris[t].v[1],this->tris[t].v[2]};
            if (this->findMinAngle(v[0],v[1],v[2]) >= this->options.sliverAngle)
            {
                continue;
            }

            // Edge i is v[i] - v[i+1], angle i is opposite to edge i.
            double lengths[3];
            for (int i=0;i<3;i++)
            {
                lengths[i] = norm(sub(this->points[v[(i+1)%3]],this->points[v[i]]));
            }
            int shortest = 0;
            int longest = 0;
            for (int i=1;i<3;i++)
            {
                if (lengths[i] < lengths[shortest])
                {
                    shortest = i;
                }
                if (lengths[i] > lengths[longest])
                {
                    longest = i;
                }
            }
            // Cap (one angle close to 180 degrees) is better fixed by flip, needle by collapse.
            Vec3 ea = sub(this->points[v[longest]],this->points[v[(longest+2)%3]]);
            Vec3 eb = sub(this->points[v[(longest+1)%3]],this->points[v[(longest+2)%3]]);
            bool isCap = (dot(ea,eb) < -0.5 * norm(ea) * norm(eb));

            int a = v[shortest];
            int b = v[(shortest+1)%3];
            bool shortEdge = (lengths[shortest] <= this->options.minEdgeLength);

            auto tryCollapse = [&]() -> bool
            {
                if (this->canCollapse(a,b,shortEdge))
                {
                    this->collapse(a,b);
                    return true;
                }
                if (this->canCollapse(b,a,shortEdge))
                {
                    this->collapse(b,a);
                    return true;
                }
                return false;
            };

            if (isCap)
            {
                if (this->tryFlip(t,v[longest],v[(longest+1)%3],false))
                {
                    nFlipped++;
                    nChanged++;
                }
                else if (tryCollapse())
                {
                    nCollapsed++;
                    nChanged++;
                }
            }
            else
            {
                if (tryCollapse())
                {
                    nCollapsed++;
                    nChanged++;
                }
                else if (this->tryFlip(t,v[longest],v[(longest+1)%3],false))
                {
                    nFlipped++;
                    nChanged++;
                }
            }
        }

        if (nChanged == 0)
        {
            break;
        }
    }

    statistics.nCollapsedEdges += nCollapsed;
    statistics.nFlippedEdges += nFlipped;
    RLogger::info("Sliver removal: collapsed edges: %u, flipped edges: %u\n",nCollapsed,nFlipped);
}


void RSurfaceReconstructor::flipToDelaunay(Statistics &statistics)
{
    if (!this->options.delaunay)
    {
        return;
    }

    uint nFlipped = this->flipAllToDelaunay();

    statistics.nFlippedEdges += nFlipped;
    statistics.nDelaunayFlips += nFlipped;
    RLogger::info("Delaunay triangulation: flipped edges: %u\n",nFlipped);
}


uint RSurfaceReconstructor::flipAllToDelaunay()
{
    // Lawson flips - in planar regions this converges to Delaunay triangulation.
    // Number of passes is limited as on curved surfaces flips are not guaranteed to converge.
    uint nFlipped = 0;
    for (uint pass=0;pass<100;pass++)
    {
        uint nPass = 0;
        for (int t=0;t<int(this->tris.size());t++)
        {
            if (!this->tris[t].alive)
            {
                continue;
            }
            for (int i=0;i<3;i++)
            {
                if (this->tryFlip(t,this->tris[t].v[i],this->tris[t].v[(i+1)%3],true))
                {
                    nPass++;
                    break;
                }
            }
        }
        nFlipped += nPass;
        if (nPass == 0)
        {
            break;
        }
    }
    return nFlipped;
}


void RSurfaceReconstructor::refineMesh(Statistics &statistics)
{
    if (!this->options.refine)
    {
        return;
    }

    this->vertexSegments.assign(this->points.size(),{-1,-1});
    this->insertedVertices.clear();
    this->findPatches();

    // Mesh must be constrained Delaunay within patches.
    uint nFlipped = this->flipAllToDelaunay();

    // Limit of inserted vertices - protection against endless refinement.
    double area = 0.0;
    size_t nTris = 0;
    for (int t=0;t<int(this->tris.size());t++)
    {
        if (this->tris[t].alive)
        {
            area += 0.5 * norm(this->findTriNormal(t));
            nTris++;
        }
    }
    size_t maxInserted = 20 * nTris + 1000;
    if (this->options.maxEdgeLength > 0.0)
    {
        maxInserted += size_t(std::min(1.0e8,8.0 * area / (this->options.maxEdgeLength * this->options.maxEdgeLength)));
    }

    std::deque<std::pair<int,int>> segmentQueue;
    std::deque<int> triQueue;

    for (uint64_t key : this->segments)
    {
        int a = int(key >> 32);
        int b = int(key & 0xffffffffu);
        if (this->isSegmentToSplit(a,b))
        {
            segmentQueue.push_back({a,b});
        }
    }
    for (int t=0;t<int(this->tris.size());t++)
    {
        if (this->tris[t].alive)
        {
            triQueue.push_back(t);
        }
    }

    // Triangles around new vertex may be bad and their segments encroached.
    auto queueVertexStar = [&](int v)
    {
        for (int t : this->vertexTris[v])
        {
            triQueue.push_back(t);
            for (int k=0;k<3;k++)
            {
                int a = this->tris[t].v[k];
                int b = this->tris[t].v[(k+1)%3];
                if (this->isSegment(a,b) && this->isSegmentToSplit(a,b))
                {
                    segmentQueue.push_back({a,b});
                }
            }
        }
    };

    // Split edge. Split improving quality (minLength > 0) must not create edges shorter than
    // minLength and must not spoil triangles which can not be refined.
    auto splitAndLegalize = [&](int a, int b, double minLength) -> bool
    {
        Vec3 point;
        if (!this->findSegmentSplitPoint(a,b,point))
        {
            return false;
        }
        if (minLength > 0.0)
        {
            if (std::min(norm(sub(point,this->points[a])),norm(sub(point,this->points[b]))) < minLength)
            {
                return false;
            }
            std::vector<int> edgeTris;
            this->findEdgeTris(a,b,edgeTris);
            for (int u : edgeTris)
            {
                const int *uv = this->tris[u].v;
                // Bad triangle which can not be refined must not get worse and a facet of a curved
                // surface must not become bad (refinement would spread over the curved surface).
                const double minAngle = this->findMinAngle(uv[0],uv[1],uv[2]);
                double limit = -1.0;
                if (this->patches[this->tris[u].patch].curvedFacet)
                {
                    limit = std::min(minAngle,this->options.qualityAngle);
                }
                else if (minAngle < this->options.qualityAngle && !this->isBadTri(u))
                {
                    limit = minAngle;
                }
                if (limit < 0.0)
                {
                    continue;
                }
                const int c = uv[0] + uv[1] + uv[2] - a - b;
                const Vec3 *halves[2][3] = {{&this->points[a],&point,&this->points[c]},{&point,&this->points[b],&this->points[c]}};
                for (int i=0;i<2;i++)
                {
                    for (int k=0;k<3;k++)
                    {
                        Vec3 e1 = sub(*halves[i][(k+1)%3],*halves[i][k]);
                        Vec3 e2 = sub(*halves[i][(k+2)%3],*halves[i][k]);
                        if (std::atan2(norm(cross(e1,e2)),dot(e1,e2)) < limit * (1.0 - 1.0e-9))
                        {
                            return false;
                        }
                    }
                }
            }
        }
        int m = this->splitEdge(a,b,point);
        if (m < 0)
        {
            return false;
        }
        this->legalize(m);
        queueVertexStar(m);
        return true;
    };

    std::set<std::array<int,3>> failedTris;
    auto triKey = [this](int t) -> std::array<int,3>
    {
        std::array<int,3> key = {this->tris[t].v[0],this->tris[t].v[1],this->tris[t].v[2]};
        std::sort(key.begin(),key.end());
        return key;
    };

    uint nSplitEdges = 0;
    uint nCircumcenters = 0;
    bool limitReached = false;
    std::vector<std::pair<int,int>> splitEdges;
    while (true)
    {
        if (size_t(nSplitEdges) + size_t(nCircumcenters) >= maxInserted)
        {
            limitReached = true;
            break;
        }

        // Encroached segments are split first.
        if (!segmentQueue.empty())
        {
            std::pair<int,int> segment = segmentQueue.front();
            segmentQueue.pop_front();
            if (this->isSegment(segment.first,segment.second) &&
                this->isSegmentToSplit(segment.first,segment.second) &&
                splitAndLegalize(segment.first,segment.second,0.0))
            {
                nSplitEdges++;
            }
            continue;
        }

        if (triQueue.empty())
        {
            break;
        }
        int t = triQueue.front();
        triQueue.pop_front();
        if (!this->tris[t].alive || !this->isBadTri(t))
        {
            continue;
        }
        std::array<int,3> key = triKey(t);
        if (failedTris.find(key) != failedTris.end())
        {
            continue;
        }

        int newVertex = -1;
        InsertResult result = this->insertCircumcenter(t,newVertex,splitEdges);
        if (result == InsertResult::Inserted)
        {
            nCircumcenters++;
            this->legalize(newVertex);
            queueVertexStar(newVertex);
        }
        else if (result == InsertResult::SplitEdges)
        {
            // Splits needed to improve quality must not create edges shorter than the shortest
            // edge of the bad triangle (Shewchuk's terminator) - otherwise refinement cascades
            // at small angles. Splits of too long triangles are not limited.
            double minLength = 0.0;
            {
                const int *tv = this->tris[t].v;
                double shortest = norm(sub(this->points[tv[1]],this->points[tv[0]]));
                double longest = shortest;
                for (int k=1;k<3;k++)
                {
                    double length = norm(sub(this->points[tv[(k+1)%3]],this->points[tv[k]]));
                    shortest = std::min(shortest,length);
                    longest = std::max(longest,length);
                }
                if (this->options.maxEdgeLength <= 0.0 || longest <= this->options.maxEdgeLength)
                {
                    minLength = shortest * (1.0 - 1.0e-9);
                }
            }
            bool split = false;
            for (const std::pair<int,int> &edge : splitEdges)
            {
                if (splitAndLegalize(edge.first,edge.second,minLength))
                {
                    nSplitEdges++;
                    split = true;
                }
            }
            if (split)
            {
                if (this->tris[t].alive)
                {
                    triQueue.push_back(t);
                }
            }
            else
            {
                failedTris.insert(key);
            }
        }
        else
        {
            failedTris.insert(key);
        }
    }

    uint nSmoothed = this->smoothInsertedVertices();
    nFlipped += this->flipAllToDelaunay();

    statistics.nFlippedEdges += nFlipped;
    statistics.nSmoothedNodes += nSmoothed;

    if (limitReached)
    {
        RLogger::warning("Quality refinement stopped - maximum number of inserted nodes (%u) was reached.\n",uint(maxInserted));
    }
    RLogger::info("Quality refinement: patches: %u, inserted circumcenters: %u, split edges: %u, smoothed nodes: %u, unrefinable triangles: %u\n",
                  uint(this->patches.size()),nCircumcenters,nSplitEdges,nSmoothed,uint(failedTris.size()));

    this->segments.clear();
}


void RSurfaceReconstructor::findPatches()
{
    this->patches.clear();
    this->segments.clear();

    std::vector<int> order;
    std::vector<double> areas(this->tris.size(),0.0);
    for (int t=0;t<int(this->tris.size());t++)
    {
        this->tris[t].patch = -1;
        if (this->tris[t].alive)
        {
            order.push_back(t);
            areas[t] = norm(this->findTriNormal(t));
        }
    }
    std::stable_sort(order.begin(),order.end(),[&](int t1, int t2) { return areas[t1] > areas[t2]; });

    std::vector<int> edgeTris;
    for (int seed : order)
    {
        if (this->tris[seed].patch >= 0)
        {
            continue;
        }
        const int *sv = this->tris[seed].v;

        Patch patch;
        patch.origin = scale(add(add(this->points[sv[0]],this->points[sv[1]]),this->points[sv[2]]),1.0/3.0);
        patch.normal = (areas[seed] > 0.0) ? normalized(this->findTriNormal(seed)) : Vec3{0.0,0.0,1.0};
        patch.e1 = normalized((std::abs(patch.normal.x) < 0.9) ? cross(patch.normal,Vec3{1.0,0.0,0.0}) : cross(patch.normal,Vec3{0.0,1.0,0.0}));
        patch.e2 = cross(patch.normal,patch.e1);
        patch.curvedFacet = false;

        const int p = int(this->patches.size());
        this->patches.push_back(patch);
        this->tris[seed].patch = p;

        // Thin seed has no reliable plane - patch is not grown.
        std::vector<int> patchTris = {seed};
        for (size_t q=0;this->hasReliableNormal(seed) && q<patchTris.size();q++)
        {
            int t = patchTris[q];
            for (int i=0;i<3;i++)
            {
                int a = this->tris[t].v[i];
                int b = this->tris[t].v[(i+1)%3];
                this->findEdgeTris(a,b,edgeTris);
                if (edgeTris.size() != 2 || this->isLockedEdge(a,b))
                {
                    continue;
                }
                int u = (edgeTris[0] == t) ? edgeTris[1] : edgeTris[0];
                const Tri &triU = this->tris[u];
                if (triU.patch >= 0 || triU.surfaceID != this->tris[t].surfaceID || !triTraverses(triU.v,b,a))
                {
                    continue;
                }
                double c = dot(normalized(this->findTriNormal(u)),patch.normal);
                if (this->hasReliableNormal(u) ? (c < this->cosAngleTolerance) : (c <= 0.0))
                {
                    continue;
                }
                bool coplanar = true;
                for (int k=0;k<3 && coplanar;k++)
                {
                    coplanar = (std::abs(dot(sub(this->points[triU.v[k]],patch.origin),patch.normal)) <= this->options.distanceTolerance);
                }
                if (!coplanar && patchTris.size() < 64)
                {
                    // Plane of the seed doubles the noise of nearly coplanar triangles (e.g. halves of
                    // a thin planar face) - try plane fitted to the whole patch including the candidate.
                    std::vector<int> candidateTris(patchTris);
                    candidateTris.push_back(u);
                    Vec3 normalSum = {0.0,0.0,0.0};
                    Vec3 centroidSum = {0.0,0.0,0.0};
                    double areaSum = 0.0;
                    for (int s : candidateTris)
                    {
                        const int *sv = this->tris[s].v;
                        Vec3 n = this->findTriNormal(s);
                        double area = norm(n);
                        normalSum = add(normalSum,n);
                        centroidSum = add(centroidSum,scale(add(add(this->points[sv[0]],this->points[sv[1]]),this->points[sv[2]]),area / 3.0));
                        areaSum += area;
                    }
                    if (areaSum > 0.0 && norm(normalSum) > 0.0)
                    {
                        Patch fitted = patch;
                        fitted.normal = normalized(normalSum);
                        fitted.origin = scale(centroidSum,1.0 / areaSum);
                        coplanar = true;
                        for (size_t j=0;j<candidateTris.size() && coplanar;j++)
                        {
                            const int s = candidateTris[j];
                            double cs = dot(normalized(this->findTriNormal(s)),fitted.normal);
                            coplanar = this->hasReliableNormal(s) ? (cs >= this->cosAngleTolerance) : (cs > 0.0);
                            for (int k=0;k<3 && coplanar;k++)
                            {
                                coplanar = (std::abs(dot(sub(this->points[this->tris[s].v[k]],fitted.origin),fitted.normal)) <= this->options.distanceTolerance);
                            }
                        }
                        if (coplanar)
                        {
                            fitted.e1 = normalized((std::abs(fitted.normal.x) < 0.9) ? cross(fitted.normal,Vec3{1.0,0.0,0.0}) : cross(fitted.normal,Vec3{0.0,1.0,0.0}));
                            fitted.e2 = cross(fitted.normal,fitted.e1);
                            patch = fitted;
                            this->patches[p] = fitted;
                        }
                    }
                }
                if (!coplanar)
                {
                    continue;
                }
                this->tris[u].patch = p;
                patchTris.push_back(u);
            }
        }
    }

    // Segments - patch boundaries, open, non-manifold and locked edges.
    for (int t : order)
    {
        for (int i=0;i<3;i++)
        {
            int a = this->tris[t].v[i];
            int b = this->tris[t].v[(i+1)%3];
            uint64_t key = RSurfaceReconstructor::edgeKey(a,b);
            if (this->segments.find(key) != this->segments.end())
            {
                continue;
            }
            this->findEdgeTris(a,b,edgeTris);
            if (edgeTris.size() != 2 || this->isLockedEdge(a,b) || this->tris[edgeTris[0]].patch != this->tris[edgeTris[1]].patch)
            {
                this->segments.insert(key);
            }
        }
    }

    // Curved facets - single triangle patches with a smooth neighbor (dihedral angle below 20 degrees).
    std::vector<int> patchSizes(this->patches.size(),0);
    for (int t : order)
    {
        patchSizes[this->tris[t].patch]++;
    }
    const double cosSmooth = std::cos(R_DEG_TO_RAD(20.0));
    for (int t : order)
    {
        Patch &patch = this->patches[this->tris[t].patch];
        patch.curvedFacet = false;
        if (patchSizes[this->tris[t].patch] != 1 || !this->hasReliableNormal(t))
        {
            continue;
        }
        const Vec3 n = normalized(this->findTriNormal(t));
        for (int i=0;i<3 && !patch.curvedFacet;i++)
        {
            int a = this->tris[t].v[i];
            int b = this->tris[t].v[(i+1)%3];
            this->findEdgeTris(a,b,edgeTris);
            if (edgeTris.size() != 2 || this->isLockedEdge(a,b))
            {
                continue;
            }
            int u = (edgeTris[0] == t) ? edgeTris[1] : edgeTris[0];
            if (this->tris[u].surfaceID != this->tris[t].surfaceID || !triTraverses(this->tris[u].v,b,a) || !this->hasReliableNormal(u))
            {
                continue;
            }
            patch.curvedFacet = (dot(n,normalized(this->findTriNormal(u))) > cosSmooth);
        }
    }
}


bool RSurfaceReconstructor::isBadTri(int t) const
{
    const int *v = this->tris[t].v;

    // Edge i is v[i] - v[i+1].
    double lengths[3];
    double longest = 0.0;
    for (int i=0;i<3;i++)
    {
        lengths[i] = norm(sub(this->points[v[(i+1)%3]],this->points[v[i]]));
        longest = std::max(longest,lengths[i]);
    }
    if (this->options.maxEdgeLength > 0.0 && longest > this->options.maxEdgeLength)
    {
        return true;
    }
    if (this->options.qualityAngle <= 0.0 || this->patches[this->tris[t].patch].curvedFacet)
    {
        return false;
    }

    // Smallest angle (at vertex k, opposite to edge k+1).
    int kMin = 0;
    double minAngle = RConstants::pi;
    for (int k=0;k<3;k++)
    {
        Vec3 e1 = sub(this->points[v[(k+1)%3]],this->points[v[k]]);
        Vec3 e2 = sub(this->points[v[(k+2)%3]],this->points[v[k]]);
        double angle = std::atan2(norm(cross(e1,e2)),dot(e1,e2));
        if (angle < minAngle)
        {
            minAngle = angle;
            kMin = k;
        }
    }
    if (minAngle >= this->options.qualityAngle)
    {
        return false;
    }

    // New vertex would be closer than minimum edge length.
    double area2 = norm(this->findTriNormal(t));
    if (area2 <= 0.0)
    {
        return false;
    }
    double circumradius = lengths[0] * lengths[1] * lengths[2] / (2.0 * area2);
    if (circumradius < this->options.minEdgeLength)
    {
        return false;
    }

    // Small angle between two segments (input angle) can not be improved.
    int a = v[kMin];
    int b = v[(kMin+1)%3];
    int c = v[(kMin+2)%3];
    if (this->isSegment(a,b) && this->isSegment(a,c))
    {
        return false;
    }

    // Smallest angle at a patch corner whose angle is less than twice the quality angle
    // can not be split into good angles.
    {
        const int patch = this->tris[t].patch;
        bool onBoundary = false;
        double cornerAngle = 0.0;
        for (int u : this->vertexTris[a])
        {
            const int *uv = this->tris[u].v;
            if (this->tris[u].patch != patch)
            {
                onBoundary = true;
                continue;
            }
            int k = (uv[0] == a) ? 0 : ((uv[1] == a) ? 1 : 2);
            Vec3 e1 = sub(this->points[uv[(k+1)%3]],this->points[a]);
            Vec3 e2 = sub(this->points[uv[(k+2)%3]],this->points[a]);
            cornerAngle += std::atan2(norm(cross(e1,e2)),dot(e1,e2));
            onBoundary = onBoundary || this->isSegment(a,uv[(k+1)%3]) || this->isSegment(a,uv[(k+2)%3]);
        }
        if (onBoundary && cornerAngle < 2.0 * this->options.qualityAngle)
        {
            return false;
        }
    }

    // Shortest edge connects vertices on two segments sharing an end vertex at the same
    // distance from it (concentric shells) - small input angle, refinement would not end.
    const std::array<int,2> &sb = this->vertexSegments[b];
    const std::array<int,2> &sc = this->vertexSegments[c];
    if (sb[0] >= 0 && sc[0] >= 0 && !((sb[0] == sc[0] && sb[1] == sc[1]) || (sb[0] == sc[1] && sb[1] == sc[0])))
    {
        for (int z : sb)
        {
            if (z != sc[0] && z != sc[1])
            {
                continue;
            }
            double db = norm(sub(this->points[b],this->points[z]));
            double dc = norm(sub(this->points[c],this->points[z]));
            if (std::abs(db - dc) <= 1.0e-3 * std::max(db,dc))
            {
                return false;
            }
        }
    }

    return true;
}


bool RSurfaceReconstructor::isSegmentToSplit(int a, int b) const
{
    if (this->isLockedEdge(a,b))
    {
        return false;
    }
    // Encroached segments are split only when circumcenter of a bad triangle encroaches them
    // (lazy encroachment) - segments of curved surfaces made of many small patches would be
    // refined without any quality improvement otherwise.
    return (this->options.maxEdgeLength > 0.0 && norm(sub(this->points[b],this->points[a])) > this->options.maxEdgeLength);
}


bool RSurfaceReconstructor::findSegmentSplitPoint(int a, int b, Vec3 &point) const
{
    if (this->isLockedEdge(a,b))
    {
        return false;
    }
    const Vec3 &pa = this->points[a];
    const Vec3 &pb = this->points[b];
    const double length = norm(sub(pb,pa));
    if (length <= 0.0)
    {
        return false;
    }

    // Distance of split point from a.
    double split = 0.5 * length;

    // Segment adjacent to its original end vertex is split at power of two distance
    // from that vertex (concentric shells) - prevents endless refinement at small angles.
    const bool aIsEnd = (this->vertexSegments[a][0] < 0);
    const bool bIsEnd = (this->vertexSegments[b][0] < 0);
    if (this->isSegment(a,b) && aIsEnd != bIsEnd)
    {
        double distance = std::exp2(std::round(std::log2(0.5 * length)));
        while (distance > 2.0 * length / 3.0)
        {
            distance *= 0.5;
        }
        while (distance < length / 3.0)
        {
            distance *= 2.0;
        }
        split = aIsEnd ? distance : length - distance;
    }

    if (split < this->options.minEdgeLength || length - split < this->options.minEdgeLength)
    {
        return false;
    }
    point = add(pa,scale(sub(pb,pa),split / length));
    return true;
}


RSurfaceReconstructor::InsertResult RSurfaceReconstructor::insertCircumcenter(int t, int &newVertex, std::vector<std::pair<int,int>> &splitEdges)
{
    newVertex = -1;
    splitEdges.clear();

    const int patchID = this->tris[t].patch;
    if (patchID < 0)
    {
        return InsertResult::Failed;
    }

    struct Point2D
    {
        double x, y;
    };
    auto to2D = [&](int v) -> Point2D
    {
        Point2D p;
        this->projectToPatch(patchID,this->points[v],p.x,p.y);
        return p;
    };
    auto orient = [](const Point2D &a, const Point2D &b, const Point2D &c) -> double
    {
        return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    };
    // Circumcenter and squared circumradius of positively oriented triangle.
    auto circumcircle = [&](int u, Point2D &center, double &radius2) -> bool
    {
        Point2D p0 = to2D(this->tris[u].v[0]);
        Point2D p1 = to2D(this->tris[u].v[1]);
        Point2D p2 = to2D(this->tris[u].v[2]);
        double bx = p1.x - p0.x;
        double by = p1.y - p0.y;
        double cx = p2.x - p0.x;
        double cy = p2.y - p0.y;
        double d = 2.0 * (bx * cy - by * cx);
        double b2 = bx * bx + by * by;
        double c2 = cx * cx + cy * cy;
        if (d <= 1.0e-14 * std::max(b2,c2))
        {
            return false;
        }
        double ux = (cy * b2 - by * c2) / d;
        double uy = (bx * c2 - cx * b2) / d;
        center = {p0.x + ux,p0.y + uy};
        radius2 = ux * ux + uy * uy;
        return true;
    };

    Point2D c;
    double radius2 = 0.0;
    if (!circumcircle(t,c,radius2))
    {
        return InsertResult::Failed;
    }

    // Cavity - triangles of the patch whose circumcircle contains the new point.
    std::vector<int> cavity = {t};
    std::unordered_set<int> inCavity = {t};
    std::vector<int> edgeTris;
    for (size_t q=0;q<cavity.size();q++)
    {
        int s = cavity[q];
        for (int i=0;i<3;i++)
        {
            int a = this->tris[s].v[i];
            int b = this->tris[s].v[(i+1)%3];
            if (this->isSegment(a,b))
            {
                continue;
            }
            this->findEdgeTris(a,b,edgeTris);
            if (edgeTris.size() != 2)
            {
                continue;
            }
            int u = (edgeTris[0] == s) ? edgeTris[1] : edgeTris[0];
            if (inCavity.find(u) != inCavity.end() || this->tris[u].patch != patchID)
            {
                continue;
            }
            Point2D center;
            double r2;
            if (!circumcircle(u,center,r2))
            {
                continue;
            }
            double dx = c.x - center.x;
            double dy = c.y - center.y;
            if (dx * dx + dy * dy < r2 * (1.0 - 1.0e-12))
            {
                cavity.push_back(u);
                inCavity.insert(u);
                if (cavity.size() > 10000)
                {
                    return InsertResult::Failed;
                }
            }
        }
    }

    // Cavity boundary - directed edges not shared by two cavity triangles.
    std::unordered_map<uint64_t,int> edgeCount;
    for (int s : cavity)
    {
        for (int i=0;i<3;i++)
        {
            edgeCount[RSurfaceReconstructor::edgeKey(this->tris[s].v[i],this->tris[s].v[(i+1)%3])]++;
        }
    }
    std::vector<std::pair<int,int>> boundary;
    for (int s : cavity)
    {
        for (int i=0;i<3;i++)
        {
            int a = this->tris[s].v[i];
            int b = this->tris[s].v[(i+1)%3];
            if (edgeCount[RSurfaceReconstructor::edgeKey(a,b)] == 1)
            {
                boundary.push_back({a,b});
            }
            else if (this->isSegment(a,b))
            {
                // Segment inside the cavity would be removed.
                Vec3 point;
                if (this->findSegmentSplitPoint(a,b,point))
                {
                    splitEdges.push_back({a,b});
                    return InsertResult::SplitEdges;
                }
                return InsertResult::Failed;
            }
        }
    }

    auto isEncroached = [&](int a, int b) -> bool
    {
        Point2D pa = to2D(a);
        Point2D pb = to2D(b);
        double mx = 0.5 * (pa.x + pb.x) - c.x;
        double my = 0.5 * (pa.y + pb.y) - c.y;
        double half2 = 0.25 * ((pb.x - pa.x) * (pb.x - pa.x) + (pb.y - pa.y) * (pb.y - pa.y));
        return (mx * mx + my * my < half2 * (1.0 - 1.0e-12));
    };
    auto isSplittable = [&](int a, int b) -> bool
    {
        Vec3 point;
        return this->findSegmentSplitPoint(a,b,point);
    };

    // Triangle containing the new point.
    int container = -1;
    const double eps = 1.0e-10 * radius2;
    for (int s : cavity)
    {
        Point2D p0 = to2D(this->tris[s].v[0]);
        Point2D p1 = to2D(this->tris[s].v[1]);
        Point2D p2 = to2D(this->tris[s].v[2]);
        if (orient(p0,p1,c) >= -eps && orient(p1,p2,c) >= -eps && orient(p2,p0,c) >= -eps)
        {
            container = s;
            break;
        }
    }

    if (container < 0)
    {
        // New point lies outside the cavity - beyond a segment or curvature limit of the patch.
        for (const std::pair<int,int> &edge : boundary)
        {
            if (this->isSegment(edge.first,edge.second) && isEncroached(edge.first,edge.second) && isSplittable(edge.first,edge.second))
            {
                splitEdges.push_back(edge);
            }
        }
        if (!splitEdges.empty())
        {
            return InsertResult::SplitEdges;
        }
        // Split boundary edge crossed by the line from triangle centroid to the new point.
        const int *tv = this->tris[t].v;
        Point2D p0 = to2D(tv[0]);
        Point2D p1 = to2D(tv[1]);
        Point2D p2 = to2D(tv[2]);
        Point2D g = {(p0.x + p1.x + p2.x) / 3.0,(p0.y + p1.y + p2.y) / 3.0};
        double bestParameter = 2.0;
        std::pair<int,int> bestEdge = {-1,-1};
        for (const std::pair<int,int> &edge : boundary)
        {
            Point2D pa = to2D(edge.first);
            Point2D pb = to2D(edge.second);
            double oa = orient(g,c,pa);
            double ob = orient(g,c,pb);
            double og = orient(pa,pb,g);
            double oc = orient(pa,pb,c);
            if (oa * ob > 0.0 || og * oc > 0.0 || og == oc)
            {
                continue;
            }
            double parameter = og / (og - oc);
            if (parameter < bestParameter)
            {
                bestParameter = parameter;
                bestEdge = edge;
            }
        }
        if (bestEdge.first >= 0 && isSplittable(bestEdge.first,bestEdge.second))
        {
            splitEdges.push_back(bestEdge);
            return InsertResult::SplitEdges;
        }
        return InsertResult::Failed;
    }

    // Encroached segments are split instead of inserting the new point.
    for (const std::pair<int,int> &edge : boundary)
    {
        if (this->isSegment(edge.first,edge.second) && isEncroached(edge.first,edge.second) && isSplittable(edge.first,edge.second))
        {
            splitEdges.push_back(edge);
        }
    }
    if (!splitEdges.empty())
    {
        return InsertResult::SplitEdges;
    }

    // New point lies on the plane of the containing triangle (on the existing surface).
    const Patch &patch = this->patches[patchID];
    Vec3 point = add(patch.origin,add(scale(patch.e1,c.x),scale(patch.e2,c.y)));
    Vec3 containerNormal = normalized(this->findTriNormal(container));
    if (norm(containerNormal) > 0.0)
    {
        point = sub(point,scale(containerNormal,dot(sub(point,this->points[this->tris[container].v[0]]),containerNormal)));
    }

    // Validate new triangles - positive orientation, no short edges and the fan covers
    // the full angle around the new point exactly once (cavity is star shaped).
    double angleSum = 0.0;
    for (const std::pair<int,int> &edge : boundary)
    {
        const Vec3 &pa = this->points[edge.first];
        const Vec3 &pb = this->points[edge.second];
        if (norm(sub(pa,point)) < this->options.minEdgeLength || norm(sub(pb,point)) < this->options.minEdgeLength)
        {
            return InsertResult::Failed;
        }
        Point2D qa = to2D(edge.first);
        Point2D qb = to2D(edge.second);
        double o = orient(qa,qb,c);
        if (o <= 1.0e-12 * radius2)
        {
            return InsertResult::Failed;
        }
        if (dot(cross(sub(pb,pa),sub(point,pa)),patch.normal) <= 0.0)
        {
            return InsertResult::Failed;
        }
        double ax = qa.x - c.x;
        double ay = qa.y - c.y;
        double bx = qb.x - c.x;
        double by = qb.y - c.y;
        angleSum += std::atan2(ax * by - ay * bx,ax * bx + ay * by);
    }
    if (std::abs(angleSum - 2.0 * RConstants::pi) > 1.0e-6)
    {
        return InsertResult::Failed;
    }

    const uint surfaceID = this->tris[t].surfaceID;
    newVertex = this->addVertex(point);
    for (int s : cavity)
    {
        this->killTri(s);
    }
    for (const std::pair<int,int> &edge : boundary)
    {
        int u = this->addTri(edge.first,edge.second,newVertex,surfaceID);
        this->tris[u].patch = patchID;
    }
    this->insertedVertices.push_back(newVertex);

    return InsertResult::Inserted;
}


int RSurfaceReconstructor::splitEdge(int a, int b, const Vec3 &point)
{
    std::vector<int> edgeTris;
    this->findEdgeTris(a,b,edgeTris);
    if (edgeTris.empty() || this->isLockedEdge(a,b))
    {
        return -1;
    }

    int m = this->addVertex(point);

    if (this->isSegment(a,b))
    {
        // Remember original end vertices of the segment.
        std::array<int,2> ends = {a,b};
        if (this->vertexSegments[a][0] >= 0)
        {
            ends = this->vertexSegments[a];
        }
        else if (this->vertexSegments[b][0] >= 0)
        {
            ends = this->vertexSegments[b];
        }
        this->vertexSegments[m] = ends;
        this->segments.erase(RSurfaceReconstructor::edgeKey(a,b));
        this->segments.insert(RSurfaceReconstructor::edgeKey(a,m));
        this->segments.insert(RSurfaceReconstructor::edgeKey(m,b));
    }
    else
    {
        this->insertedVertices.push_back(m);
    }

    for (int t : edgeTris)
    {
        const Tri tri = this->tris[t];
        // Directed edge x->y of the triangle and its opposite vertex.
        int x = -1, y = -1, c = -1;
        for (int k=0;k<3;k++)
        {
            if ((tri.v[k] == a && tri.v[(k+1)%3] == b) || (tri.v[k] == b && tri.v[(k+1)%3] == a))
            {
                x = tri.v[k];
                y = tri.v[(k+1)%3];
                c = tri.v[(k+2)%3];
            }
        }
        this->killTri(t);
        int t1 = this->addTri(x,m,c,tri.surfaceID);
        int t2 = this->addTri(m,y,c,tri.surfaceID);
        this->tris[t1].patch = tri.patch;
        this->tris[t2].patch = tri.patch;
    }

    return m;
}


void RSurfaceReconstructor::legalize(int v)
{
    std::vector<int> stack = this->vertexTris[v];
    for (uint guard=0;!stack.empty() && guard<1000000;guard++)
    {
        int t = stack.back();
        stack.pop_back();
        const Tri &tri = this->tris[t];
        if (!tri.alive || !triHas(tri.v,v))
        {
            continue;
        }
        int k = (tri.v[0] == v) ? 0 : ((tri.v[1] == v) ? 1 : 2);
        int x = tri.v[(k+1)%3];
        int y = tri.v[(k+2)%3];
        // Apex of t is v, after the flip both triangles contain v (the other one was added last).
        if (this->tryFlip(t,x,y,true))
        {
            stack.push_back(t);
            stack.push_back(this->vertexTris[v].back());
        }
    }
}


uint RSurfaceReconstructor::smoothInsertedVertices()
{
    std::vector<char> moved(this->points.size(),0);
    std::vector<int> neighbors;

    for (uint iteration=0;iteration<5;iteration++)
    {
        uint nMoved = 0;
        for (int v : this->insertedVertices)
        {
            const std::vector<int> &star = this->vertexTris[v];
            if (star.empty() || this->vertexSegments[v][0] >= 0)
            {
                continue;
            }
            const int patchID = this->tris[star[0]].patch;
            bool valid = (patchID >= 0);
            for (int t : star)
            {
                valid = valid && (this->tris[t].patch == patchID);
            }
            this->findVertexNeighbors(v,neighbors);
            for (int n : neighbors)
            {
                valid = valid && !this->isSegment(v,n);
            }
            if (!valid || neighbors.empty())
            {
                continue;
            }
            const Patch &patch = this->patches[patchID];

            // Laplacian - average of neighbors within the patch plane.
            double su = 0.0;
            double sv = 0.0;
            for (int n : neighbors)
            {
                double u, w;
                this->projectToPatch(patchID,this->points[n],u,w);
                su += u;
                sv += w;
            }
            su /= double(neighbors.size());
            sv /= double(neighbors.size());
            Vec3 target = add(patch.origin,add(scale(patch.e1,su),scale(patch.e2,sv)));

            double oldMinAngle = RConstants::pi;
            for (int t : star)
            {
                oldMinAngle = std::min(oldMinAngle,this->findMinAngle(this->tris[t].v[0],this->tris[t].v[1],this->tris[t].v[2]));
            }

            const Vec3 original = this->points[v];
            this->points[v] = target;
            bool accepted = true;
            double newMinAngle = RConstants::pi;
            for (int t : star)
            {
                const int *tv = this->tris[t].v;
                if (dot(this->findTriNormal(t),patch.normal) <= 0.0 || !this->hasReliableNormal(tv[0],tv[1],tv[2]))
                {
                    accepted = false;
                    break;
                }
                newMinAngle = std::min(newMinAngle,this->findMinAngle(tv[0],tv[1],tv[2]));
            }
            if (!accepted || newMinAngle <= oldMinAngle || norm(sub(target,original)) <= 1.0e-12 * this->options.minEdgeLength)
            {
                this->points[v] = original;
                continue;
            }
            moved[v] = 1;
            nMoved++;
        }
        this->flipAllToDelaunay();
        if (nMoved == 0)
        {
            break;
        }
    }

    return uint(std::count(moved.begin(),moved.end(),1));
}


int RSurfaceReconstructor::addVertex(const Vec3 &point)
{
    int v = int(this->points.size());
    this->points.push_back(point);
    this->vertexNodeIDs.push_back(RConstants::eod);
    this->lockedVertices.push_back(false);
    this->vertexTris.emplace_back();
    this->vertexSegments.push_back({-1,-1});
    return v;
}


void RSurfaceReconstructor::projectToPatch(int patch, const Vec3 &point, double &u, double &v) const
{
    const Patch &p = this->patches[patch];
    Vec3 d = sub(point,p.origin);
    u = dot(d,p.e1);
    v = dot(d,p.e2);
}


bool RSurfaceReconstructor::isSegment(int a, int b) const
{
    return (!this->segments.empty() && this->segments.find(RSurfaceReconstructor::edgeKey(a,b)) != this->segments.end());
}


void RSurfaceReconstructor::rebuildVertexTris()
{
    this->vertexTris.assign(this->points.size(),std::vector<int>());
    for (int t=0;t<int(this->tris.size());t++)
    {
        if (!this->tris[t].alive)
        {
            continue;
        }
        for (int k=0;k<3;k++)
        {
            this->vertexTris[this->tris[t].v[k]].push_back(t);
        }
    }
}


void RSurfaceReconstructor::killTri(int t)
{
    Tri &tri = this->tris[t];
    if (!tri.alive)
    {
        return;
    }
    tri.alive = false;
    for (int k=0;k<3;k++)
    {
        eraseValue(this->vertexTris[tri.v[k]],t);
    }
}


int RSurfaceReconstructor::addTri(int v0, int v1, int v2, uint surfaceID)
{
    Tri tri;
    tri.v[0] = v0;
    tri.v[1] = v1;
    tri.v[2] = v2;
    tri.surfaceID = surfaceID;
    tri.alive = true;
    tri.patch = -1;
    int t = int(this->tris.size());
    this->tris.push_back(tri);
    for (int k=0;k<3;k++)
    {
        this->vertexTris[tri.v[k]].push_back(t);
    }
    return t;
}


void RSurfaceReconstructor::findEdgeTris(int a, int b, std::vector<int> &edgeTris) const
{
    edgeTris.clear();
    for (int t : this->vertexTris[a])
    {
        if (triHas(this->tris[t].v,b))
        {
            edgeTris.push_back(t);
        }
    }
}


void RSurfaceReconstructor::findVertexNeighbors(int a, std::vector<int> &neighbors) const
{
    neighbors.clear();
    for (int t : this->vertexTris[a])
    {
        for (int k=0;k<3;k++)
        {
            int v = this->tris[t].v[k];
            if (v != a && std::find(neighbors.begin(),neighbors.end(),v) == neighbors.end())
            {
                neighbors.push_back(v);
            }
        }
    }
}


bool RSurfaceReconstructor::isLockedEdge(int a, int b) const
{
    return (!this->lockedEdges.empty() && this->lockedEdges.find(RSurfaceReconstructor::edgeKey(a,b)) != this->lockedEdges.end());
}


bool RSurfaceReconstructor::isHardFeatureEdge(int a, int b) const
{
    std::vector<int> edgeTris;
    this->findEdgeTris(a,b,edgeTris);
    return (edgeTris.size() != 2 || this->isLockedEdge(a,b) || this->tris[edgeTris[0]].surfaceID != this->tris[edgeTris[1]].surfaceID);
}


bool RSurfaceReconstructor::isFeatureEdge(int a, int b) const
{
    std::vector<int> edgeTris;
    this->findEdgeTris(a,b,edgeTris);
    if (edgeTris.size() != 2 || this->isLockedEdge(a,b))
    {
        return true;
    }
    const Tri &t0 = this->tris[edgeTris[0]];
    const Tri &t1 = this->tris[edgeTris[1]];
    if (t0.surfaceID != t1.surfaceID)
    {
        return true;
    }
    if (!this->hasReliableNormal(edgeTris[0]) || !this->hasReliableNormal(edgeTris[1]))
    {
        return false;
    }
    Vec3 n0 = normalized(this->findTriNormal(edgeTris[0]));
    Vec3 n1 = normalized(this->findTriNormal(edgeTris[1]));
    // Consistently oriented neighbors traverse shared edge in opposite directions.
    if (triTraverses(t0.v,a,b) == triTraverses(t1.v,a,b))
    {
        n1 = scale(n1,-1.0);
    }
    return (dot(n0,n1) < this->cosAngleTolerance);
}


Vec3 RSurfaceReconstructor::findTriNormal(int t) const
{
    return this->findTriNormal(this->tris[t].v[0],this->tris[t].v[1],this->tris[t].v[2]);
}


Vec3 RSurfaceReconstructor::findTriNormal(int v0, int v1, int v2) const
{
    const Vec3 &p0 = this->points[v0];
    return cross(sub(this->points[v1],p0),sub(this->points[v2],p0));
}


bool RSurfaceReconstructor::hasReliableNormal(int t) const
{
    return this->hasReliableNormal(this->tris[t].v[0],this->tris[t].v[1],this->tris[t].v[2]);
}


bool RSurfaceReconstructor::hasReliableNormal(int v0, int v1, int v2) const
{
    const int v[3] = {v0,v1,v2};
    double longest = 0.0;
    for (int k=0;k<3;k++)
    {
        longest = std::max(longest,norm(sub(this->points[v[(k+1)%3]],this->points[v[k]])));
    }
    if (longest <= 0.0)
    {
        return false;
    }
    // Error of the normal is roughly given by distance tolerance over triangle height.
    double height = norm(this->findTriNormal(v0,v1,v2)) / longest;
    return (height * this->sinAngleTolerance > this->options.distanceTolerance);
}


double RSurfaceReconstructor::findMinAngle(int v0, int v1, int v2) const
{
    const int v[3] = {v0,v1,v2};
    double minAngle = RConstants::pi;
    for (int k=0;k<3;k++)
    {
        const Vec3 &p = this->points[v[k]];
        Vec3 e1 = sub(this->points[v[(k+1)%3]],p);
        Vec3 e2 = sub(this->points[v[(k+2)%3]],p);
        if (norm(e1) <= 0.0 || norm(e2) <= 0.0)
        {
            return 0.0;
        }
        minAngle = std::min(minAngle,std::atan2(norm(cross(e1,e2)),dot(e1,e2)));
    }
    return minAngle;
}


bool RSurfaceReconstructor::canCollapse(int a, int b, bool shortEdge) const
{
    if (a == b || this->lockedVertices[a])
    {
        return false;
    }

    std::vector<int> edgeTris;
    this->findEdgeTris(a,b,edgeTris);
    if (edgeTris.empty())
    {
        return false;
    }

    const Vec3 &pa = this->points[a];
    const Vec3 &pb = this->points[b];

    std::vector<int> neighborsA;
    this->findVertexNeighbors(a,neighborsA);

    if (shortEdge)
    {
        // Short edge may be collapsed across dihedral features (moving a corner by at most
        // minimum edge length), but boundaries, surface interfaces and locked edges must not
        // be pinched: if both vertices lie on such lines the edge itself must be one of them.
        if (!this->isHardFeatureEdge(a,b))
        {
            bool aOnHard = false;
            for (int c : neighborsA)
            {
                if (this->isHardFeatureEdge(a,c))
                {
                    aOnHard = true;
                    break;
                }
            }
            if (aOnHard)
            {
                std::vector<int> neighborsB;
                this->findVertexNeighbors(b,neighborsB);
                for (int c : neighborsB)
                {
                    if (this->isHardFeatureEdge(b,c))
                    {
                        return false;
                    }
                }
            }
        }
    }

    // Feature vertex may only slide along its (straight) feature line.
    std::vector<int> featureNeighbors;
    for (int c : neighborsA)
    {
        if (shortEdge ? false : this->isFeatureEdge(a,c))
        {
            featureNeighbors.push_back(c);
        }
    }
    if (!featureNeighbors.empty())
    {
        if (featureNeighbors.size() != 2)
        {
            return false;
        }
        int other = -1;
        if (featureNeighbors[0] == b)
        {
            other = featureNeighbors[1];
        }
        else if (featureNeighbors[1] == b)
        {
            other = featureNeighbors[0];
        }
        else
        {
            return false;
        }
        if (!shortEdge)
        {
            const Vec3 &po = this->points[other];
            Vec3 d1 = sub(pa,po);
            Vec3 d2 = sub(pb,pa);
            if (dot(d1,d2) < norm(d1) * norm(d2) * this->cosAngleTolerance)
            {
                return false;
            }
            if (distanceToLine(pa,po,pb) > this->options.distanceTolerance)
            {
                return false;
            }
        }
    }

    // Link condition - common neighbors must be opposite vertices of triangles sharing the edge.
    std::vector<int> neighborsB;
    this->findVertexNeighbors(b,neighborsB);
    for (int c : neighborsA)
    {
        if (c == b || std::find(neighborsB.begin(),neighborsB.end(),c) == neighborsB.end())
        {
            continue;
        }
        bool isOpposite = false;
        for (int t : edgeTris)
        {
            isOpposite = isOpposite || triHas(this->tris[t].v,c);
        }
        if (!isOpposite)
        {
            return false;
        }
    }
    // New edges must not coincide with locked edges (edges of elements outside the working mesh).
    for (int c : neighborsA)
    {
        if (c != b && std::find(neighborsB.begin(),neighborsB.end(),c) == neighborsB.end() && this->isLockedEdge(b,c))
        {
            return false;
        }
    }
    // Edge part of the link condition - remapped triangle must not duplicate existing one.
    for (int t : this->vertexTris[a])
    {
        const Tri &tri = this->tris[t];
        if (triHas(tri.v,b))
        {
            continue;
        }
        int c = -1;
        int d = -1;
        for (int k=0;k<3;k++)
        {
            if (tri.v[k] != a)
            {
                ((c < 0) ? c : d) = tri.v[k];
            }
        }
        for (int u : this->vertexTris[b])
        {
            if (triHas(this->tris[u].v,c) && triHas(this->tris[u].v,d))
            {
                return false;
            }
        }
    }

    // Geometry - no folded triangles, deviation from original surface within tolerance
    // and (unless collapsing short edge) quality must not drop below the worst current one.
    double minAngleOld = RConstants::pi;
    double minAngleNew = RConstants::pi;
    for (int t : this->vertexTris[a])
    {
        const Tri &tri = this->tris[t];
        minAngleOld = std::min(minAngleOld,this->findMinAngle(tri.v[0],tri.v[1],tri.v[2]));
        if (triHas(tri.v,b))
        {
            continue;
        }
        int v[3] = {tri.v[0],tri.v[1],tri.v[2]};
        for (int k=0;k<3;k++)
        {
            if (v[k] == a)
            {
                v[k] = b;
            }
        }
        Vec3 nOld = this->findTriNormal(t);
        Vec3 nNew = this->findTriNormal(v[0],v[1],v[2]);
        double lOld = norm(nOld);
        if (lOld <= 0.0)
        {
            continue;
        }
        // Inverted triangle is accepted only if both old and new one are thin (unreliable normals).
        if (dot(nNew,nOld) <= 0.0 && (this->hasReliableNormal(t) || this->hasReliableNormal(v[0],v[1],v[2])))
        {
            return false;
        }
        if (!shortEdge)
        {
            if (std::abs(dot(sub(pb,pa),nOld)) / lOld > this->options.distanceTolerance)
            {
                return false;
            }
            minAngleNew = std::min(minAngleNew,this->findMinAngle(v[0],v[1],v[2]));
        }
    }
    if (!shortEdge && minAngleNew <= minAngleOld)
    {
        return false;
    }

    return true;
}


void RSurfaceReconstructor::collapse(int a, int b)
{
    std::vector<int> aTris = this->vertexTris[a];
    for (int t : aTris)
    {
        Tri &tri = this->tris[t];
        if (triHas(tri.v,b))
        {
            this->killTri(t);
            continue;
        }
        for (int k=0;k<3;k++)
        {
            if (tri.v[k] == a)
            {
                tri.v[k] = b;
            }
        }
        this->vertexTris[b].push_back(t);
    }
    this->vertexTris[a].clear();
}


bool RSurfaceReconstructor::tryFlip(int t, int a, int b, bool delaunay)
{
    std::vector<int> edgeTris;
    this->findEdgeTris(a,b,edgeTris);
    if (edgeTris.size() != 2 || this->isLockedEdge(a,b) || this->isSegment(a,b))
    {
        return false;
    }
    int u = (edgeTris[0] == t) ? edgeTris[1] : edgeTris[0];
    if (this->tris[t].surfaceID != this->tris[u].surfaceID)
    {
        return false;
    }

    // t = [p,q,r], u = [q,p,s] (consistent orientation required).
    int p = a;
    int q = b;
    if (!triTraverses(this->tris[t].v,p,q))
    {
        std::swap(p,q);
    }
    if (!triTraverses(this->tris[u].v,q,p))
    {
        return false;
    }
    int r = -1;
    int s = -1;
    for (int k=0;k<3;k++)
    {
        if (this->tris[t].v[k] != p && this->tris[t].v[k] != q)
        {
            r = this->tris[t].v[k];
        }
        if (this->tris[u].v[k] != p && this->tris[u].v[k] != q)
        {
            s = this->tris[u].v[k];
        }
    }
    if (r < 0 || s < 0 || r == s || this->isLockedEdge(r,s))
    {
        return false;
    }
    std::vector<int> neighborsR;
    this->findVertexNeighbors(r,neighborsR);
    if (std::find(neighborsR.begin(),neighborsR.end(),s) != neighborsR.end())
    {
        return false;
    }

    // Deviation - apex of smaller triangle from plane of larger one.
    Vec3 nt = this->findTriNormal(p,q,r);
    Vec3 nu = this->findTriNormal(q,p,s);
    double at = norm(nt);
    double au = norm(nu);
    if (at <= 0.0 && au <= 0.0)
    {
        return false;
    }
    double deviation = (at >= au) ? std::abs(dot(sub(this->points[s],this->points[p]),nt)) / at
                                  : std::abs(dot(sub(this->points[r],this->points[p]),nu)) / au;
    if (deviation > this->options.distanceTolerance)
    {
        return false;
    }

    // New triangles [p,s,r] and [s,q,r] must keep orientation.
    Vec3 reference = add(nt,nu);
    if (dot(this->findTriNormal(p,s,r),reference) <= 0.0 || dot(this->findTriNormal(s,q,r),reference) <= 0.0)
    {
        return false;
    }
    if (delaunay)
    {
        // Delaunay criterion - sum of angles opposite to the edge exceeds 180 degrees.
        auto findAngle = [this](int apex, int v1, int v2) -> double
        {
            Vec3 e1 = sub(this->points[v1],this->points[apex]);
            Vec3 e2 = sub(this->points[v2],this->points[apex]);
            return std::atan2(norm(cross(e1,e2)),dot(e1,e2));
        };
        if (findAngle(r,p,q) + findAngle(s,p,q) <= RConstants::pi * (1.0 + 1.0e-10))
        {
            return false;
        }
    }
    else
    {
        // Quality must improve.
        double minAngleOld = std::min(this->findMinAngle(p,q,r),this->findMinAngle(q,p,s));
        double minAngleNew = std::min(this->findMinAngle(p,s,r),this->findMinAngle(s,q,r));
        if (minAngleNew <= minAngleOld)
        {
            return false;
        }
    }

    Tri &triT = this->tris[t];
    Tri &triU = this->tris[u];
    triT.v[0] = p; triT.v[1] = s; triT.v[2] = r;
    triU.v[0] = s; triU.v[1] = q; triU.v[2] = r;

    eraseValue(this->vertexTris[p],u);
    eraseValue(this->vertexTris[q],t);
    this->vertexTris[r].push_back(u);
    this->vertexTris[s].push_back(t);

    return true;
}


uint64_t RSurfaceReconstructor::edgeKey(int a, int b)
{
    if (a > b)
    {
        std::swap(a,b);
    }
    return (uint64_t(uint32_t(a)) << 32) | uint64_t(uint32_t(b));
}
