#include <algorithm>
#include <cmath>
#include <deque>

#include "rml_constrained_delaunay.h"

RConstrainedDelaunay::RConstrainedDelaunay()
    : lastTri(0)
{

}

bool RConstrainedDelaunay::triangulate(const std::vector<Point> &inputPoints,
                                       const std::vector<std::pair<unsigned int, unsigned int> > &constraints,
                                       std::vector<Triangle> &triangles)
{
    triangles.clear();
    this->points.clear();
    this->tris.clear();
    this->vertexTri.clear();
    this->lastTri = 0;

    int nPoints = int(inputPoints.size());
    if (nPoints < 3)
    {
        return false;
    }

    // Normalize coordinates to unit box (uniform scale keeps angles).
    double xMin = inputPoints[0].x, xMax = xMin, yMin = inputPoints[0].y, yMax = yMin;
    for (const Point &point : inputPoints)
    {
        if (!std::isfinite(point.x) || !std::isfinite(point.y))
        {
            return false;
        }
        xMin = std::min(xMin,point.x);
        xMax = std::max(xMax,point.x);
        yMin = std::min(yMin,point.y);
        yMax = std::max(yMax,point.y);
    }
    double size = std::max(xMax - xMin, yMax - yMin);
    if (size <= 0.0)
    {
        return false;
    }

    this->points.reserve(nPoints + 3);
    for (const Point &point : inputPoints)
    {
        this->points.push_back({(point.x - xMin) / size, (point.y - yMin) / size});
    }
    int s0 = nPoints, s1 = nPoints + 1, s2 = nPoints + 2;
    this->points.push_back({-10.0,-10.0});
    this->points.push_back({ 20.0,-10.0});
    this->points.push_back({-10.0, 20.0});

    this->vertexTri.assign(nPoints + 3, -1);
    this->tris.reserve(2 * nPoints + 8);
    this->tris.push_back(Tri());
    this->setTri(0, s0, s1, s2, -1, -1, -1, false, false, false);

    // Constraint end points are inserted first, interior points after them.
    std::vector<bool> isConstrained(nPoints, false);
    for (const auto &constraint : constraints)
    {
        if (int(constraint.first) >= nPoints || int(constraint.second) >= nPoints)
        {
            return false;
        }
        isConstrained[constraint.first] = true;
        isConstrained[constraint.second] = true;
    }

    std::vector<int> merged(nPoints);
    for (int pass=0;pass<2;pass++)
    {
        for (int i=0;i<nPoints;i++)
        {
            if (isConstrained[i] == (pass == 0))
            {
                merged[i] = this->insertPoint(i);
                if (merged[i] < 0)
                {
                    return false;
                }
            }
        }
    }

    for (const auto &constraint : constraints)
    {
        int a = merged[constraint.first];
        int b = merged[constraint.second];
        if (a != b && !this->insertConstraint(a, b))
        {
            return false;
        }
    }

    // Classify triangles - region depth increases when crossing constrained edge.
    int nTris = int(this->tris.size());
    std::vector<int> depth(nTris, -1);
    std::deque<int> queue;
    for (int t=0;t<nTris;t++)
    {
        const Tri &tri = this->tris[t];
        if (tri.v[0] >= nPoints || tri.v[1] >= nPoints || tri.v[2] >= nPoints)
        {
            depth[t] = 0;
            queue.push_back(t);
        }
    }
    while (!queue.empty())
    {
        int t = queue.front();
        queue.pop_front();
        for (int i=0;i<3;i++)
        {
            int u = this->tris[t].n[i];
            if (u < 0)
            {
                continue;
            }
            int d = depth[t] + (this->tris[t].c[i] ? 1 : 0);
            if (depth[u] < 0 || d < depth[u])
            {
                depth[u] = d;
                if (this->tris[t].c[i])
                {
                    queue.push_back(u);
                }
                else
                {
                    queue.push_front(u);
                }
            }
        }
    }

    // Map merged points back to original indexes.
    triangles.reserve(nTris);
    for (int t=0;t<nTris;t++)
    {
        const Tri &tri = this->tris[t];
        if (depth[t] % 2 == 1 && tri.v[0] < nPoints && tri.v[1] < nPoints && tri.v[2] < nPoints)
        {
            triangles.push_back({unsigned(tri.v[0]),unsigned(tri.v[1]),unsigned(tri.v[2])});
        }
    }

    return true;
}

double RConstrainedDelaunay::orient(int a, int b, int c) const
{
    const Point &pa = this->points[a];
    const Point &pb = this->points[b];
    const Point &pc = this->points[c];
    return (pb.x - pa.x) * (pc.y - pa.y) - (pb.y - pa.y) * (pc.x - pa.x);
}

bool RConstrainedDelaunay::inCircle(int t, int d) const
{
    const Tri &tri = this->tris[t];
    const Point &pa = this->points[tri.v[0]];
    const Point &pb = this->points[tri.v[1]];
    const Point &pc = this->points[tri.v[2]];
    const Point &pd = this->points[d];

    double adx = pa.x - pd.x, ady = pa.y - pd.y;
    double bdx = pb.x - pd.x, bdy = pb.y - pd.y;
    double cdx = pc.x - pd.x, cdy = pc.y - pd.y;

    double det = (adx * adx + ady * ady) * (bdx * cdy - cdx * bdy)
               - (bdx * bdx + bdy * bdy) * (adx * cdy - cdx * ady)
               + (cdx * cdx + cdy * cdy) * (adx * bdy - bdx * ady);

    return det > 1e-14;
}

void RConstrainedDelaunay::setTri(int t, int v0, int v1, int v2, int n0, int n1, int n2, bool c0, bool c1, bool c2)
{
    Tri &tri = this->tris[t];
    tri.v[0] = v0; tri.v[1] = v1; tri.v[2] = v2;
    tri.n[0] = n0; tri.n[1] = n1; tri.n[2] = n2;
    tri.c[0] = c0; tri.c[1] = c1; tri.c[2] = c2;
    this->vertexTri[v0] = t;
    this->vertexTri[v1] = t;
    this->vertexTri[v2] = t;
}

void RConstrainedDelaunay::replaceNeighbor(int t, int oldTri, int newTri)
{
    if (t < 0)
    {
        return;
    }
    for (int i=0;i<3;i++)
    {
        if (this->tris[t].n[i] == oldTri)
        {
            this->tris[t].n[i] = newTri;
            return;
        }
    }
}

int RConstrainedDelaunay::sharedEdge(int u, int t) const
{
    for (int i=0;i<3;i++)
    {
        if (this->tris[u].n[i] == t)
        {
            return i;
        }
    }
    return -1;
}

int RConstrainedDelaunay::locate(int p) const
{
    int nTris = int(this->tris.size());
    int t = (this->lastTri >= 0 && this->lastTri < nTris) ? this->lastTri : 0;
    int nSteps = 0;
    int start = 0;
    while (nSteps++ < 4 * nTris + 16)
    {
        const Tri &tri = this->tris[t];
        int next = -1;
        for (int k=0;k<3;k++)
        {
            int i = (start + k) % 3;
            if (this->orient(tri.v[(i+1)%3], tri.v[(i+2)%3], p) < 0.0)
            {
                next = tri.n[i];
                break;
            }
        }
        if (next < 0)
        {
            return t;
        }
        t = next;
        start = (start + 1) % 3;
    }
    // Walk failed (degenerate configuration) - exhaustive search.
    for (int i=0;i<nTris;i++)
    {
        const Tri &tri = this->tris[i];
        if (this->orient(tri.v[1], tri.v[2], p) >= 0.0 &&
            this->orient(tri.v[2], tri.v[0], p) >= 0.0 &&
            this->orient(tri.v[0], tri.v[1], p) >= 0.0)
        {
            return i;
        }
    }
    return -1;
}

int RConstrainedDelaunay::insertPoint(int p)
{
    int t = this->locate(p);
    if (t < 0)
    {
        return -1;
    }

    const double tolerance = 1e-12;
    const Point &pp = this->points[p];

    // Coincident vertex.
    for (int i=0;i<3;i++)
    {
        const Point &pv = this->points[this->tris[t].v[i]];
        if (std::abs(pv.x - pp.x) <= tolerance && std::abs(pv.y - pp.y) <= tolerance)
        {
            return this->tris[t].v[i];
        }
    }

    // Point on edge.
    int onEdge = -1;
    for (int i=0;i<3;i++)
    {
        int a = this->tris[t].v[(i+1)%3];
        int b = this->tris[t].v[(i+2)%3];
        double length = std::hypot(this->points[b].x - this->points[a].x, this->points[b].y - this->points[a].y);
        if (std::abs(this->orient(a, b, p)) <= tolerance * length && this->tris[t].n[i] >= 0)
        {
            onEdge = i;
            break;
        }
    }

    std::vector<int> stack;

    if (onEdge < 0)
    {
        Tri old = this->tris[t];
        int t1 = int(this->tris.size());
        int t2 = t1 + 1;
        this->tris.push_back(Tri());
        this->tris.push_back(Tri());

        this->setTri(t,  p, old.v[1], old.v[2], old.n[0], t1, t2, old.c[0], false, false);
        this->setTri(t1, p, old.v[2], old.v[0], old.n[1], t2, t,  old.c[1], false, false);
        this->setTri(t2, p, old.v[0], old.v[1], old.n[2], t,  t1, old.c[2], false, false);

        this->replaceNeighbor(old.n[1], t, t1);
        this->replaceNeighbor(old.n[2], t, t2);

        stack = {t, t1, t2};
    }
    else
    {
        int e = onEdge;
        int u = this->tris[t].n[e];
        int f = this->sharedEdge(u, t);
        if (f < 0)
        {
            return -1;
        }
        Tri ot = this->tris[t];
        Tri ou = this->tris[u];

        int x = ot.v[e], a = ot.v[(e+1)%3], b = ot.v[(e+2)%3];
        int d = ou.v[f];
        int ntA = ot.n[(e+1)%3], ntB = ot.n[(e+2)%3];
        int nuB = ou.n[(f+1)%3], nuA = ou.n[(f+2)%3];
        bool ctA = ot.c[(e+1)%3], ctB = ot.c[(e+2)%3];
        bool cuB = ou.c[(f+1)%3], cuA = ou.c[(f+2)%3];
        bool cab = ot.c[e];

        int tA = t, tC = u;
        int tB = int(this->tris.size());
        int tD = tB + 1;
        this->tris.push_back(Tri());
        this->tris.push_back(Tri());

        this->setTri(tA, p, x, a, ntB, tD, tB, ctB, cab, false);
        this->setTri(tB, p, b, x, ntA, tA, tC, ctA, false, cab);
        this->setTri(tC, p, d, b, nuA, tB, tD, cuA, cab, false);
        this->setTri(tD, p, a, d, nuB, tC, tA, cuB, false, cab);

        this->replaceNeighbor(ntA, t, tB);
        this->replaceNeighbor(nuB, u, tD);

        stack = {tA, tB, tC, tD};
    }

    this->legalize(stack);
    this->lastTri = this->vertexTri[p];
    return p;
}

void RConstrainedDelaunay::flip(int t, int e)
{
    int u = this->tris[t].n[e];
    int f = this->sharedEdge(u, t);

    Tri ot = this->tris[t];
    Tri ou = this->tris[u];

    int p = ot.v[e], a = ot.v[(e+1)%3], b = ot.v[(e+2)%3];
    int d = ou.v[f];
    int ntA = ot.n[(e+1)%3], ntB = ot.n[(e+2)%3];
    int nuB = ou.n[(f+1)%3], nuA = ou.n[(f+2)%3];
    bool ctA = ot.c[(e+1)%3], ctB = ot.c[(e+2)%3];
    bool cuB = ou.c[(f+1)%3], cuA = ou.c[(f+2)%3];

    this->setTri(t, p, a, d, nuB, u, ntB, cuB, false, ctB);
    this->setTri(u, p, d, b, nuA, ntA, t, cuA, ctA, false);

    this->replaceNeighbor(nuB, u, t);
    this->replaceNeighbor(ntA, t, u);
}

void RConstrainedDelaunay::legalize(std::vector<int> &stack)
{
    // Every triangle on stack has the inserted point as vertex 0.
    size_t guard = 0;
    while (!stack.empty() && guard++ < 100000000)
    {
        int t = stack.back();
        stack.pop_back();
        int u = this->tris[t].n[0];
        if (u < 0 || this->tris[t].c[0])
        {
            continue;
        }
        int f = this->sharedEdge(u, t);
        if (f < 0)
        {
            continue;
        }
        if (this->inCircle(t, this->tris[u].v[f]))
        {
            this->flip(t, 0);
            stack.push_back(t);
            stack.push_back(u);
        }
    }
}

bool RConstrainedDelaunay::findEdge(int a, int b, int &t, int &e) const
{
    int t0 = this->vertexTri[a];
    if (t0 < 0)
    {
        return false;
    }
    // Fans around super-triangle vertices are open - search both directions.
    for (int direction=0;direction<2;direction++)
    {
        int tc = t0;
        int guard = 0;
        do
        {
            const Tri &tri = this->tris[tc];
            int i = (tri.v[0] == a) ? 0 : ((tri.v[1] == a) ? 1 : 2);
            if (tri.v[(i+1)%3] == b)
            {
                t = tc;
                e = (i+2)%3;
                return true;
            }
            if (tri.v[(i+2)%3] == b)
            {
                t = tc;
                e = (i+1)%3;
                return true;
            }
            // Rotate counter-clockwise (or clockwise) around vertex a.
            tc = tri.n[(direction == 0) ? (i+1)%3 : (i+2)%3];
        }
        while (tc >= 0 && tc != t0 && guard++ < 100000);
        if (tc == t0)
        {
            // Closed fan was fully searched.
            break;
        }
    }
    return false;
}

void RConstrainedDelaunay::markConstrained(int t, int e)
{
    this->tris[t].c[e] = true;
    int u = this->tris[t].n[e];
    if (u >= 0)
    {
        int f = this->sharedEdge(u, t);
        if (f >= 0)
        {
            this->tris[u].c[f] = true;
        }
    }
}

bool RConstrainedDelaunay::insertConstraint(int a, int b, int depth)
{
    if (depth > 64)
    {
        return false;
    }

    int t, e;
    if (this->findEdge(a, b, t, e))
    {
        this->markConstrained(t, e);
        return true;
    }

    const Point &pa = this->points[a];
    const Point &pb = this->points[b];
    double length = std::hypot(pb.x - pa.x, pb.y - pa.y);
    const double tolerance = 1e-12 * length;

    auto onSegment = [&](int v) -> bool
    {
        if (std::abs(this->orient(a, b, v)) > tolerance)
        {
            return false;
        }
        const Point &pv = this->points[v];
        double s = ((pv.x - pa.x) * (pb.x - pa.x) + (pv.y - pa.y) * (pb.y - pa.y)) / (length * length);
        return (s > 0.0 && s < 1.0);
    };

    // Find triangle around a through which segment leaves vertex a.
    int t0 = this->vertexTri[a];
    int tc = t0;
    int guard = 0;
    int start = -1;
    int startEdge = -1;
    do
    {
        const Tri &tri = this->tris[tc];
        int i = (tri.v[0] == a) ? 0 : ((tri.v[1] == a) ? 1 : 2);
        int v1 = tri.v[(i+1)%3];
        int v2 = tri.v[(i+2)%3];
        if (onSegment(v1))
        {
            return this->insertConstraint(a, v1, depth + 1) && this->insertConstraint(v1, b, depth + 1);
        }
        if (onSegment(v2))
        {
            return this->insertConstraint(a, v2, depth + 1) && this->insertConstraint(v2, b, depth + 1);
        }
        if (this->orient(a, v1, b) > 0.0 && this->orient(a, b, v2) > 0.0)
        {
            start = tc;
            startEdge = i;
            break;
        }
        tc = tri.n[(i+1)%3];
    }
    while (tc >= 0 && tc != t0 && guard++ < 100000);

    if (start < 0)
    {
        return false;
    }

    // Collect edges crossed by the segment.
    std::deque<std::pair<int,int>> crossing;
    int tw = start;
    int ew = startEdge;
    guard = 0;
    while (guard++ < 10000000)
    {
        int x = this->tris[tw].v[(ew+1)%3];
        int y = this->tris[tw].v[(ew+2)%3];
        crossing.push_back({x, y});
        int u = this->tris[tw].n[ew];
        if (u < 0)
        {
            return false;
        }
        if (this->tris[tw].c[ew])
        {
            // Intersecting constraints.
            return false;
        }
        int f = this->sharedEdge(u, tw);
        int d = this->tris[u].v[f];
        if (d == b)
        {
            break;
        }
        if (onSegment(d))
        {
            return this->insertConstraint(a, d, depth + 1) && this->insertConstraint(d, b, depth + 1);
        }
        int u1 = this->tris[u].v[(f+1)%3];
        // One of vertices u1, u2 lies on the same side as d.
        double od = this->orient(a, b, d);
        double o1 = this->orient(a, b, u1);
        int sameSide = ((od > 0.0) == (o1 > 0.0)) ? 1 : 2;
        // Next crossed edge is opposite to the vertex on the same side as d.
        tw = u;
        ew = (sameSide == 1) ? (f+1)%3 : (f+2)%3;
    }

    // Flip crossing edges until segment appears.
    auto properlyIntersects = [&](int x, int y) -> bool
    {
        double o1 = this->orient(a, b, x);
        double o2 = this->orient(a, b, y);
        double o3 = this->orient(x, y, a);
        double o4 = this->orient(x, y, b);
        return (((o1 > 0.0 && o2 < 0.0) || (o1 < 0.0 && o2 > 0.0)) &&
                ((o3 > 0.0 && o4 < 0.0) || (o3 < 0.0 && o4 > 0.0)));
    };

    std::vector<std::pair<int,int>> created;
    size_t maxIterations = 1000 * crossing.size() + 1000;
    size_t iteration = 0;
    while (!crossing.empty())
    {
        if (iteration++ > maxIterations)
        {
            return false;
        }
        auto edge = crossing.front();
        crossing.pop_front();
        int te, ee;
        if (!this->findEdge(edge.first, edge.second, te, ee))
        {
            return false;
        }
        int u = this->tris[te].n[ee];
        int f = this->sharedEdge(u, te);
        int p = this->tris[te].v[ee];
        int d = this->tris[u].v[f];
        int x = edge.first, y = edge.second;
        // Quadrilateral is strictly convex if diagonals properly intersect.
        double o1 = this->orient(p, d, x);
        double o2 = this->orient(p, d, y);
        bool convex = ((o1 > 0.0 && o2 < 0.0) || (o1 < 0.0 && o2 > 0.0));
        if (!convex)
        {
            crossing.push_back(edge);
            continue;
        }
        this->flip(te, ee);
        if (!((p == a && d == b) || (p == b && d == a)) && properlyIntersects(p, d))
        {
            crossing.push_back({p, d});
        }
        else
        {
            created.push_back({p, d});
        }
    }

    if (!this->findEdge(a, b, t, e))
    {
        return false;
    }
    this->markConstrained(t, e);

    // Restore Delaunay property of newly created edges.
    bool swapped = true;
    int passes = 0;
    while (swapped && passes++ < 100)
    {
        swapped = false;
        for (auto &edge : created)
        {
            if ((edge.first == a && edge.second == b) || (edge.first == b && edge.second == a))
            {
                continue;
            }
            int te, ee;
            if (!this->findEdge(edge.first, edge.second, te, ee) || this->tris[te].c[ee])
            {
                continue;
            }
            int u = this->tris[te].n[ee];
            if (u < 0)
            {
                continue;
            }
            int f = this->sharedEdge(u, te);
            int d = this->tris[u].v[f];
            int p = this->tris[te].v[ee];
            // Rotate triangle so that p is vertex 0 for circle test independence.
            if (this->inCircle(te, d))
            {
                double o1 = this->orient(p, d, edge.first);
                double o2 = this->orient(p, d, edge.second);
                if ((o1 > 0.0 && o2 < 0.0) || (o1 < 0.0 && o2 > 0.0))
                {
                    this->flip(te, ee);
                    edge = {p, d};
                    swapped = true;
                }
            }
        }
    }

    return true;
}
