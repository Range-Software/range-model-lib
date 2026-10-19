#ifndef RML_CONSTRAINED_DELAUNAY_H
#define RML_CONSTRAINED_DELAUNAY_H

#include <array>
#include <utility>
#include <vector>

//! Planar constrained Delaunay triangulation of polygonal domains.
//! Constraint segments define domain boundaries - closed rings, holes are
//! resolved by even-odd rule. Points not used by any constraint are inserted
//! as interior (Steiner) points.
class RConstrainedDelaunay
{

    public:

        //! 2D point.
        struct Point
        {
            double x;
            double y;
        };

        //! Output triangle - indexes into input point list, counter-clockwise.
        typedef std::array<unsigned int,3> Triangle;

    protected:

        //! Internal triangle.
        //! Edge i is opposite to vertex v[i]; n[i] is neighbor across edge i.
        struct Tri
        {
            int v[3];
            int n[3];
            bool c[3];
        };

        //! Points (normalized) including 3 super-triangle vertices at the end.
        std::vector<Point> points;
        //! Triangles.
        std::vector<Tri> tris;
        //! One triangle incident to each vertex.
        std::vector<int> vertexTri;
        //! Last visited triangle (walk start).
        int lastTri;

    public:

        //! Constructor.
        RConstrainedDelaunay();

        //! Triangulate.
        //! Return false if triangulation failed.
        bool triangulate(const std::vector<Point> &inputPoints,
                         const std::vector<std::pair<unsigned int,unsigned int>> &constraints,
                         std::vector<Triangle> &triangles);

    protected:

        //! Orientation of point c relative to directed line a->b (positive = left).
        double orient(int a, int b, int c) const;

        //! Return true if point d lies inside circumcircle of triangle t.
        bool inCircle(int t, int d) const;

        //! Set triangle data and update vertex map.
        void setTri(int t, int v0, int v1, int v2, int n0, int n1, int n2, bool c0, bool c1, bool c2);

        //! Replace neighbor reference oldTri by newTri in triangle t.
        void replaceNeighbor(int t, int oldTri, int newTri);

        //! Return index of edge in triangle u which is shared with triangle t.
        int sharedEdge(int u, int t) const;

        //! Insert point. Return vertex index the point was merged with (or itself).
        int insertPoint(int p);

        //! Locate triangle containing point.
        int locate(int p) const;

        //! Flip edge e of triangle t.
        void flip(int t, int e);

        //! Restore Delaunay property starting with given edges (opposite to vertex 0).
        void legalize(std::vector<int> &stack);

        //! Find triangle and edge index for edge (a,b). Return false if not found.
        bool findEdge(int a, int b, int &t, int &e) const;

        //! Insert constraint segment.
        bool insertConstraint(int a, int b, int depth = 0);

        //! Mark edge as constrained on both sides.
        void markConstrained(int t, int e);

};

#endif // RML_CONSTRAINED_DELAUNAY_H
