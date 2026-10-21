#include <QtTest>
#include <QMap>
#include <QPair>

#include <cmath>

#include "rml_polygon.h"
#include "rml_triangle.h"
#include "rml_node.h"

namespace
{
constexpr double tol = 1.0e-9;

//! Verify that triangles form a valid patch of the polygon given by nodes:
//! n-2 triangles, each polygon edge used exactly once in polygon direction,
//! each diagonal used exactly twice in opposite directions (no overlaps, no
//! holes) and all triangles oriented along the reference normal.
void verifyPatch(const std::vector<RNode> &nodes, const std::vector<RElement> &triangles, const RR3Vector &referenceNormal)
{
    uint n = uint(nodes.size());

    QCOMPARE(uint(triangles.size()), n - 2);

    QMap<QPair<uint,uint>,int> directedEdges;
    for (const RElement &triangle : triangles)
    {
        QCOMPARE(triangle.size(), uint(3));
        for (uint k=0;k<3;k++)
        {
            directedEdges[QPair<uint,uint>(triangle.getNodeId(k),triangle.getNodeId((k+1)%3))]++;
        }

        RTriangle t(nodes[triangle.getNodeId(0)],nodes[triangle.getNodeId(1)],nodes[triangle.getNodeId(2)]);
        QVERIFY(t.findArea() > tol);
        QVERIFY(RRVector::dot(t.getNormal(),referenceNormal) > 0.0);
    }

    for (auto iter = directedEdges.cbegin(); iter != directedEdges.cend(); ++iter)
    {
        uint a = iter.key().first;
        uint b = iter.key().second;
        QCOMPARE(iter.value(), 1);
        bool isPolygonEdge = (b == (a+1)%n);
        bool hasReverse = directedEdges.contains(QPair<uint,uint>(b,a));
        // Polygon edges have no reverse, diagonals are shared by two triangles.
        QVERIFY(isPolygonEdge != hasReverse);
    }
}
}

class TestPolygon : public QObject
{
    Q_OBJECT

private slots:

    void triangle();
    void concavePolygon();
    void combPolygon();
    void nonCoplanarRing();
};

void TestPolygon::triangle()
{
    std::vector<RNode> nodes = { RNode(0.0,0.0,0.0), RNode(1.0,0.0,0.0), RNode(0.0,1.0,0.0) };
    std::vector<RElement> triangles = RPolygon::triangulate(nodes,true);
    verifyPatch(nodes,triangles,RR3Vector(0.0,0.0,1.0));
}

void TestPolygon::concavePolygon()
{
    // Arrow shape with reflex node at (2,1) - counter-clockwise.
    std::vector<RNode> nodes = {
        RNode(0.0,0.0,0.0),
        RNode(4.0,0.0,0.0),
        RNode(4.0,4.0,0.0),
        RNode(2.0,1.0,0.0),
        RNode(0.0,4.0,0.0)
    };
    std::vector<RElement> triangles = RPolygon::triangulate(nodes,true);
    verifyPatch(nodes,triangles,RR3Vector(0.0,0.0,1.0));
}

void TestPolygon::combPolygon()
{
    // Comb with three teeth - most corners are not ears.
    std::vector<RNode> nodes = {
        RNode(0.0,0.0,0.0),
        RNode(5.0,0.0,0.0),
        RNode(5.0,3.0,0.0),
        RNode(4.0,3.0,0.0),
        RNode(4.0,1.0,0.0),
        RNode(3.0,1.0,0.0),
        RNode(3.0,3.0,0.0),
        RNode(2.0,3.0,0.0),
        RNode(2.0,1.0,0.0),
        RNode(1.0,1.0,0.0),
        RNode(1.0,3.0,0.0),
        RNode(0.0,3.0,0.0)
    };
    std::vector<RElement> triangles = RPolygon::triangulate(nodes,true);
    verifyPatch(nodes,triangles,RR3Vector(0.0,0.0,1.0));
}

void TestPolygon::nonCoplanarRing()
{
    // Ring of nodes around a removed node on a cylinder of radius 1 around
    // the X axis - nodes are not coplanar. One node is pulled inwards to make
    // the projected ring concave.
    std::vector<RNode> nodes;
    const uint n = 8;
    for (uint i=0;i<n;i++)
    {
        double phi = 2.0 * M_PI * double(i) / double(n);
        double r = (i == 3) ? 0.15 : 0.3;
        double x = r * std::cos(phi);
        double s = r * std::sin(phi);
        nodes.push_back(RNode(x,std::sin(s),std::cos(s)));
    }
    std::vector<RElement> triangles = RPolygon::triangulate(nodes,true);
    verifyPatch(nodes,triangles,RR3Vector(0.0,0.0,1.0));
}

QTEST_APPLESS_MAIN(TestPolygon)

#include "tst_polygon.moc"
