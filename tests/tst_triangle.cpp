#include <QtTest>

#include "rml_triangle.h"
#include "rml_node.h"

namespace
{
constexpr double tol = 1.0e-9;
}

class TestTriangle : public QObject
{
    Q_OBJECT

private slots:

    void area();
    void staticArea();
    void normalIsUnitAndPerpendicular();
    void longestEdge();
    void pointInside();
};

void TestTriangle::area()
{
    RTriangle triangle(RNode(0.0, 0.0, 0.0),
                       RNode(3.0, 0.0, 0.0),
                       RNode(0.0, 4.0, 0.0));
    // Right triangle, legs 3 and 4 -> area = 6.
    QVERIFY(qAbs(triangle.findArea() - 6.0) < tol);
}

void TestTriangle::staticArea()
{
    double a = RTriangle::findArea(RNode(0.0, 0.0, 0.0),
                                   RNode(2.0, 0.0, 0.0),
                                   RNode(0.0, 2.0, 0.0));
    QVERIFY(qAbs(a - 2.0) < tol);
}

void TestTriangle::normalIsUnitAndPerpendicular()
{
    RTriangle triangle(RNode(0.0, 0.0, 0.0),
                       RNode(3.0, 0.0, 0.0),
                       RNode(0.0, 4.0, 0.0));
    RR3Vector normal = triangle.getNormal();

    QVERIFY(qAbs(normal.length() - 1.0) < tol);
    // Triangle lies in the XY plane, so the normal is along Z.
    QVERIFY(qAbs(normal[0]) < tol);
    QVERIFY(qAbs(normal[1]) < tol);
    QVERIFY(qAbs(qAbs(normal[2]) - 1.0) < tol);
}

void TestTriangle::longestEdge()
{
    RTriangle triangle(RNode(0.0, 0.0, 0.0),
                       RNode(3.0, 0.0, 0.0),
                       RNode(0.0, 4.0, 0.0));
    // Edges: 3, 4, 5 -> longest is the hypotenuse 5.
    QVERIFY(qAbs(triangle.findLongestEdgeLength() - 5.0) < tol);
}

void TestTriangle::pointInside()
{
    RTriangle triangle(RNode(0.0, 0.0, 0.0),
                       RNode(4.0, 0.0, 0.0),
                       RNode(0.0, 4.0, 0.0));
    QVERIFY(triangle.isPointInside(RR3Vector(1.0, 1.0, 0.0)));
    QVERIFY(!triangle.isPointInside(RR3Vector(3.0, 3.0, 0.0)));
}

QTEST_APPLESS_MAIN(TestTriangle)

#include "tst_triangle.moc"
