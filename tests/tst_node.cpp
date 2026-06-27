#include <QtTest>

#include "rml_node.h"

namespace
{
constexpr double tol = 1.0e-12;
}

class TestNode : public QObject
{
    Q_OBJECT

private slots:

    void construction();
    void setAndMove();
    void distance();
    void toVector();
    void scaleAndTranslate();
    void equality();
};

void TestNode::construction()
{
    RNode def;
    QCOMPARE(def.getX(), 0.0);
    QCOMPARE(def.getY(), 0.0);
    QCOMPARE(def.getZ(), 0.0);

    RNode n(1.0, 2.0, 3.0);
    QCOMPARE(n.getX(), 1.0);
    QCOMPARE(n.getY(), 2.0);
    QCOMPARE(n.getZ(), 3.0);
}

void TestNode::setAndMove()
{
    RNode n;
    n.set(1.0, 2.0, 3.0);
    QCOMPARE(n.getX(), 1.0);
    QCOMPARE(n.getZ(), 3.0);

    n.move(RR3Vector(1.0, -1.0, 0.5));
    QCOMPARE(n.getX(), 2.0);
    QCOMPARE(n.getY(), 1.0);
    QCOMPARE(n.getZ(), 3.5);
}

void TestNode::distance()
{
    RNode a(0.0, 0.0, 0.0);
    RNode b(3.0, 4.0, 0.0);
    QVERIFY(qAbs(a.getDistance(b) - 5.0) < tol);
    QVERIFY(qAbs(b.getDistance(a) - 5.0) < tol);
    QCOMPARE(a.getDistance(a), 0.0);
}

void TestNode::toVector()
{
    RNode n(1.0, 2.0, 3.0);
    RR3Vector v = n.toVector();
    QCOMPARE(v.getNRows(), 3u);
    QCOMPARE(v[0], 1.0);
    QCOMPARE(v[1], 2.0);
    QCOMPARE(v[2], 3.0);
}

void TestNode::scaleAndTranslate()
{
    RNode n(1.0, 2.0, 3.0);
    n.scale(2.0);
    QCOMPARE(n.getX(), 2.0);
    QCOMPARE(n.getY(), 4.0);
    QCOMPARE(n.getZ(), 6.0);

    n.translate(RR3Vector(-2.0, -4.0, -6.0));
    QCOMPARE(n.getX(), 0.0);
    QCOMPARE(n.getY(), 0.0);
    QCOMPARE(n.getZ(), 0.0);
}

void TestNode::equality()
{
    RNode a(1.0, 2.0, 3.0);
    RNode b(1.0, 2.0, 3.0);
    RNode c(1.0, 2.0, 4.0);

    QVERIFY(a == b);
    QVERIFY(a != c);
}

QTEST_APPLESS_MAIN(TestNode)

#include "tst_node.moc"
