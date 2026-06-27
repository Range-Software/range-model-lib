#include <QtTest>

#include "rml_segment.h"
#include "rml_node.h"

namespace
{
constexpr double tol = 1.0e-12;
}

class TestSegment : public QObject
{
    Q_OBJECT

private slots:

    void length();
    void staticLength();
    void direction();
    void pointInside();
};

void TestSegment::length()
{
    RSegment segment(RNode(0.0, 0.0, 0.0), RNode(3.0, 4.0, 0.0));
    QVERIFY(qAbs(segment.findLength() - 5.0) < tol);
}

void TestSegment::staticLength()
{
    QVERIFY(qAbs(RSegment::findLength(RNode(0.0, 0.0, 0.0),
                                      RNode(0.0, 0.0, 2.0)) - 2.0) < tol);
}

void TestSegment::direction()
{
    RSegment segment(RNode(1.0, 1.0, 1.0), RNode(1.0, 1.0, 4.0));
    RR3Vector d = segment.findDirection();
    QCOMPARE(d.getNRows(), 3u);
    QVERIFY(qAbs(d[0]) < tol);
    QVERIFY(qAbs(d[1]) < tol);
    QVERIFY(qAbs(d[2] - 3.0) < tol);
}

void TestSegment::pointInside()
{
    RSegment segment(RNode(0.0, 0.0, 0.0), RNode(2.0, 0.0, 0.0));
    QVERIFY(segment.isPointInside(RR3Vector(1.0, 0.0, 0.0)));
    QVERIFY(!segment.isPointInside(RR3Vector(1.0, 1.0, 0.0)));
    QVERIFY(!segment.isPointInside(RR3Vector(3.0, 0.0, 0.0)));
}

QTEST_APPLESS_MAIN(TestSegment)

#include "tst_segment.moc"
