#include <QtTest>

#include "rml_tetrahedron.h"
#include "rml_node.h"

namespace
{
constexpr double tol = 1.0e-9;
}

class TestTetrahedron : public QObject
{
    Q_OBJECT

private slots:

    void unitVolume();
    void staticVolume();
    void scaledVolume();
};

void TestTetrahedron::unitVolume()
{
    RTetrahedron tetrahedron(RNode(0.0, 0.0, 0.0),
                             RNode(1.0, 0.0, 0.0),
                             RNode(0.0, 1.0, 0.0),
                             RNode(0.0, 0.0, 1.0));
    // Volume of the unit corner tetrahedron is 1/6.
    QVERIFY(qAbs(qAbs(tetrahedron.findVolume()) - 1.0 / 6.0) < tol);
}

void TestTetrahedron::staticVolume()
{
    double v = RTetrahedron::findVolume(RNode(0.0, 0.0, 0.0),
                                        RNode(1.0, 0.0, 0.0),
                                        RNode(0.0, 1.0, 0.0),
                                        RNode(0.0, 0.0, 1.0));
    QVERIFY(qAbs(qAbs(v) - 1.0 / 6.0) < tol);
}

void TestTetrahedron::scaledVolume()
{
    // Scaling every edge by 2 scales the volume by 8.
    RTetrahedron tetrahedron(RNode(0.0, 0.0, 0.0),
                             RNode(2.0, 0.0, 0.0),
                             RNode(0.0, 2.0, 0.0),
                             RNode(0.0, 0.0, 2.0));
    QVERIFY(qAbs(qAbs(tetrahedron.findVolume()) - 8.0 / 6.0) < tol);
}

QTEST_APPLESS_MAIN(TestTetrahedron)

#include "tst_tetrahedron.moc"
