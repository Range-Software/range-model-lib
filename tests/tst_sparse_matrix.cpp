#include <QtTest>

#include "rml_sparse_matrix.h"
#include "rbl_rvector.h"

namespace
{
constexpr double tol = 1.0e-12;
}

class TestSparseMatrix : public QObject
{
    Q_OBJECT

private slots:

    void dimensionsAndValues();
    void accumulation();
    void trace();
    void matrixVectorMultiply();
};

void TestSparseMatrix::dimensionsAndValues()
{
    RSparseMatrix m;
    m.setNRows(2);
    m.addValue(0, 0, 1.0);
    m.addValue(0, 1, 2.0);
    m.addValue(1, 1, 4.0);

    QCOMPARE(m.getNRows(), 2u);
    QVERIFY(qAbs(m.findValue(0, 1) - 2.0) < tol);
    QVERIFY(qAbs(m.findValue(1, 1) - 4.0) < tol);
    // Missing entry returns 0.
    QVERIFY(qAbs(m.findValue(1, 0)) < tol);
}

void TestSparseMatrix::accumulation()
{
    RSparseMatrix m;
    m.setNRows(1);
    m.addValue(0, 0, 1.5);
    m.addValue(0, 0, 2.5);
    QVERIFY(qAbs(m.findValue(0, 0) - 4.0) < tol);
}

void TestSparseMatrix::trace()
{
    RSparseMatrix m;
    m.setNRows(3);
    m.addValue(0, 0, 1.0);
    m.addValue(1, 1, 2.0);
    m.addValue(2, 2, 3.0);
    m.addValue(0, 2, 9.0); // off-diagonal must not affect trace
    QVERIFY(qAbs(m.findTrace() - 6.0) < tol);
}

void TestSparseMatrix::matrixVectorMultiply()
{
    // Fully populated matrix [[1,2],[3,4]].
    RSparseMatrix m;
    m.setNRows(2);
    m.addValue(0, 0, 1.0);
    m.addValue(0, 1, 2.0);
    m.addValue(1, 0, 3.0);
    m.addValue(1, 1, 4.0);

    RRVector x(2, 0.0);
    x[0] = 5.0;
    x[1] = 6.0;

    RRVector y;
    RSparseMatrix::mlt(m, x, y);

    QCOMPARE(y.getNRows(), 2u);
    QVERIFY(qAbs(y[0] - 17.0) < tol); // 1*5 + 2*6
    QVERIFY(qAbs(y[1] - 39.0) < tol); // 3*5 + 4*6

    // A genuinely sparse matrix, where the storage position of a value inside
    // its row is not its column index.
    //   [[0,0,2],
    //    [3,0,0],
    //    [0,4,0]]
    RSparseMatrix s;
    s.setNRows(3);
    s.addValue(0, 2, 2.0);
    s.addValue(1, 0, 3.0);
    s.addValue(2, 1, 4.0);

    RRVector v(3, 0.0);
    v[0] = 7.0;
    v[1] = 8.0;
    v[2] = 9.0;

    // Deliberately reuse a vector which already holds values - the result must
    // replace them, not accumulate on top of them.
    RRVector w(3, 1000.0);
    RSparseMatrix::mlt(s, v, w);

    QCOMPARE(w.getNRows(), 3u);
    QVERIFY(qAbs(w[0] - 18.0) < tol); // 2*9
    QVERIFY(qAbs(w[1] - 21.0) < tol); // 3*7
    QVERIFY(qAbs(w[2] - 32.0) < tol); // 4*8
}

QTEST_APPLESS_MAIN(TestSparseMatrix)

#include "tst_sparse_matrix.moc"
