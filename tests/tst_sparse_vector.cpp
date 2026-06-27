#include <QtTest>

#include "rml_sparse_vector.h"

class TestSparseVector : public QObject
{
    Q_OBJECT

private slots:

    void insertionKeepsSortedIndexes();
    void addAccumulates();
    void findPosition();
    void getValuesExpandsToDense();
    void addVectorMerges();
};

void TestSparseVector::insertionKeepsSortedIndexes()
{
    RSparseVector<double> v;
    v.addValue(3, 2.0);
    v.addValue(1, 1.0);
    v.addValue(5, 3.0);

    QCOMPARE(v.size(), 3u);
    QCOMPARE(v.getIndex(0), 1u);
    QCOMPARE(v.getIndex(1), 3u);
    QCOMPARE(v.getIndex(2), 5u);
    QCOMPARE(v.getValue(0), 1.0);
    QCOMPARE(v.getValue(1), 2.0);
    QCOMPARE(v.getValue(2), 3.0);
}

void TestSparseVector::addAccumulates()
{
    RSparseVector<double> v;
    v.addValue(2, 1.0);
    v.addValue(2, 4.0);
    QCOMPARE(v.size(), 1u);
    QCOMPARE(v.getValue(0), 5.0);
}

void TestSparseVector::findPosition()
{
    RSparseVector<double> v;
    v.addValue(1, 1.0);
    v.addValue(3, 2.0);
    v.addValue(5, 3.0);

    uint position = 0;
    QVERIFY(v.findPosition(3, position));
    QCOMPARE(position, 1u);
    QVERIFY(!v.findPosition(4, position));
}

void TestSparseVector::getValuesExpandsToDense()
{
    RSparseVector<double> v;
    v.addValue(1, 1.0);
    v.addValue(3, 2.0);
    v.addValue(5, 3.0);

    std::vector<double> dense = v.getValues(6);
    QCOMPARE(uint(dense.size()), 6u);
    QCOMPARE(dense[0], 0.0);
    QCOMPARE(dense[1], 1.0);
    QCOMPARE(dense[3], 2.0);
    QCOMPARE(dense[5], 3.0);
}

void TestSparseVector::addVectorMerges()
{
    RSparseVector<double> a;
    a.addValue(1, 1.0);
    a.addValue(3, 2.0);

    RSparseVector<double> b;
    b.addValue(3, 4.0);
    b.addValue(4, 5.0);

    a.addVector(b);
    QCOMPARE(a.size(), 3u);

    uint position = 0;
    QVERIFY(a.findPosition(3, position));
    QCOMPARE(a.getValue(position), 6.0); // 2 + 4
    QVERIFY(a.findPosition(4, position));
    QCOMPARE(a.getValue(position), 5.0);
}

QTEST_APPLESS_MAIN(TestSparseVector)

#include "tst_sparse_vector.moc"
