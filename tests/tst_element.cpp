#include <QtTest>

#include "rml_element.h"

class TestElement : public QObject
{
    Q_OBJECT

private slots:

    void nodeCountByType();
    void typeChangeResizesNodes();
    void nodeIds();
    void hasNodeId();
    void names();
};

void TestElement::nodeCountByType()
{
    QCOMPARE(RElement::getNNodes(R_ELEMENT_POINT), 1u);
    QCOMPARE(RElement::getNNodes(R_ELEMENT_TRUSS1), 2u);
    QCOMPARE(RElement::getNNodes(R_ELEMENT_TRI1), 3u);
    QCOMPARE(RElement::getNNodes(R_ELEMENT_TETRA1), 4u);
}

void TestElement::typeChangeResizesNodes()
{
    RElement element(R_ELEMENT_TRI1);
    QCOMPARE(element.getType(), R_ELEMENT_TRI1);
    QCOMPARE(element.size(), 3u);

    element.setType(R_ELEMENT_TETRA1);
    QCOMPARE(element.getType(), R_ELEMENT_TETRA1);
    QCOMPARE(element.size(), 4u);
}

void TestElement::nodeIds()
{
    RElement element(R_ELEMENT_TRI1);
    element.setNodeId(0, 10);
    element.setNodeId(1, 20);
    element.setNodeId(2, 30);

    QCOMPARE(element.getNodeId(0), 10u);
    QCOMPARE(element.getNodeId(1), 20u);
    QCOMPARE(element.getNodeId(2), 30u);

    element.swapNodeIds(0, 2);
    QCOMPARE(element.getNodeId(0), 30u);
    QCOMPARE(element.getNodeId(2), 10u);
}

void TestElement::hasNodeId()
{
    RElement element(R_ELEMENT_TRUSS1);
    element.setNodeId(0, 5);
    element.setNodeId(1, 7);

    QVERIFY(element.hasNodeId(5));
    QVERIFY(element.hasNodeId(7));
    QVERIFY(!element.hasNodeId(99));
}

void TestElement::names()
{
    QCOMPARE(RElement::getName(R_ELEMENT_POINT), QString("Point"));
    QCOMPARE(RElement::getName(R_ELEMENT_TRI1), QString("Triangle1"));
}

QTEST_APPLESS_MAIN(TestElement)

#include "tst_element.moc"
