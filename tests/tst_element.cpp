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
    void interpolateLine();
    void interpolateTriangle();
    void interpolateTetrahedron();
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

void TestElement::interpolateLine()
{
    const std::vector<RNode> nodes = {RNode(0.0,0.0,0.0),RNode(4.0,0.0,0.0)};

    RElement element(R_ELEMENT_TRUSS1);
    element.setNodeId(0,0);
    element.setNodeId(1,1);

    RRVector nodeValues(2);
    nodeValues[0] = 10.0;
    nodeValues[1] = 30.0;

    QCOMPARE(element.interpolate(nodes,RNode(0.0,0.0,0.0),nodeValues),10.0);
    QCOMPARE(element.interpolate(nodes,RNode(4.0,0.0,0.0),nodeValues),30.0);
    QCOMPARE(element.interpolate(nodes,RNode(1.0,0.0,0.0),nodeValues),15.0);

    // The same with distances found when looking for the element.
    RRVector volumes;
    const RNode node(1.0,0.0,0.0);
    QVERIFY(element.isInside(nodes,node,volumes));
    QCOMPARE(element.interpolate(nodes,node,nodeValues,volumes),15.0);
}

void TestElement::interpolateTriangle()
{
    const std::vector<RNode> nodes = {RNode(0.0,0.0,0.0),RNode(4.0,0.0,0.0),RNode(0.0,4.0,0.0)};

    RElement element(R_ELEMENT_TRI1);
    for (uint i=0;i<3;i++)
    {
        element.setNodeId(i,i);
    }

    RRVector nodeValues(3);
    nodeValues[0] = 10.0;
    nodeValues[1] = 30.0;
    nodeValues[2] = 50.0;

    for (uint i=0;i<3;i++)
    {
        QCOMPARE(element.interpolate(nodes,nodes[i],nodeValues),nodeValues[i]);
    }
    QCOMPARE(element.interpolate(nodes,RNode(1.0,1.0,0.0),nodeValues),25.0);
}

void TestElement::interpolateTetrahedron()
{
    const std::vector<RNode> nodes = {RNode(0.0,0.0,0.0),RNode(4.0,0.0,0.0),RNode(0.0,4.0,0.0),RNode(0.0,0.0,4.0)};

    RElement element(R_ELEMENT_TETRA1);
    for (uint i=0;i<4;i++)
    {
        element.setNodeId(i,i);
    }

    RRVector nodeValues(4);
    nodeValues[0] = 10.0;
    nodeValues[1] = 30.0;
    nodeValues[2] = 50.0;
    nodeValues[3] = 70.0;

    for (uint i=0;i<4;i++)
    {
        QCOMPARE(element.interpolate(nodes,nodes[i],nodeValues),nodeValues[i]);
    }
    QCOMPARE(element.interpolate(nodes,RNode(1.0,1.0,1.0),nodeValues),40.0);
}

QTEST_APPLESS_MAIN(TestElement)

#include "tst_element.moc"
