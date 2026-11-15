#include <QTest>

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
    void secondOrderTypes();
    void secondOrderShapeFunctions_data();
    void secondOrderShapeFunctions();
    void secondOrderMassVectors();
    void secondOrderGeometry();
    void secondOrderInterpolation();
    void secondOrderEdgeElements();
    void secondOrderNeighbors();
    void nodeShapeFunctions();
};

//! Second order tetrahedron with corners of a first order one and with nodes in the middle of its edges.
static RElement createTetrahedron2(std::vector<RNode> &nodes)
{
    nodes = {RNode(0.0,0.0,0.0),RNode(4.0,0.0,0.0),RNode(0.0,4.0,0.0),RNode(0.0,0.0,4.0)};
    RElement element(R_ELEMENT_TETRA2);
    for (uint i=0;i<4;i++)
    {
        element.setNodeId(i,i);
    }
    for (uint i=0;i<6;i++)
    {
        uint c1, c2;
        RElement::getMidEdgeNodeCorners(R_ELEMENT_TETRA2,i,c1,c2);
        nodes.push_back(RNode(0.5*(nodes[c1].getX()+nodes[c2].getX()),
                              0.5*(nodes[c1].getY()+nodes[c2].getY()),
                              0.5*(nodes[c1].getZ()+nodes[c2].getZ())));
        element.setNodeId(4+i,4+i);
    }
    return element;
}

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

void TestElement::secondOrderTypes()
{
    QCOMPARE(RElement::getNNodes(R_ELEMENT_TRUSS2), 3u);
    QCOMPARE(RElement::getNNodes(R_ELEMENT_TRI2), 6u);
    QCOMPARE(RElement::getNNodes(R_ELEMENT_TETRA2), 10u);
    QCOMPARE(RElement::getNNeighbors(R_ELEMENT_TETRA2), 4u);
    QVERIFY(R_ELEMENT_TYPE_IS_VOLUME(R_ELEMENT_TETRA2));

    QCOMPARE(RElement::getOrder(R_ELEMENT_TETRA1), 1u);
    QCOMPARE(RElement::getOrder(R_ELEMENT_TETRA2), 2u);
    QCOMPARE(RElement::getLinearType(R_ELEMENT_TETRA2), R_ELEMENT_TETRA1);
    QCOMPARE(RElement::getLinearType(R_ELEMENT_TRI2), R_ELEMENT_TRI1);
    QCOMPARE(RElement::getLinearType(R_ELEMENT_TRUSS2), R_ELEMENT_TRUSS1);
    QCOMPARE(RElement::getLinearType(R_ELEMENT_QUAD1), R_ELEMENT_QUAD1);
    QCOMPARE(RElement::getQuadraticType(R_ELEMENT_TETRA1), R_ELEMENT_TETRA2);
    QCOMPARE(RElement::getQuadraticType(R_ELEMENT_QUAD1), R_ELEMENT_NONE);
    QCOMPARE(RElement::getNCornerNodes(R_ELEMENT_TETRA2), 4u);
    QCOMPARE(RElement::getNCornerNodes(R_ELEMENT_TRI1), 3u);
    QCOMPARE(RElement::getNMidEdgeNodes(R_ELEMENT_TETRA2), 6u);
    QCOMPARE(RElement::getNMidEdgeNodes(R_ELEMENT_TETRA1), 0u);

    // Type is found by its name in a file.
    QCOMPARE(RElement::getTypeFromId(RElement::getId(R_ELEMENT_TETRA2)), R_ELEMENT_TETRA2);
    // Types which were there before keep their values.
    QCOMPARE(int(R_ELEMENT_TETRA1), 10);
    QCOMPARE(int(R_ELEMENT_HEXA1), 12);
}

void TestElement::secondOrderShapeFunctions_data()
{
    QTest::addColumn<int>("type");
    QTest::addColumn<double>("size");
    QTest::newRow("truss2") << int(R_ELEMENT_TRUSS2) << 2.0;
    QTest::newRow("triangle2") << int(R_ELEMENT_TRI2) << 0.5;
    QTest::newRow("tetrahedron2") << int(R_ELEMENT_TETRA2) << 1.0/6.0;
}

void TestElement::secondOrderShapeFunctions()
{
    QFETCH(int,type);
    QFETCH(double,size);
    const RElementType elementType = RElementType(type);
    const uint nn = RElement::getNNodes(elementType);

    QVERIFY(RElement::getNIntegrationPoints(elementType) > 0);
    QVERIFY(!RElement::hasConstantDerivative(elementType));
    // Weights give size of the reference element.
    QVERIFY(std::fabs(RElement::getTotalWeightFactor(elementType) - size) < 1.0e-14);

    for (uint i=0;i<RElement::getNIntegrationPoints(elementType);i++)
    {
        const RElementShapeFunction &rShapeFunction = RElement::getShapeFunction(elementType,i);
        QVERIFY(rShapeFunction.getW() > 0.0);
        QCOMPARE(rShapeFunction.getN().size(), size_t(nn));
        QCOMPARE(rShapeFunction.getDN().getNRows(), nn);
        // Shape functions give one in total, so their derivatives give zero.
        double sum = 0.0;
        for (uint j=0;j<nn;j++)
        {
            sum += rShapeFunction.getN()[j];
        }
        QVERIFY(std::fabs(sum - 1.0) < 1.0e-14);
        for (uint d=0;d<rShapeFunction.getDN().getNColumns();d++)
        {
            double derivativeSum = 0.0;
            for (uint j=0;j<nn;j++)
            {
                derivativeSum += rShapeFunction.getDN()[j][d];
            }
            QVERIFY(std::fabs(derivativeSum) < 1.0e-13);
        }
    }

    // Mass matrix is symmetric and gives size of the reference element in total.
    const RRMatrix &M = RElement::getMassMatrix(elementType);
    double massSum = 0.0;
    for (uint i=0;i<nn;i++)
    {
        QVERIFY(M[i][i] > 0.0);
        for (uint j=0;j<nn;j++)
        {
            QVERIFY(std::fabs(M[i][j] - M[j][i]) < 1.0e-15);
            massSum += M[i][j];
        }
    }
    QVERIFY(std::fabs(massSum - size) < 1.0e-14);
}

void TestElement::secondOrderMassVectors()
{
    // Integrals of shape functions which are known exactly.
    const RRVector &mLine = RElement::getMassVector(R_ELEMENT_TRUSS2);
    QVERIFY(std::fabs(mLine[0] - 1.0/3.0) < 1.0e-14);
    QVERIFY(std::fabs(mLine[1] - 1.0/3.0) < 1.0e-14);
    QVERIFY(std::fabs(mLine[2] - 4.0/3.0) < 1.0e-14);

    const RRVector &mTriangle = RElement::getMassVector(R_ELEMENT_TRI2);
    for (uint i=0;i<3;i++)
    {
        QVERIFY(std::fabs(mTriangle[i]) < 1.0e-14);
        QVERIFY(std::fabs(mTriangle[3+i] - 1.0/6.0) < 1.0e-14);
    }

    const RRVector &mTetrahedron = RElement::getMassVector(R_ELEMENT_TETRA2);
    for (uint i=0;i<4;i++)
    {
        QVERIFY(std::fabs(mTetrahedron[i] + 1.0/120.0) < 1.0e-14);
    }
    for (uint i=4;i<10;i++)
    {
        QVERIFY(std::fabs(mTetrahedron[i] - 1.0/30.0) < 1.0e-14);
    }

    // Mass matrices of a triangle and of a tetrahedron, which need the fourth degree to be exact.
    const RRMatrix &MTriangle = RElement::getMassMatrix(R_ELEMENT_TRI2);
    QVERIFY(std::fabs(MTriangle[0][0] - 1.0/60.0) < 1.0e-14);
    QVERIFY(std::fabs(MTriangle[0][1] + 1.0/360.0) < 1.0e-14);
    QVERIFY(std::fabs(MTriangle[3][3] - 4.0/45.0) < 1.0e-14);
    QVERIFY(std::fabs(MTriangle[3][4] - 2.0/45.0) < 1.0e-14);
    const RRMatrix &MTetrahedron = RElement::getMassMatrix(R_ELEMENT_TETRA2);
    QVERIFY(std::fabs(MTetrahedron[0][0] - 1.0/420.0) < 1.0e-14);
    QVERIFY(std::fabs(MTetrahedron[0][1] - 1.0/2520.0) < 1.0e-14);
    QVERIFY(std::fabs(MTetrahedron[4][4] - 4.0/315.0) < 1.0e-14);
}

void TestElement::secondOrderGeometry()
{
    std::vector<RNode> nodes;
    const RElement element = createTetrahedron2(nodes);

    double volume = 0.0;
    QVERIFY(element.findVolume(nodes,volume));
    QVERIFY(std::fabs(volume - 64.0/6.0) < 1.0e-12);
    QVERIFY(element.findSize(nodes) > 0.0);
    QVERIFY(element.isInside(nodes,RNode(1.0,1.0,1.0)));
    QVERIFY(!element.isInside(nodes,RNode(3.0,3.0,3.0)));

    // Jacobian of a straight element is the same at all points and gives its volume.
    double integral = 0.0;
    for (uint i=0;i<RElement::getNIntegrationPoints(R_ELEMENT_TETRA2);i++)
    {
        RRMatrix J, Rt;
        const double detJ = element.findJacobian(nodes,i,J,Rt);
        integral += detJ * RElement::getShapeFunction(R_ELEMENT_TETRA2,i).getW();
    }
    QVERIFY(std::fabs(integral - volume) < 1.0e-12);

    // Triangle keeps its mid-edge nodes on their edges when its normal is swapped.
    RElement triangle(R_ELEMENT_TRI2);
    for (uint i=0;i<6;i++)
    {
        triangle.setNodeId(i,10+i);
    }
    QVERIFY(triangle.swapNormal());
    QCOMPARE(triangle.getNodeId(0),11u);
    QCOMPARE(triangle.getNodeId(1),10u);
    QCOMPARE(triangle.getNodeId(2),12u);
    QCOMPARE(triangle.getNodeId(3),13u);
    QCOMPARE(triangle.getNodeId(4),15u);
    QCOMPARE(triangle.getNodeId(5),14u);
}

void TestElement::secondOrderInterpolation()
{
    std::vector<RNode> nodes;
    const RElement element = createTetrahedron2(nodes);

    // A function of the second degree is interpolated exactly.
    auto f = [](const RNode &node)
    {
        return 1.0 + 2.0*node.getX() - node.getY() + node.getX()*node.getX() + 3.0*node.getY()*node.getZ() - node.getZ()*node.getZ();
    };
    RRVector nodeValues(10);
    for (uint i=0;i<10;i++)
    {
        nodeValues[i] = f(nodes[i]);
    }
    for (uint i=0;i<10;i++)
    {
        QVERIFY(std::fabs(element.interpolate(nodes,nodes[i],nodeValues) - nodeValues[i]) < 1.0e-12);
    }
    const RNode inside(0.7,1.1,0.4);
    QVERIFY(std::fabs(element.interpolate(nodes,inside,nodeValues) - f(inside)) < 1.0e-12);

    RRVector volumes;
    QVERIFY(element.isInside(nodes,inside,volumes));
    QVERIFY(std::fabs(element.interpolate(nodes,inside,nodeValues,volumes) - f(inside)) < 1.0e-12);

    RRVector ratios;
    QVERIFY(element.findClosestPoint(nodes,inside,ratios) < 1.0e-12);
    QCOMPARE(ratios.size(), size_t(10));
    double value = 0.0;
    for (uint i=0;i<10;i++)
    {
        value += ratios[i] * nodeValues[i];
    }
    QVERIFY(std::fabs(value - f(inside)) < 1.0e-12);

    // Line with its node in the middle.
    const std::vector<RNode> lineNodes = {RNode(0.0,0.0,0.0),RNode(4.0,0.0,0.0),RNode(2.0,0.0,0.0)};
    RElement line(R_ELEMENT_TRUSS2);
    for (uint i=0;i<3;i++)
    {
        line.setNodeId(i,i);
    }
    RRVector lineValues(3);
    lineValues[0] = 0.0;
    lineValues[1] = 16.0;
    lineValues[2] = 4.0;
    QVERIFY(std::fabs(line.interpolate(lineNodes,RNode(1.0,0.0,0.0),lineValues) - 1.0) < 1.0e-12);
    double length = 0.0;
    QVERIFY(line.findLength(lineNodes,length));
    QCOMPARE(length,4.0);
}

void TestElement::secondOrderEdgeElements()
{
    std::vector<RNode> nodes;
    const RElement element = createTetrahedron2(nodes);

    QCOMPARE(element.getNEdgeElements(), 4u);
    const std::vector<RElement> faces = element.generateEdgeElements();
    const std::vector<RElement> linearFaces = element.getLinearElement().generateEdgeElements();
    QCOMPARE(faces.size(), size_t(4));
    for (uint i=0;i<4;i++)
    {
        QCOMPARE(faces[i].getType(), R_ELEMENT_TRI2);
        for (uint j=0;j<3;j++)
        {
            // Corners are those of a face of the first order element.
            QCOMPARE(faces[i].getNodeId(j), linearFaces[i].getNodeId(j));
            QVERIFY(element.nodeIsOnEdge(faces[i].getNodeId(j),i));
            // Nodes in the middle of edges lie between their corners.
            const RNode &n1 = nodes[faces[i].getNodeId(j)];
            const RNode &n2 = nodes[faces[i].getNodeId((j+1)%3)];
            const RNode &nm = nodes[faces[i].getNodeId(3+j)];
            QVERIFY(std::fabs(nm.getX() - 0.5*(n1.getX()+n2.getX())) < 1.0e-12);
            QVERIFY(std::fabs(nm.getY() - 0.5*(n1.getY()+n2.getY())) < 1.0e-12);
            QVERIFY(std::fabs(nm.getZ() - 0.5*(n1.getZ()+n2.getZ())) < 1.0e-12);
            QVERIFY(element.nodeIsOnEdge(faces[i].getNodeId(3+j),i));
        }
        // The node opposite to a face is not on it.
        QVERIFY(!element.nodeIsOnEdge(i,i));

        const std::vector<RElement> edges = faces[i].generateEdgeElements();
        QCOMPARE(edges.size(), size_t(3));
        for (const RElement &rEdge : edges)
        {
            QCOMPARE(rEdge.getType(), R_ELEMENT_TRUSS2);
            QCOMPARE(rEdge.generateEdgeElements().size(), size_t(2));
        }
    }
}

void TestElement::secondOrderNeighbors()
{
    // Two tetrahedra which share a face with nodes 1, 2, 3.
    RElement e1(R_ELEMENT_TETRA2);
    RElement e2(R_ELEMENT_TETRA2);
    const uint n1[10] = {0,1,2,3, 10,11,12,13,14,15};
    const uint n2[10] = {4,2,1,3, 20,11,21,22,15,14};
    for (uint i=0;i<10;i++)
    {
        e1.setNodeId(i,n1[i]);
        e2.setNodeId(i,n2[i]);
    }
    QVERIFY(e1.isNeighbor(e2));
    QCOMPARE(e1.findEdgePositionForNeighborElement(e2), 0u);
    QCOMPARE(e2.findEdgePositionForNeighborElement(e1), 0u);

    // Triangles sharing an edge 1-2, which they go along in opposite directions.
    RElement t1(R_ELEMENT_TRI2);
    RElement t2(R_ELEMENT_TRI2);
    const uint m1[6] = {0,1,2, 10,11,12};
    const uint m2[6] = {2,1,3, 11,13,14};
    for (uint i=0;i<6;i++)
    {
        t1.setNodeId(i,m1[i]);
        t2.setNodeId(i,m2[i]);
    }
    QVERIFY(t1.isNeighbor(t2));
    QVERIFY(t1.isNeighborNormalSync(t2));
    QVERIFY(t2.swapNormal());
    QVERIFY(!t1.isNeighborNormalSync(t2));
}

void TestElement::nodeShapeFunctions()
{
    for (RElementType type : {R_ELEMENT_TRUSS1,R_ELEMENT_TRUSS2,R_ELEMENT_TRI1,R_ELEMENT_TRI2,R_ELEMENT_TETRA1,R_ELEMENT_TETRA2})
    {
        const uint nn = RElement::getNNodes(type);
        for (uint p=0;p<nn;p++)
        {
            const RElementShapeFunction shapeFunction = RElement::generateNodeShapeFunction(type,p);
            QCOMPARE(shapeFunction.getN().size(),size_t(nn));
            QCOMPARE(shapeFunction.getDN().getNRows(),nn);
            // A shape function is one at its own node and zero at the others.
            for (uint m=0;m<nn;m++)
            {
                QVERIFY(std::fabs(shapeFunction.getN()[m] - (m == p ? 1.0 : 0.0)) < 1.0e-15);
            }
            for (uint d=0;d<shapeFunction.getDN().getNColumns();d++)
            {
                double sum = 0.0;
                for (uint m=0;m<nn;m++)
                {
                    sum += shapeFunction.getDN()[m][d];
                }
                QVERIFY(std::fabs(sum) < 1.0e-14);
            }
        }
    }

    // Derivatives at a node of a second order tetrahedron give the exact gradient of a quadratic function.
    std::vector<RNode> nodes;
    const RElement element = createTetrahedron2(nodes);
    RRMatrix J, Rt;
    element.findJacobian(nodes,0,J,Rt);
    auto f = [](const RNode &n) { return n.getX()*n.getX() + 2.0*n.getY()*n.getZ(); };
    for (uint p=0;p<10;p++)
    {
        const RElementShapeFunction shapeFunction = RElement::generateNodeShapeFunction(R_ELEMENT_TETRA2,p);
        const RRMatrix &dN = shapeFunction.getDN();
        double gradient[3] = {0.0,0.0,0.0};
        for (uint m=0;m<10;m++)
        {
            for (uint d=0;d<3;d++)
            {
                gradient[d] += (dN[m][0]*J[d][0] + dN[m][1]*J[d][1] + dN[m][2]*J[d][2]) * f(nodes[m]);
            }
        }
        const RNode &n = nodes[p];
        QVERIFY(std::fabs(gradient[0] - 2.0*n.getX()) < 1.0e-12);
        QVERIFY(std::fabs(gradient[1] - 2.0*n.getZ()) < 1.0e-12);
        QVERIFY(std::fabs(gradient[2] - 2.0*n.getY()) < 1.0e-12);
    }
}

QTEST_APPLESS_MAIN(TestElement)

#include "tst_element.moc"
