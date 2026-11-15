#include <QTest>
#include <QTemporaryDir>

#include <algorithm>
#include <cmath>
#include <map>
#include <vector>

#include "rml_mesh_generator.h"
#include "rml_model.h"

//! Generation of second order (quadratic) elements by both mesh generators.
class TestMeshSecondOrder : public QObject
{
    Q_OBJECT

    private:

        //! Cube surface with a point on its corner, a line on its edge and a line inside of it.
        static RModel createModel()
        {
            RModel model;
            const double nodes[10][3] = {{0,0,0},{1,0,0},{1,1,0},{0,1,0},{0,0,1},{1,0,1},{1,1,1},{0,1,1},
                                         {0.25,0.5,0.5},{0.75,0.5,0.5}};
            for (const auto &node : nodes)
            {
                model.addNode(RNode(node[0],node[1],node[2]));
            }

            model.addSurface(RSurface());
            const uint triangles[12][3] = {{0,2,1},{0,3,2},{4,5,6},{4,6,7},
                                           {0,1,5},{0,5,4},{1,2,6},{1,6,5},
                                           {2,3,7},{2,7,6},{3,0,4},{3,4,7}};
            for (const auto &triangle : triangles)
            {
                RElement element(R_ELEMENT_TRI1);
                for (uint i=0;i<3;i++)
                {
                    element.setNodeId(i,triangle[i]);
                }
                model.addElement(element,true,model.getEntityGroupID(R_ENTITY_GROUP_SURFACE,0));
            }

            model.addPoint(RPoint());
            RElement point(R_ELEMENT_POINT);
            point.setNodeId(0,0);
            model.addElement(point,true,model.getEntityGroupID(R_ENTITY_GROUP_POINT,0));

            const uint lines[2][2] = {{0,1},{8,9}};
            for (uint i=0;i<2;i++)
            {
                model.addLine(RLine());
                RElement element(R_ELEMENT_TRUSS1);
                element.setNodeId(0,lines[i][0]);
                element.setNodeId(1,lines[i][1]);
                model.addElement(element,true,model.getEntityGroupID(R_ENTITY_GROUP_LINE,i));
            }
            return model;
        }

        static RMeshInput createMeshInput(RMeshGeneratorType generatorType, double volumeConstraint, uint elementOrder)
        {
            RMeshInput meshInput;
            meshInput.setGeneratorType(generatorType);
            meshInput.setVerbose(false);
            meshInput.setKeepResults(false);
            meshInput.setReconstruct(false);
            meshInput.setVolumeConstraint(volumeConstraint);
            meshInput.setElementOrder(elementOrder);
            return meshInput;
        }

        //! Function of the second degree, which second order elements represent exactly.
        static double findQuadraticValue(const RNode &node)
        {
            return 1.0 + node.getX() - 2.0*node.getY() + node.getX()*node.getX() + 3.0*node.getY()*node.getZ() - node.getZ()*node.getZ();
        }

        //! Check that the model is a valid mesh of a unit cube made of elements of given order.
        static void verifyModel(const RModel &model, uint elementOrder, bool hasNeighbors = true, double tolerance = 1.0e-12)
        {
            QCOMPARE(model.findElementOrder(),elementOrder);
            QCOMPARE(model.getNVolumes(),uint(1));
            QCOMPARE(model.getNSurfaces(),uint(1));
            QCOMPARE(model.getNLines(),uint(2));
            // Nodes in the middle of edges must not become points.
            QCOMPARE(model.getNPoints(),uint(1));
            QCOMPARE(model.getPoint(0).size(),uint(1));

            const RElementType lineType = elementOrder == 2 ? R_ELEMENT_TRUSS2 : R_ELEMENT_TRUSS1;
            const RElementType surfaceType = elementOrder == 2 ? R_ELEMENT_TRI2 : R_ELEMENT_TRI1;
            const RElementType volumeType = elementOrder == 2 ? R_ELEMENT_TETRA2 : R_ELEMENT_TETRA1;

            double volume = 0.0, area = 0.0, length = 0.0;
            std::vector<bool> nodeUsed(model.getNNodes(),false);
            std::map<std::pair<uint,uint>,uint> edgeNodes;
            bool valid = true;

            for (uint i=0;i<model.getNElements();i++)
            {
                const RElement &rElement = model.getElement(i);
                const RElementType type = rElement.getType();
                if (R_ELEMENT_TYPE_IS_LINE(type))
                {
                    valid = valid && type == lineType;
                    double l = 0.0;
                    valid = valid && rElement.findLength(model.getNodes(),l);
                    length += l;
                }
                else if (R_ELEMENT_TYPE_IS_SURFACE(type))
                {
                    valid = valid && type == surfaceType;
                    double a = 0.0;
                    valid = valid && rElement.findArea(model.getNodes(),a);
                    area += a;
                }
                else if (R_ELEMENT_TYPE_IS_VOLUME(type))
                {
                    valid = valid && type == volumeType;
                    double v = 0.0;
                    valid = valid && rElement.findVolume(model.getNodes(),v) && v > 0.0;
                    volume += v;
                    // Neighbors share a whole face.
                    const std::vector<uint> *pNeighborIDs = model.getNeighborIDs(i);
                    valid = valid && (!hasNeighbors || (pNeighborIDs && pNeighborIDs->size() <= 4));
                    if (pNeighborIDs)
                    {
                        for (uint neighborID : *pNeighborIDs)
                        {
                            valid = valid && rElement.isNeighbor(model.getElement(neighborID));
                            valid = valid && rElement.findEdgePositionForNeighborElement(model.getElement(neighborID)) != RConstants::eod;
                        }
                    }
                }
                for (uint j=0;j<rElement.size();j++)
                {
                    nodeUsed[rElement.getNodeId(j)] = true;
                }
                const uint nc = RElement::getNCornerNodes(type);
                for (uint j=0;j<RElement::getNMidEdgeNodes(type);j++)
                {
                    uint c1, c2;
                    RElement::getMidEdgeNodeCorners(type,j,c1,c2);
                    const uint n1 = rElement.getNodeId(c1);
                    const uint n2 = rElement.getNodeId(c2);
                    const uint nm = rElement.getNodeId(nc+j);
                    const RNode &a = model.getNode(n1);
                    const RNode &b = model.getNode(n2);
                    const RNode middle(0.5*(a.getX()+b.getX()),0.5*(a.getY()+b.getY()),0.5*(a.getZ()+b.getZ()));
                    // The node lies in the middle of its edge and all elements at the edge share it.
                    valid = valid && model.getNode(nm).getDistance(middle) < tolerance;
                    auto entry = edgeNodes.emplace(std::make_pair(std::min(n1,n2),std::max(n1,n2)),nm);
                    valid = valid && entry.first->second == nm;
                }
            }
            QVERIFY(valid);
            QVERIFY(std::fabs(volume - 1.0) < 1.0e-9);
            QVERIFY(std::fabs(area - 6.0) < 1.0e-9);
            QVERIFY(std::fabs(length - 1.5) < 1.0e-9);
            // No node is left without an element.
            QVERIFY(std::find(nodeUsed.begin(),nodeUsed.end(),false) == nodeUsed.end());
            if (elementOrder == 2)
            {
                uint nCornerNodes = 0;
                const RUVector nodeBook = model.findCornerNodeBook(nCornerNodes);
                QCOMPARE(model.getNNodes(),nCornerNodes + uint(edgeNodes.size()));
                for (const auto &rEdge : edgeNodes)
                {
                    QCOMPARE(nodeBook[rEdge.second],RConstants::eod);
                }
            }
            else
            {
                QVERIFY(edgeNodes.empty());
            }
        }

    private slots:

        void meshInput();
        void generate_data();
        void generate();
        void sameCornersAsFirstOrder();
        void refine_data();
        void refine();
        void backToFirstOrder_data();
        void backToFirstOrder();
        void keepResults_data();
        void keepResults();
        void file();
};

static void addGenerators()
{
    QTest::addColumn<int>("generator");
    QTest::newRow("Range") << int(R_MESH_GENERATOR_RANGE);
    QTest::newRow("TetGen") << int(R_MESH_GENERATOR_TETGEN);
}

void TestMeshSecondOrder::meshInput()
{
    RMeshInput meshInput;
    QCOMPARE(meshInput.getElementOrder(),uint(1));
    const RModel model = createModel();
    QVERIFY(!model.generateMeshTetGenInputParams(meshInput).contains("o2"));

    meshInput.setElementOrder(2);
    const RMeshInput copy(meshInput);
    QCOMPARE(copy.getElementOrder(),uint(2));
    QVERIFY(model.generateMeshTetGenInputParams(meshInput).endsWith("o2"));

    // Only the first and the second order can be generated.
    meshInput.setElementOrder(0);
    QCOMPARE(meshInput.getElementOrder(),uint(1));
    meshInput.setElementOrder(3);
    QCOMPARE(meshInput.getElementOrder(),uint(2));
}

void TestMeshSecondOrder::generate_data()
{
    addGenerators();
}

void TestMeshSecondOrder::generate()
{
    QFETCH(int,generator);

    RModel model = createModel();
    QCOMPARE(model.findElementOrder(),uint(1));
    RMeshGenerator::generate(createMeshInput(RMeshGeneratorType(generator),0.01,2),model);
    verifyModel(model,2);
    QVERIFY(model.getNElements(R_ELEMENT_TETRA2) >= uint(100));
    QCOMPARE(model.getNElements(R_ELEMENT_TETRA1),uint(0));
    QCOMPARE(model.getNElements(R_ELEMENT_TRI1),uint(0));
    QCOMPARE(model.getNElements(R_ELEMENT_TRUSS1),uint(0));
}

void TestMeshSecondOrder::sameCornersAsFirstOrder()
{
    // The Range generator adds nodes to the middle of edges of the mesh it generates otherwise.
    RModel linear = createModel();
    RModel quadratic = createModel();
    RMeshGenerator::generate(createMeshInput(R_MESH_GENERATOR_RANGE,0.01,1),linear);
    RMeshGenerator::generate(createMeshInput(R_MESH_GENERATOR_RANGE,0.01,2),quadratic);
    verifyModel(linear,1);

    QCOMPARE(quadratic.getNElements(),linear.getNElements());
    QVERIFY(quadratic.getNNodes() > linear.getNNodes());
    for (uint i=0;i<linear.getNNodes();i++)
    {
        QVERIFY(quadratic.getNode(i).getDistance(linear.getNode(i)) == 0.0);
    }
    for (uint i=0;i<linear.getNElements();i++)
    {
        QCOMPARE(RElement::getLinearType(quadratic.getElement(i).getType()),linear.getElement(i).getType());
        for (uint j=0;j<linear.getElement(i).size();j++)
        {
            QCOMPARE(quadratic.getElement(i).getNodeId(j),linear.getElement(i).getNodeId(j));
        }
    }
}

void TestMeshSecondOrder::refine_data()
{
    addGenerators();
}

void TestMeshSecondOrder::refine()
{
    QFETCH(int,generator);

    // A second order mesh is reconstructed from its corner nodes.
    RModel model = createModel();
    RMeshGenerator::generate(createMeshInput(RMeshGeneratorType(generator),0.1,2),model);
    verifyModel(model,2);
    const uint nCoarseElements = model.getNElements(R_ELEMENT_TETRA2);

    RMeshInput meshInput = createMeshInput(RMeshGeneratorType(generator),0.001,2);
    meshInput.setReconstruct(true);
    RMeshGenerator::generate(meshInput,model);
    verifyModel(model,2);
    QVERIFY(model.getNElements(R_ELEMENT_TETRA2) > 10 * nCoarseElements);
}

void TestMeshSecondOrder::backToFirstOrder_data()
{
    addGenerators();
}

void TestMeshSecondOrder::backToFirstOrder()
{
    QFETCH(int,generator);

    RModel model = createModel();
    RMeshGenerator::generate(createMeshInput(RMeshGeneratorType(generator),0.01,2),model);
    verifyModel(model,2);
    uint nCornerNodes = 0;
    model.findCornerNodeBook(nCornerNodes);

    // Nodes in the middle of edges are not left in a first order mesh.
    RMeshInput meshInput = createMeshInput(RMeshGeneratorType(generator),0.01,1);
    meshInput.setReconstruct(true);
    RMeshGenerator::generate(meshInput,model);
    verifyModel(model,1);
    QVERIFY(model.getNNodes() < 2 * nCornerNodes);
}

void TestMeshSecondOrder::keepResults_data()
{
    addGenerators();
}

void TestMeshSecondOrder::keepResults()
{
    QFETCH(int,generator);

    RModel model = createModel();
    RMeshGenerator::generate(createMeshInput(RMeshGeneratorType(generator),0.01,2),model);

    RVariable &rVariable = model.getVariable(model.addVariable(RVariable(R_VARIABLE_TEMPERATURE,R_VARIABLE_APPLY_NODE)));
    rVariable.resize(1,model.getNNodes());
    for (uint i=0;i<model.getNNodes();i++)
    {
        rVariable.setValue(0,i,findQuadraticValue(model.getNode(i)));
    }

    RMeshInput meshInput = createMeshInput(RMeshGeneratorType(generator),0.001,2);
    meshInput.setReconstruct(true);
    meshInput.setKeepResults(true);
    RMeshGenerator::generate(meshInput,model);
    verifyModel(model,2);

    // Values are interpolated by shape functions of the second order, also to nodes in the middle of edges.
    const RVariable &rNewVariable = model.getVariable(model.findVariable(R_VARIABLE_TEMPERATURE));
    QCOMPARE(rNewVariable.getNValues(),model.getNNodes());
    uint nWrongNodes = 0;
    for (uint i=0;i<model.getNNodes();i++)
    {
        if (std::fabs(rNewVariable.getValue(0,i) - findQuadraticValue(model.getNode(i))) > 1.0e-6)
        {
            nWrongNodes++;
        }
    }
    QCOMPARE(nWrongNodes,uint(0));
}

void TestMeshSecondOrder::file()
{
    RModel model = createModel();
    RMeshGenerator::generate(createMeshInput(R_MESH_GENERATOR_RANGE,0.01,2),model);

    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    for (const QString &extension : {RModel::getDefaultFileExtension(true),RModel::getDefaultFileExtension(false)})
    {
        const QString fileName = directory.filePath("second_order." + extension);
        model.write(fileName,false);

        RModel loaded;
        loaded.read(fileName);
        // Neighbors are not a part of a file and a text file rounds coordinates.
        verifyModel(loaded,2,false,1.0e-6);
        QCOMPARE(loaded.getNNodes(),model.getNNodes());
        QCOMPARE(loaded.getNElements(),model.getNElements());
        for (uint i=0;i<model.getNElements();i++)
        {
            QCOMPARE(loaded.getElement(i).getType(),model.getElement(i).getType());
            for (uint j=0;j<model.getElement(i).size();j++)
            {
                QCOMPARE(loaded.getElement(i).getNodeId(j),model.getElement(i).getNodeId(j));
            }
        }
    }
}

QTEST_APPLESS_MAIN(TestMeshSecondOrder)
#include "tst_mesh_second_order.moc"
