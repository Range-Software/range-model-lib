#include <QtTest>

#include <cmath>

#include "rml_mesh_generator.h"
#include "rml_model.h"

class TestMeshPointsLines : public QObject
{
    Q_OBJECT

    private:

        //! Cube surface with points and lines on its corner, edges and inside.
        static RModel createModel()
        {
            RModel model;
            model.addNode(RNode(0,0,0));
            model.addNode(RNode(1,0,0));
            model.addNode(RNode(1,1,0));
            model.addNode(RNode(0,1,0));
            model.addNode(RNode(0,0,1));
            model.addNode(RNode(1,0,1));
            model.addNode(RNode(1,1,1));
            model.addNode(RNode(0,1,1));
            model.addNode(RNode(0.5,0.5,0.25));
            model.addNode(RNode(0.25,0.5,0.5));
            model.addNode(RNode(0.75,0.5,0.5));

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

            // Point on a corner of the cube and a point inside of it.
            const uint points[2] = {0,8};
            for (uint i=0;i<2;i++)
            {
                model.addPoint(RPoint());
                RElement element(R_ELEMENT_POINT);
                element.setNodeId(0,points[i]);
                model.addElement(element,true,model.getEntityGroupID(R_ENTITY_GROUP_POINT,i));
            }

            // Lines on two edges of the cube and a line inside of it.
            const uint lines[3][2] = {{0,1},{2,3},{9,10}};
            for (uint i=0;i<3;i++)
            {
                model.addLine(RLine());
                RElement element(R_ELEMENT_TRUSS1);
                element.setNodeId(0,lines[i][0]);
                element.setNodeId(1,lines[i][1]);
                model.addElement(element,true,model.getEntityGroupID(R_ENTITY_GROUP_LINE,i));
            }

            return model;
        }

        //! Mesh input without any output.
        static RMeshInput createMeshInput(double volumeConstraint)
        {
            RMeshInput meshInput;
            meshInput.setVerbose(false);
            meshInput.setKeepResults(false);
            meshInput.setReconstruct(false);
            meshInput.setVolumeConstraint(volumeConstraint);
            return meshInput;
        }

        //! Length of a line.
        static double findLineLength(const RModel &model, uint lineID)
        {
            double length = 0.0;
            const RLine &rLine = model.getLine(lineID);
            for (uint i=0;i<rLine.size();i++)
            {
                const RElement &rElement = model.getElement(rLine.get(i));
                length += model.getNode(rElement.getNodeId(0)).getDistance(model.getNode(rElement.getNodeId(1)));
            }
            return length;
        }

        //! Check that each element of a line lies on the segment given by its end nodes.
        static uint findNOffLineNodes(const RModel &model, uint lineID, const RNode &node1, const RNode &node2)
        {
            uint nOffLine = 0;
            const double length = node1.getDistance(node2);
            const RLine &rLine = model.getLine(lineID);
            for (uint i=0;i<rLine.size();i++)
            {
                const RElement &rElement = model.getElement(rLine.get(i));
                for (uint j=0;j<2;j++)
                {
                    const RNode &rNode = model.getNode(rElement.getNodeId(j));
                    if (std::fabs(rNode.getDistance(node1) + rNode.getDistance(node2) - length) > 1.0e-9)
                    {
                        nOffLine++;
                    }
                }
            }
            return nOffLine;
        }

        //! Check that points and lines are the same as those from createModel().
        static void verifyEntities(const RModel &model)
        {
            QVERIFY(model.getNVolumes() > 0);

            // Steiner nodes must not become points.
            QCOMPARE(model.getNPoints(),uint(2));
            QCOMPARE(model.getPoint(0).size(),uint(1));
            QCOMPARE(model.getPoint(1).size(),uint(1));

            const RNode expectedPoints[2] = {RNode(0,0,0),RNode(0.5,0.5,0.25)};
            for (uint i=0;i<2;i++)
            {
                const RElement &rElement = model.getElement(model.getPoint(i).get(0));
                QCOMPARE(rElement.getType(),R_ELEMENT_POINT);
                QVERIFY(model.getNode(rElement.getNodeId(0)).getDistance(expectedPoints[i]) < 1.0e-9);
            }

            // Lines may get split, but must cover the same path.
            QCOMPARE(model.getNLines(),uint(3));
            const RNode expectedLines[3][2] = {{RNode(0,0,0),RNode(1,0,0)},
                                               {RNode(1,1,0),RNode(0,1,0)},
                                               {RNode(0.25,0.5,0.5),RNode(0.75,0.5,0.5)}};
            for (uint i=0;i<3;i++)
            {
                QVERIFY(model.getLine(i).size() > 0);
                QCOMPARE(findNOffLineNodes(model,i,expectedLines[i][0],expectedLines[i][1]),uint(0));
                QVERIFY(std::fabs(findLineLength(model,i) - expectedLines[i][0].getDistance(expectedLines[i][1])) < 1.0e-9);
            }
        }

    private slots:

        void generate_data();
        void generate();
        void noQuality();
        void refine();
        void refineWithSizeFunction();
        void keepResults();
        void customParameters();
};

void TestMeshPointsLines::generate_data()
{
    QTest::addColumn<double>("volumeConstraint");
    QTest::newRow("coarse") << 0.1;
    QTest::newRow("fine") << 0.001;
}

void TestMeshPointsLines::generate()
{
    QFETCH(double,volumeConstraint);

    RModel model = createModel();
    RMeshGenerator::generate(createMeshInput(volumeConstraint),model);
    verifyEntities(model);
}

void TestMeshPointsLines::noQuality()
{
    RModel model = createModel();
    RMeshInput meshInput = createMeshInput(0.1);
    meshInput.setQualityMesh(false);
    RMeshGenerator::generate(meshInput,model);
    verifyEntities(model);
}

void TestMeshPointsLines::refine()
{
    RModel model = createModel();
    RMeshGenerator::generate(createMeshInput(0.1),model);

    RMeshInput meshInput = createMeshInput(0.001);
    meshInput.setReconstruct(true);
    RMeshGenerator::generate(meshInput,model);
    verifyEntities(model);
}

void TestMeshPointsLines::refineWithSizeFunction()
{
    RModel model = createModel();
    RMeshGenerator::generate(createMeshInput(0.1),model);

    RMeshInput meshInput = createMeshInput(0.1);
    meshInput.setReconstruct(true);
    meshInput.setUseSizeFunction(true);
    meshInput.setSizeFunctionValues(RRVector(model.getNNodes(),0.1));
    RMeshGenerator::generate(meshInput,model);
    verifyEntities(model);
}

void TestMeshPointsLines::keepResults()
{
    RModel model = createModel();
    RMeshGenerator::generate(createMeshInput(0.1),model);

    RVariable &rVariable = model.getVariable(model.addVariable(RVariable(R_VARIABLE_STRESS_VON_MISES,R_VARIABLE_APPLY_ELEMENT)));
    rVariable.resize(1,model.getNElements());
    for (uint i=0;i<model.getNElements();i++)
    {
        const RElementType type = model.getElement(i).getType();
        rVariable.setValue(0,i,type == R_ELEMENT_POINT ? 1.0 : (type == R_ELEMENT_TRUSS1 ? 2.0 : (type == R_ELEMENT_TRI1 ? 3.0 : 4.0)));
    }

    RMeshInput meshInput = createMeshInput(0.001);
    meshInput.setReconstruct(true);
    meshInput.setKeepResults(true);
    RMeshGenerator::generate(meshInput,model);
    verifyEntities(model);

    const RVariable &rKeptVariable = model.getVariable(model.findVariable(R_VARIABLE_STRESS_VON_MISES));
    QCOMPARE(rKeptVariable.getNValues(),model.getNElements());
    uint nWrong = 0;
    for (uint i=0;i<model.getNElements();i++)
    {
        const RElementType type = model.getElement(i).getType();
        const double expected = type == R_ELEMENT_POINT ? 1.0 : (type == R_ELEMENT_TRUSS1 ? 2.0 : (type == R_ELEMENT_TRI1 ? 3.0 : 4.0));
        if (std::fabs(rKeptVariable.getValue(0,i) - expected) > 1.0e-9)
        {
            nWrong++;
        }
    }
    QCOMPARE(nWrong,uint(0));
}

void TestMeshPointsLines::customParameters()
{
    RModel model = createModel();
    RMeshInput meshInput = createMeshInput(0.1);
    meshInput.setUseTetGenInputParams(true);
    meshInput.setTetGenInputParams("npAQq1.6a0.1");
    RMeshGenerator::generate(meshInput,model);
    verifyEntities(model);
}

QTEST_APPLESS_MAIN(TestMeshPointsLines)

#include "tst_mesh_points_lines.moc"
