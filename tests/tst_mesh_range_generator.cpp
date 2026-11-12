#include <QTest>

#include <cmath>

#include "rml_mesh_generator.h"
#include "rml_model.h"

//! Mesh generation with the Range mesh generator (range-element-lib).
class TestMeshRangeGenerator : public QObject
{
    Q_OBJECT

    private:

        //! Node rotated by given angle around Z and Y axes.
        static RNode createNode(double x, double y, double z, double angle)
        {
            const double c = std::cos(angle);
            const double s = std::sin(angle);
            const double x1 = c*x - s*y;
            const double y1 = s*x + c*y;
            return RNode(c*x1 + s*z,y1,-s*x1 + c*z);
        }

        //! Surface of a box with one surface entity.
        static RModel createBox(double a, double b, double c, double angle)
        {
            RModel model;
            model.addNode(createNode(0,0,0,angle));
            model.addNode(createNode(a,0,0,angle));
            model.addNode(createNode(a,b,0,angle));
            model.addNode(createNode(0,b,0,angle));
            model.addNode(createNode(0,0,c,angle));
            model.addNode(createNode(a,0,c,angle));
            model.addNode(createNode(a,b,c,angle));
            model.addNode(createNode(0,b,c,angle));

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
            return model;
        }

        //! Cube surface with points and lines on its corner, edges and inside.
        static RModel createModelWithPointsAndLines()
        {
            RModel model = createBox(1.0,1.0,1.0,0.0);
            model.addNode(RNode(0.5,0.5,0.25));
            model.addNode(RNode(0.25,0.5,0.5));
            model.addNode(RNode(0.75,0.5,0.5));

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

        static RMeshInput createMeshInput(double volumeConstraint)
        {
            RMeshInput meshInput;
            meshInput.setGeneratorType(R_MESH_GENERATOR_RANGE);
            meshInput.setVerbose(false);
            meshInput.setKeepResults(false);
            meshInput.setReconstruct(false);
            meshInput.setVolumeConstraint(volumeConstraint);
            return meshInput;
        }

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

        //! Sum of volumes of all volume elements.
        static double findVolume(const RModel &model)
        {
            double volume = 0.0;
            for (uint i=0;i<model.getNElements();i++)
            {
                const RElement &rElement = model.getElement(i);
                if (rElement.getType() != R_ELEMENT_TETRA1)
                {
                    continue;
                }
                const RNode &a = model.getNode(rElement.getNodeId(0));
                const RNode &b = model.getNode(rElement.getNodeId(1));
                const RNode &c = model.getNode(rElement.getNodeId(2));
                const RNode &d = model.getNode(rElement.getNodeId(3));
                const double u[3] = {b.getX()-a.getX(),b.getY()-a.getY(),b.getZ()-a.getZ()};
                const double v[3] = {c.getX()-a.getX(),c.getY()-a.getY(),c.getZ()-a.getZ()};
                const double w[3] = {d.getX()-a.getX(),d.getY()-a.getY(),d.getZ()-a.getZ()};
                volume += std::fabs(  u[0]*(v[1]*w[2]-v[2]*w[1])
                                    + u[1]*(v[2]*w[0]-v[0]*w[2])
                                    + u[2]*(v[0]*w[1]-v[1]*w[0])) / 6.0;
            }
            return volume;
        }

        //! Sum of areas of all surface elements.
        static double findArea(const RModel &model)
        {
            double area = 0.0;
            for (uint i=0;i<model.getNElements();i++)
            {
                const RElement &rElement = model.getElement(i);
                if (rElement.getType() != R_ELEMENT_TRI1)
                {
                    continue;
                }
                const RNode &a = model.getNode(rElement.getNodeId(0));
                const RNode &b = model.getNode(rElement.getNodeId(1));
                const RNode &c = model.getNode(rElement.getNodeId(2));
                const double u[3] = {b.getX()-a.getX(),b.getY()-a.getY(),b.getZ()-a.getZ()};
                const double v[3] = {c.getX()-a.getX(),c.getY()-a.getY(),c.getZ()-a.getZ()};
                area += 0.5 * std::hypot(u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]);
            }
            return area;
        }

        //! Check that points and lines are the same as those from createModelWithPointsAndLines().
        static void verifyEntities(const RModel &model)
        {
            QCOMPARE(model.getNVolumes(),uint(1));
            QCOMPARE(model.getNSurfaces(),uint(1));
            QVERIFY(std::fabs(findVolume(model) - 1.0) < 1.0e-9);
            QVERIFY(std::fabs(findArea(model) - 6.0) < 1.0e-9);

            // Added nodes must not become points.
            QCOMPARE(model.getNPoints(),uint(2));
            const RNode expectedPoints[2] = {RNode(0,0,0),RNode(0.5,0.5,0.25)};
            for (uint i=0;i<2;i++)
            {
                QCOMPARE(model.getPoint(i).size(),uint(1));
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
                const double length = expectedLines[i][0].getDistance(expectedLines[i][1]);
                QVERIFY(std::fabs(findLineLength(model,i) - length) < 1.0e-9);
                const RLine &rLine = model.getLine(i);
                for (uint j=0;j<rLine.size();j++)
                {
                    const RElement &rElement = model.getElement(rLine.get(j));
                    for (uint k=0;k<2;k++)
                    {
                        const RNode &rNode = model.getNode(rElement.getNodeId(k));
                        QVERIFY(std::fabs(rNode.getDistance(expectedLines[i][0]) + rNode.getDistance(expectedLines[i][1]) - length) < 1.0e-9);
                    }
                }
            }
        }

        //! Linear function which linear elements represent exactly.
        static double findLinearValue(const RNode &node)
        {
            return 1.0 + node.getX() + 2.0*node.getY() + 3.0*node.getZ();
        }

        //! Add variables: linear one on nodes, constant one on each kind of element.
        static void addVariables(RModel &model)
        {
            RVariable &rNodeVariable = model.getVariable(model.addVariable(RVariable(R_VARIABLE_TEMPERATURE,R_VARIABLE_APPLY_NODE)));
            rNodeVariable.resize(1,model.getNNodes());
            for (uint i=0;i<model.getNNodes();i++)
            {
                rNodeVariable.setValue(0,i,findLinearValue(model.getNode(i)));
            }

            RVariable &rElementVariable = model.getVariable(model.addVariable(RVariable(R_VARIABLE_STRESS_VON_MISES,R_VARIABLE_APPLY_ELEMENT)));
            rElementVariable.resize(1,model.getNElements());
            for (uint i=0;i<model.getNElements();i++)
            {
                rElementVariable.setValue(0,i,R_ELEMENT_TYPE_IS_VOLUME(model.getElement(i).getType()) ? 5.0 : 7.0);
            }
        }

    private slots:

        void generatorName();
        void generate_data();
        void generate();
        void noQuality();
        void refine();
        void refineWithSizeFunction();
        void keepResults_data();
        void keepResults();
        void sameVolumeAsTetGen();
};

void TestMeshRangeGenerator::generatorName()
{
    QCOMPARE(RMeshInput().getGeneratorType(),R_MESH_GENERATOR_TETGEN);
    QCOMPARE(RMeshInput::getGeneratorName(R_MESH_GENERATOR_TETGEN),QString("TetGen"));
    QCOMPARE(RMeshInput::getGeneratorName(R_MESH_GENERATOR_RANGE),QString("Range"));

    RMeshInput meshInput;
    meshInput.setGeneratorType(R_MESH_GENERATOR_RANGE);
    const RMeshInput copy(meshInput);
    QCOMPARE(copy.getGeneratorType(),R_MESH_GENERATOR_RANGE);
}

void TestMeshRangeGenerator::generate_data()
{
    QTest::addColumn<double>("volumeConstraint");
    QTest::newRow("coarse") << 0.1;
    QTest::newRow("fine") << 0.001;
}

void TestMeshRangeGenerator::generate()
{
    QFETCH(double,volumeConstraint);

    RModel model = createModelWithPointsAndLines();
    RMeshGenerator::generate(createMeshInput(volumeConstraint),model);
    verifyEntities(model);

    QVERIFY(model.getNElements(R_ELEMENT_TETRA1) >= uint(1.0 / volumeConstraint));
}

void TestMeshRangeGenerator::noQuality()
{
    RModel model = createModelWithPointsAndLines();
    RMeshInput meshInput = createMeshInput(0.1);
    meshInput.setQualityMesh(false);
    RMeshGenerator::generate(meshInput,model);
    verifyEntities(model);
}

void TestMeshRangeGenerator::refine()
{
    RModel model = createModelWithPointsAndLines();
    RMeshGenerator::generate(createMeshInput(0.1),model);
    const uint nCoarseElements = model.getNElements(R_ELEMENT_TETRA1);

    RMeshInput meshInput = createMeshInput(0.001);
    meshInput.setReconstruct(true);
    RMeshGenerator::generate(meshInput,model);
    verifyEntities(model);

    QVERIFY(model.getNElements(R_ELEMENT_TETRA1) > 10 * nCoarseElements);
}

void TestMeshRangeGenerator::refineWithSizeFunction()
{
    RModel model = createModelWithPointsAndLines();
    RMeshGenerator::generate(createMeshInput(0.1),model);
    const uint nCoarseElements = model.getNElements(R_ELEMENT_TETRA1);

    RMeshInput meshInput = createMeshInput(0.1);
    meshInput.setReconstruct(true);
    meshInput.setUseSizeFunction(true);
    meshInput.setSizeFunctionValues(RRVector(model.getNNodes(),0.1));
    RMeshGenerator::generate(meshInput,model);
    verifyEntities(model);

    QVERIFY(model.getNElements(R_ELEMENT_TETRA1) > 10 * nCoarseElements);
}

void TestMeshRangeGenerator::keepResults_data()
{
    QTest::addColumn<double>("angle");
    QTest::newRow("aligned with axes") << 0.0;
    QTest::newRow("rotated") << 0.5;
}

void TestMeshRangeGenerator::keepResults()
{
    QFETCH(double,angle);

    const double a = 1.0, b = 0.1, c = 0.04;

    RModel model = createBox(a,b,c,angle);
    RMeshGenerator::generate(createMeshInput(1.0e-4),model);
    QVERIFY(std::fabs(findVolume(model) - a*b*c) < 1.0e-12);

    addVariables(model);
    const uint nOldNodes = model.getNNodes();
    const uint nOldElements = model.getNElements();

    RMeshInput meshInput = createMeshInput(1.0e-5);
    meshInput.setReconstruct(true);
    meshInput.setKeepResults(true);
    RMeshGenerator::generate(meshInput,model);

    QVERIFY(model.getNNodes() > nOldNodes);
    QVERIFY(model.getNElements() > nOldElements);
    QVERIFY(std::fabs(findVolume(model) - a*b*c) < 1.0e-12);
    QVERIFY(std::fabs(findArea(model) - 2.0*(a*b + b*c + a*c)) < 1.0e-9);

    const RVariable &rNodeVariable = model.getVariable(model.findVariable(R_VARIABLE_TEMPERATURE));
    QCOMPARE(rNodeVariable.getNValues(),model.getNNodes());
    uint nWrongNodes = 0;
    for (uint i=0;i<model.getNNodes();i++)
    {
        if (std::fabs(rNodeVariable.getValue(0,i) - findLinearValue(model.getNode(i))) > 1.0e-6)
        {
            nWrongNodes++;
        }
    }
    QCOMPARE(nWrongNodes,uint(0));

    const RVariable &rElementVariable = model.getVariable(model.findVariable(R_VARIABLE_STRESS_VON_MISES));
    QCOMPARE(rElementVariable.getNValues(),model.getNElements());
    uint nWrongElements = 0;
    for (uint i=0;i<model.getNElements();i++)
    {
        const bool isVolume = R_ELEMENT_TYPE_IS_VOLUME(model.getElement(i).getType());
        if (std::fabs(rElementVariable.getValue(0,i) - (isVolume ? 5.0 : 7.0)) > 1.0e-9)
        {
            nWrongElements++;
        }
    }
    QCOMPARE(nWrongElements,uint(0));
}

void TestMeshRangeGenerator::sameVolumeAsTetGen()
{
    // Both generators have to fill the same volume and keep the same entities.
    RModel rangeModel = createModelWithPointsAndLines();
    RModel tetGenModel = createModelWithPointsAndLines();

    RMeshInput meshInput = createMeshInput(0.01);
    RMeshGenerator::generate(meshInput,rangeModel);
    meshInput.setGeneratorType(R_MESH_GENERATOR_TETGEN);
    RMeshGenerator::generate(meshInput,tetGenModel);

    QVERIFY(std::fabs(findVolume(rangeModel) - findVolume(tetGenModel)) < 1.0e-9);
    QVERIFY(std::fabs(findArea(rangeModel) - findArea(tetGenModel)) < 1.0e-9);
    QCOMPARE(rangeModel.getNPoints(),tetGenModel.getNPoints());
    QCOMPARE(rangeModel.getNLines(),tetGenModel.getNLines());
    QCOMPARE(rangeModel.getNSurfaces(),tetGenModel.getNSurfaces());
    QCOMPARE(rangeModel.getNVolumes(),tetGenModel.getNVolumes());
}

QTEST_APPLESS_MAIN(TestMeshRangeGenerator)
#include "tst_mesh_range_generator.moc"
