#include <QTest>

#include <cmath>

#include "rml_mesh_generator.h"
#include "rml_model.h"

class TestMeshKeepResults : public QObject
{
    Q_OBJECT

    private:

        //! Node of a beam rotated by given angle around Z and Y axes.
        static RNode createNode(double x, double y, double z, double angle)
        {
            const double c = std::cos(angle);
            const double s = std::sin(angle);
            const double x1 = c*x - s*y;
            const double y1 = s*x + c*y;
            return RNode(c*x1 + s*z,y1,-s*x1 + c*z);
        }

        //! Volume mesh of a beam, as the solver would get it.
        static RModel createModel(double angle)
        {
            const double a = 1.0, b = 0.1, c = 0.04;

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

            RMeshInput meshInput;
            meshInput.setVerbose(false);
            meshInput.setKeepResults(false);
            meshInput.setReconstruct(false);
            meshInput.setVolumeConstraint(1.0e-4);
            RMeshGenerator::generate(meshInput,model);

            return model;
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

        //! Check that results follow the new mesh.
        static void verifyResults(const RModel &model, uint nOldNodes, uint nOldElements)
        {
            QVERIFY(model.getNNodes() > nOldNodes);
            QVERIFY(model.getNElements() > nOldElements);

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

            const RVariable &rElementVariable = model.getVariable(model.findVariable(R_VARIABLE_STRESS_VON_MISES));
            QCOMPARE(rElementVariable.getNValues(),model.getNElements());
            uint nWrongVolumeElements = 0;
            uint nWrongSurfaceElements = 0;
            for (uint i=0;i<model.getNElements();i++)
            {
                const bool isVolume = R_ELEMENT_TYPE_IS_VOLUME(model.getElement(i).getType());
                if (std::fabs(rElementVariable.getValue(0,i) - (isVolume ? 5.0 : 7.0)) > 1.0e-9)
                {
                    (isVolume ? nWrongVolumeElements : nWrongSurfaceElements)++;
                }
            }

            qInfo("nodes = %u (wrong %u), elements = %u (wrong volume %u, wrong surface %u)",
                  model.getNNodes(),nWrongNodes,model.getNElements(),nWrongVolumeElements,nWrongSurfaceElements);

            QCOMPARE(nWrongNodes,uint(0));
            QCOMPARE(nWrongVolumeElements,uint(0));
            QCOMPARE(nWrongSurfaceElements,uint(0));
        }

    private slots:

        void refine_data();
        void refine();
        void refineWithSizeFunction_data();
        void refineWithSizeFunction();
};

void TestMeshKeepResults::refine_data()
{
    QTest::addColumn<double>("angle");
    QTest::addColumn<double>("volumeConstraint");
    QTest::newRow("aligned with axes") << 0.0 << 1.0e-5;
    QTest::newRow("rotated") << 0.5 << 1.0e-5;
    QTest::newRow("fine, aligned with axes") << 0.0 << 2.0e-7;
    QTest::newRow("fine, rotated") << 0.5 << 2.0e-7;
}

void TestMeshKeepResults::refine()
{
    QFETCH(double,angle);
    QFETCH(double,volumeConstraint);

    RModel model = createModel(angle);
    addVariables(model);
    const uint nOldNodes = model.getNNodes();
    const uint nOldElements = model.getNElements();

    RMeshInput meshInput;
    meshInput.setVerbose(false);
    meshInput.setKeepResults(true);
    meshInput.setReconstruct(true);
    meshInput.setVolumeConstraint(volumeConstraint);
    RMeshGenerator::generate(meshInput,model);

    verifyResults(model,nOldNodes,nOldElements);
}

void TestMeshKeepResults::refineWithSizeFunction_data()
{
    this->refine_data();
}

void TestMeshKeepResults::refineWithSizeFunction()
{
    QFETCH(double,angle);
    QFETCH(double,volumeConstraint);

    RModel model = createModel(angle);
    addVariables(model);
    const uint nOldNodes = model.getNNodes();
    const uint nOldElements = model.getNElements();

    RMeshInput meshInput;
    meshInput.setVerbose(false);
    meshInput.setKeepResults(true);
    meshInput.setReconstruct(true);
    meshInput.setVolumeConstraint(1.0e-4);
    meshInput.setUseSizeFunction(true);
    meshInput.setSizeFunctionValues(RRVector(model.getNNodes(),std::cbrt(volumeConstraint)*2.0));
    RMeshGenerator::generate(meshInput,model);

    verifyResults(model,nOldNodes,nOldElements);
}

QTEST_APPLESS_MAIN(TestMeshKeepResults)

#include "tst_mesh_keep_results.moc"
