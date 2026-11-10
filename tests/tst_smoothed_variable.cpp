#include <QTest>

#include "rml_model.h"

class TestSmoothedVariable : public QObject
{
    Q_OBJECT

    private:

        //! Two tetrahedra sharing nodes 1, 2 and 3 which are in two volumes.
        static RModel createModel()
        {
            RModel model;
            model.addNode(RNode(-1.0,0.0,0.0));
            model.addNode(RNode(0.0,0.0,0.0));
            model.addNode(RNode(0.0,1.0,0.0));
            model.addNode(RNode(0.0,0.0,1.0));
            model.addNode(RNode(1.0,0.0,0.0));

            model.addVolume(RVolume());
            model.addVolume(RVolume());

            RElement left(R_ELEMENT_TETRA1);
            left.setNodeId(0,0);
            left.setNodeId(1,1);
            left.setNodeId(2,2);
            left.setNodeId(3,3);
            model.addElement(left,true,model.getEntityGroupID(R_ENTITY_GROUP_VOLUME,0));

            RElement right(R_ELEMENT_TETRA1);
            right.setNodeId(0,4);
            right.setNodeId(1,1);
            right.setNodeId(2,2);
            right.setNodeId(3,3);
            model.addElement(right,true,model.getEntityGroupID(R_ENTITY_GROUP_VOLUME,1));

            return model;
        }

        //! Add variable with given values on elements.
        static uint addElementVariable(RModel &model, RVariableType type, const std::vector<double> &values)
        {
            const uint position = model.addVariable(RVariable(type,R_VARIABLE_APPLY_ELEMENT));
            RVariable &rVariable = model.getVariable(position);
            rVariable.resize(1,uint(values.size()));
            for (uint i=0;i<values.size();i++)
            {
                rVariable.setValue(0,i,values[i]);
            }
            return position;
        }

    private slots:

        void averagesSharedNodes();
        void keepsDifferentMaterialsApart();
        void invariantFromAveragedComponents();
        void surfaceWithoutThicknessFollowsVolume();
};

void TestSmoothedVariable::averagesSharedNodes()
{
    RModel model = createModel();
    const uint position = addElementVariable(model,R_VARIABLE_STRESS_X,{10.0,30.0});

    const RSmoothedVariable smoothed = model.findSmoothedVariable(position);

    // Node 0 belongs to the first element only, node 4 to the second one.
    QCOMPARE(smoothed.getValue(0,0),10.0);
    QCOMPARE(smoothed.getValue(1,0),30.0);
    // The elements are mirror images, so shared nodes get a plain average.
    for (uint i=1;i<4;i++)
    {
        QCOMPARE(smoothed.getValue(0,i),20.0);
        QCOMPARE(smoothed.getValue(1,i),20.0);
    }
    QCOMPARE(smoothed.elementMinValue,10.0);
    QCOMPARE(smoothed.elementMaxValue,30.0);
    QCOMPARE(smoothed.minValue,10.0);
    QCOMPARE(smoothed.maxValue,30.0);
}

void TestSmoothedVariable::keepsDifferentMaterialsApart()
{
    RModel model = createModel();
    model.getVolume(1).getMaterial().add(RMaterialProperty(RMaterialProperty::Density));
    const uint position = addElementVariable(model,R_VARIABLE_STRESS_X,{10.0,30.0});

    const RSmoothedVariable smoothed = model.findSmoothedVariable(position);

    for (uint i=0;i<4;i++)
    {
        QCOMPARE(smoothed.getValue(0,i),10.0);
        QCOMPARE(smoothed.getValue(1,i),30.0);
    }
}

void TestSmoothedVariable::invariantFromAveragedComponents()
{
    RModel model = createModel();
    addElementVariable(model,R_VARIABLE_STRESS_X,{100.0,-100.0});
    addElementVariable(model,R_VARIABLE_STRESS_Y,{0.0,0.0});
    addElementVariable(model,R_VARIABLE_STRESS_Z,{0.0,0.0});
    addElementVariable(model,R_VARIABLE_STRESS_YZ,{0.0,0.0});
    addElementVariable(model,R_VARIABLE_STRESS_XZ,{0.0,0.0});
    addElementVariable(model,R_VARIABLE_STRESS_XY,{0.0,0.0});
    const uint position = addElementVariable(model,R_VARIABLE_STRESS_VON_MISES,{100.0,100.0});

    const RSmoothedVariable smoothed = model.findSmoothedVariable(position);

    // Tension and compression cancel at shared nodes, averaged von Mises values would stay at 100.
    QCOMPARE(smoothed.getValue(0,0),100.0);
    QVERIFY(std::abs(smoothed.getValue(0,1)) < 1.0e-9);
    QCOMPARE(smoothed.elementMaxValue,100.0);
}

void TestSmoothedVariable::surfaceWithoutThicknessFollowsVolume()
{
    RModel model = createModel();
    model.addSurface(RSurface());

    RElement face(R_ELEMENT_TRI1);
    face.setNodeId(0,1);
    face.setNodeId(1,2);
    face.setNodeId(2,3);
    model.addElement(face,true,model.getEntityGroupID(R_ENTITY_GROUP_SURFACE,0));

    // Elements are renumbered by group, so find the triangle.
    uint faceID = RConstants::eod;
    for (uint i=0;i<model.getNElements();i++)
    {
        if (model.getElement(i).getType() == R_ELEMENT_TRI1)
        {
            faceID = i;
        }
    }
    QVERIFY(faceID != RConstants::eod);

    std::vector<double> values(model.getNElements(),10.0);
    values[faceID] = 0.0;
    const uint position = addElementVariable(model,R_VARIABLE_STRESS_X,values);

    const RSmoothedVariable smoothed = model.findSmoothedVariable(position);

    QCOMPARE(smoothed.getValue(faceID,0),10.0);
    QCOMPARE(smoothed.minValue,10.0);
}

QTEST_APPLESS_MAIN(TestSmoothedVariable)

#include "tst_smoothed_variable.moc"
