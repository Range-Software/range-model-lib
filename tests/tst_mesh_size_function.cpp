#include <QTest>

#include <cmath>

#include "rml_model.h"

class TestMeshSizeFunction : public QObject
{
    Q_OBJECT

    private:

        static RModel createChain(const std::vector<double> &positions)
        {
            RModel model;
            RLine line;
            line.setCrossArea(1.0);
            model.addLine(line);
            for (double x : positions)
            {
                model.addNode(RNode(x,0.0,0.0));
            }
            for (uint i=1;i<positions.size();i++)
            {
                RElement element(R_ELEMENT_TRUSS1);
                element.setNodeId(0,i-1);
                element.setNodeId(1,i);
                model.addElement(element,true,model.getEntityGroupID(R_ENTITY_GROUP_LINE,0));
            }
            return model;
        }

        static void addVariable(RModel &model, RVariableType type, RVariableApplyType apply,
                                const std::vector<std::vector<double>> &components)
        {
            const uint position = model.addVariable(RVariable(type,apply));
            RVariable &variable = model.getVariable(position);
            variable.resize(uint(components.size()),uint(components[0].size()));
            for (uint i=0;i<components.size();i++)
            {
                for (uint j=0;j<components[i].size();j++)
                {
                    variable.setValue(i,j,components[i][j]);
                }
            }
        }

        static RRVector sizes(const RModel &model, RVariableType type, double trim = 0.5)
        {
            return model.generateMeshSizeFunction(QSet<RVariableType>{type},0.1,1.0,trim);
        }

    private slots:

        void constantFieldsStayCoarse()
        {
            for (double value : {0.0,300.0,1e12})
            {
                RModel model = createChain({0,1,2});
                addVariable(model,R_VARIABLE_TEMPERATURE,R_VARIABLE_APPLY_NODE,{{value,value,value}});
                for (double size : sizes(model,R_VARIABLE_TEMPERATURE))
                {
                    QVERIFY(std::isfinite(size));
                    QCOMPARE(size,1.0);
                }
            }
        }

        void localizedGradient()
        {
            RModel model = createChain({0,1,2,3,4});
            addVariable(model,R_VARIABLE_TEMPERATURE,R_VARIABLE_APPLY_NODE,{{100,100,100.0001,101,101}});
            const RRVector field = sizes(model,R_VARIABLE_TEMPERATURE);
            QCOMPARE(field[0],1.0);
            QCOMPARE(field[1],1.0); // Negligible change beside a flat region.
            QVERIFY(std::abs(field[2]-0.1) < 1e-12);
            QVERIFY(std::abs(field[3]-0.1) < 1e-12);
            QCOMPARE(field[4],1.0);
        }

        void gradientAccountsForDistance()
        {
            RModel model = createChain({0,1,11});
            addVariable(model,R_VARIABLE_TEMPERATURE,R_VARIABLE_APPLY_NODE,{{0,1,11}});
            const RRVector field = sizes(model,R_VARIABLE_TEMPERATURE,0.0);
            // The same slope on a short and a long element needs the same size.
            QVERIFY(std::abs(field[0]-field[2]) < 1e-12);
            QVERIFY(std::abs(field[0]-0.1) < 1e-12);
        }

        void vectorDirectionChanges()
        {
            RModel model = createChain({0,1,2,3});
            addVariable(model,R_VARIABLE_VELOCITY,R_VARIABLE_APPLY_NODE,
                        {{1,1,0,0},{0,0,1,1},{0,0,0,0}});
            const RRVector field = sizes(model,R_VARIABLE_VELOCITY);
            // Magnitude is one everywhere, but direction changes in the middle.
            QCOMPARE(field[0],1.0);
            QVERIFY(std::abs(field[1]-0.1) < 1e-12);
            QVERIFY(std::abs(field[2]-0.1) < 1e-12);
            QCOMPARE(field[3],1.0);
        }

        void elementValues()
        {
            RModel model = createChain({0,1,2,3,4,5});
            addVariable(model,R_VARIABLE_TEMPERATURE,R_VARIABLE_APPLY_ELEMENT,{{10,10,20,20,20}});
            const RRVector field = sizes(model,R_VARIABLE_TEMPERATURE);
            QCOMPARE(field[0],1.0);
            QVERIFY(field[2] < field[0]);
            QCOMPARE(field[5],1.0);
        }

        void roundoffAndMissingVariables()
        {
            RModel model = createChain({0,1,2});
            addVariable(model,R_VARIABLE_TEMPERATURE,R_VARIABLE_APPLY_NODE,{{1e12,1e12+0.001,1e12}});
            for (double size : sizes(model,R_VARIABLE_TEMPERATURE))
            {
                QCOMPARE(size,1.0);
            }
            for (double size : sizes(model,R_VARIABLE_VELOCITY))
            {
                QCOMPARE(size,1.0);
            }
        }
};

QTEST_APPLESS_MAIN(TestMeshSizeFunction)

#include "tst_mesh_size_function.moc"
