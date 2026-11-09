#include <QtTest>

#include "rml_problem.h"

namespace
{

//! Return all valid selections: required problem types are present and none is excluded.
std::vector<RProblemTypeMask> findValidMasks()
{
    const std::vector<RProblemType> allTypes = RProblem::getTypes(R_PROBLEM_ALL);

    std::vector<RProblemTypeMask> masks;
    for (unsigned int bits=1;bits<(1u << allTypes.size());bits++)
    {
        RProblemTypeMask mask = R_PROBLEM_NONE;
        for (std::size_t i=0;i<allTypes.size();i++)
        {
            if (bits & (1u << i))
            {
                mask |= allTypes[i];
            }
        }
        if ((RProblem::getRequiredProblemTypeMask(mask) & ~mask) == R_PROBLEM_NONE &&
            (RProblem::getExcludedProblemTypeMask(mask) & mask) == R_PROBLEM_NONE)
        {
            masks.push_back(mask);
        }
    }
    return masks;
}

std::vector<RProblemType> findChildTypes(const RProblemTaskItem &tree)
{
    std::vector<RProblemType> types;
    for (unsigned int i=0;i<tree.getNChildren();i++)
    {
        types.push_back(tree.getChild(i).getProblemType());
    }
    return types;
}

}

class TestProblemTaskItem : public QObject
{
    Q_OBJECT

private slots:

    void defaultTreeKeepsEverySelectedType();
    void defaultTreeSolvesPrerequisitesFirst();
    void defaultTreeHasDefaultLayout();
    void thermalStressChainOrder();
    void defaultIterations();
    void groupedTreeIsNotDefaultLayout();
    void reorderedTreeIsNotDefaultLayout();
    void iteratedLinearTaskIsNotDefaultLayout();
    void iterationCountOfFlowKeepsDefaultLayout();
};

void TestProblemTaskItem::defaultTreeKeepsEverySelectedType()
{
    for (RProblemTypeMask mask : findValidMasks())
    {
        const RProblemTaskItem tree = RProblemTaskItem::generateDefaultTree(mask);
        QCOMPARE(tree.getProblemTypeMask(),mask);
        QCOMPARE(std::size_t(tree.getNChildren()),RProblem::getTypes(mask).size());
    }
}

void TestProblemTaskItem::defaultTreeSolvesPrerequisitesFirst()
{
    for (RProblemTypeMask mask : findValidMasks())
    {
        const std::vector<RProblemType> types = findChildTypes(RProblemTaskItem::generateDefaultTree(mask));

        RProblemTypeMask solvedMask = R_PROBLEM_NONE;
        for (RProblemType type : types)
        {
            const RProblemTypeMask previousMask = (RProblem::getRequiredProblemTypeMask(type) | RProblem::getSolveAfterProblemTypeMask(type)) & mask;
            QCOMPARE(previousMask & ~solvedMask,int(R_PROBLEM_NONE));
            solvedMask |= type;
        }
    }
}

void TestProblemTaskItem::defaultTreeHasDefaultLayout()
{
    for (RProblemTypeMask mask : findValidMasks())
    {
        QVERIFY(RProblemTaskItem::generateDefaultTree(mask,false).hasDefaultLayout());
        QVERIFY(RProblemTaskItem::generateDefaultTree(mask,true).hasDefaultLayout());
    }
    QVERIFY(RProblemTaskItem().hasDefaultLayout());
}

void TestProblemTaskItem::thermalStressChainOrder()
{
    const std::vector<RProblemType> types = findChildTypes(RProblemTaskItem::generateDefaultTree(R_PROBLEM_STRESS | R_PROBLEM_HEAT | R_PROBLEM_ELECTROSTATICS));
    const std::vector<RProblemType> expected = {R_PROBLEM_ELECTROSTATICS, R_PROBLEM_HEAT, R_PROBLEM_STRESS};
    QVERIFY(types == expected);
}

void TestProblemTaskItem::defaultIterations()
{
    QCOMPARE(RProblemTaskItem::generateDefaultTree(R_PROBLEM_STRESS).getNIterations(),1u);
    QCOMPARE(RProblemTaskItem::generateDefaultTree(R_PROBLEM_HEAT | R_PROBLEM_STRESS).getNIterations(),1u);
    QCOMPARE(RProblemTaskItem::generateDefaultTree(R_PROBLEM_HEAT | R_PROBLEM_RADIATIVE_HEAT).getNIterations(),5u);
    QCOMPARE(RProblemTaskItem::generateDefaultTree(R_PROBLEM_FLUID,false).getNIterations(),2000u);
    QCOMPARE(RProblemTaskItem::generateDefaultTree(R_PROBLEM_FLUID,true).getNIterations(),20u);
}

void TestProblemTaskItem::groupedTreeIsNotDefaultLayout()
{
    RProblemTaskItem group;
    group.addChild(RProblemTaskItem(R_PROBLEM_FLUID));
    group.setNIterations(10);

    RProblemTaskItem tree;
    tree.addChild(group);
    tree.addChild(RProblemTaskItem(R_PROBLEM_FLUID_PARTICLE));

    QVERIFY(!tree.hasDefaultLayout());
}

void TestProblemTaskItem::reorderedTreeIsNotDefaultLayout()
{
    RProblemTaskItem tree;
    tree.addChild(RProblemTaskItem(R_PROBLEM_STRESS));
    tree.addChild(RProblemTaskItem(R_PROBLEM_HEAT));

    QVERIFY(!tree.hasDefaultLayout());
}

void TestProblemTaskItem::iteratedLinearTaskIsNotDefaultLayout()
{
    RProblemTaskItem tree = RProblemTaskItem::generateDefaultTree(R_PROBLEM_HEAT);
    tree.setNIterations(3);

    QVERIFY(!tree.hasDefaultLayout());
}

void TestProblemTaskItem::iterationCountOfFlowKeepsDefaultLayout()
{
    RProblemTaskItem tree = RProblemTaskItem::generateDefaultTree(R_PROBLEM_FLUID);
    tree.setNIterations(123);

    QVERIFY(tree.hasDefaultLayout());
}

QTEST_APPLESS_MAIN(TestProblemTaskItem)

#include "tst_problem_task_item.moc"
