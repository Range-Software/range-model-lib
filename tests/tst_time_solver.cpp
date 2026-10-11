#include <QtTest>

#include "rml_time_solver.h"

namespace
{
constexpr double tol = 1.0e-12;

RTimeSolver makeTimeSolver(double startTime, uint nTimeSteps, double timeStepSize)
{
    RTimeSolver timeSolver;
    timeSolver.setEnabled(true);
    timeSolver.setInputStartTime(startTime);
    timeSolver.setInputNTimeSteps(nTimeSteps);
    timeSolver.setInputTimeStepSize(timeStepSize);
    return timeSolver;
}
}

class TestTimeSolver : public QObject
{
    Q_OBJECT

private slots:

    void freshRunStartsAfterStartTime();
    void restartFromSteadyStartsAtStartTime();
    void restartFromSteadyIsRepeatable();
    void restartContinuesTransient();
};

//! Fresh run: time steps start + (i+1) * dt.
void TestTimeSolver::freshRunStartsAfterStartTime()
{
    RTimeSolver timeSolver = makeTimeSolver(2.0,4,0.5);
    timeSolver.harmonizeTimesWithInput(false);

    QCOMPARE(timeSolver.getNTimeSteps(),4u);
    for (uint i=0;i<4;i++)
    {
        QVERIFY(std::fabs(timeSolver.getTime(i) - (2.0 + (i+1)*0.5)) < tol);
    }
}

//! Restart from steady solution: the default (placeholder) time of a steady model must not
//! shift the transient. Steady state is at the start time, the first step at start + dt.
void TestTimeSolver::restartFromSteadyStartsAtStartTime()
{
    RTimeSolver timeSolver = makeTimeSolver(0.0,5,0.01);
    // Steady model keeps the default time vector.
    QVERIFY(timeSolver.getNTimeSteps() == 1 && timeSolver.getTime(0) != 0.0);

    timeSolver.harmonizeTimesWithInput(true);

    QCOMPARE(timeSolver.getNTimeSteps(),6u);
    QCOMPARE(timeSolver.getCurrentTimeStep(),0u);
    for (uint i=0;i<6;i++)
    {
        QVERIFY2(std::fabs(timeSolver.getTime(i) - i*0.01) < tol,
                 qPrintable(QString("Time %1: %2, expected %3").arg(i).arg(timeSolver.getTime(i)).arg(i*0.01)));
    }

    timeSolver.setNextTimeStep();
    QVERIFY(std::fabs(timeSolver.getCurrentTime() - 0.01) < tol);
    QVERIFY(std::fabs(timeSolver.getCurrentTimeStepSize() - 0.01) < tol);
}

//! Application harmonizes times when the solver is started and the solver again when it runs.
void TestTimeSolver::restartFromSteadyIsRepeatable()
{
    RTimeSolver timeSolver = makeTimeSolver(1.0,3,0.1);
    timeSolver.harmonizeTimesWithInput(true);
    std::vector<double> times = timeSolver.getTimes();
    timeSolver.harmonizeTimesWithInput(true);

    QCOMPARE(timeSolver.getNTimeSteps(),uint(times.size()));
    for (uint i=0;i<times.size();i++)
    {
        QVERIFY(std::fabs(timeSolver.getTime(i) - times[i]) < tol);
    }
    QVERIFY(std::fabs(timeSolver.getTime(0) - 1.0) < tol);
}

//! Restart of a transient run continues from the current time step regardless of start time.
void TestTimeSolver::restartContinuesTransient()
{
    RTimeSolver timeSolver = makeTimeSolver(0.0,5,0.01);
    timeSolver.harmonizeTimesWithInput(false);
    timeSolver.setCurrentTimeStep(2);
    timeSolver.setComputedTime(timeSolver.getCurrentTime());

    timeSolver.setInputNTimeSteps(3);
    timeSolver.setInputTimeStepSize(0.1);
    timeSolver.harmonizeTimesWithInput(true);

    const std::vector<double> expected = { 0.01, 0.02, 0.03, 0.13, 0.23, 0.33 };
    QCOMPARE(timeSolver.getNTimeSteps(),uint(expected.size()));
    for (uint i=0;i<expected.size();i++)
    {
        QVERIFY2(std::fabs(timeSolver.getTime(i) - expected[i]) < tol,
                 qPrintable(QString("Time %1: %2, expected %3").arg(i).arg(timeSolver.getTime(i)).arg(expected[i])));
    }
    QCOMPARE(timeSolver.getCurrentTimeStep(),2u);
}

QTEST_APPLESS_MAIN(TestTimeSolver)

#include "tst_time_solver.moc"
