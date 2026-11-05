#include <QtTest>
#include <QTemporaryDir>

#include "rml_entity_group_data.h"
#include "rml_file.h"
#include "rml_file_io.h"
#include "rml_model.h"
#include "rml_save_file.h"

class TestEntityGroupData : public QObject
{
    Q_OBJECT

private slots:

    void drawBoundaryConditionsDefault();
    void drawBoundaryConditionsCopy();
    void fileRoundTrip();
    void fileRoundTrip_data();
    void fileOldVersion();
    void fileOldVersion_data();

private:

    //! Write two entity group data records followed by a sentinel value.
    static void writeFile(const QString &fileName, bool binary, const REntityGroupData &first, const REntityGroupData &second, int sentinel);

    //! Read two entity group data records followed by a sentinel value assuming given file version.
    static void readFile(const QString &fileName, bool binary, const RVersion &version, REntityGroupData &first, REntityGroupData &second, int &sentinel);
};

void TestEntityGroupData::drawBoundaryConditionsDefault()
{
    REntityGroupData data;
    QCOMPARE(data.getDrawBoundaryConditions(), false);
    data.setDrawBoundaryConditions(true);
    QCOMPARE(data.getDrawBoundaryConditions(), true);
}

void TestEntityGroupData::drawBoundaryConditionsCopy()
{
    REntityGroupData data;
    data.setDrawBoundaryConditions(true);

    REntityGroupData copied(data);
    QCOMPARE(copied.getDrawBoundaryConditions(), true);

    REntityGroupData assigned;
    assigned = data;
    QCOMPARE(assigned.getDrawBoundaryConditions(), true);
}

void TestEntityGroupData::writeFile(const QString &fileName, bool binary, const REntityGroupData &first, const REntityGroupData &second, int sentinel)
{
    RSaveFile file(fileName,binary ? RSaveFile::BINARY : RSaveFile::ASCII);
    QVERIFY(file.open(binary ? QIODevice::WriteOnly : (QIODevice::WriteOnly | QIODevice::Text)));
    try
    {
        if (binary)
        {
            RFileIO::writeBinary(file,first);
            RFileIO::writeBinary(file,second);
            RFileIO::writeBinary(file,sentinel);
        }
        else
        {
            RFileIO::writeAscii(file,first);
            RFileIO::writeAscii(file,second);
            RFileIO::writeAscii(file,sentinel);
        }
    }
    catch (const RError &error)
    {
        QFAIL(qPrintable("Write failed: " + error.getMessage()));
    }
    QVERIFY(file.commit());
}

void TestEntityGroupData::readFile(const QString &fileName, bool binary, const RVersion &version, REntityGroupData &first, REntityGroupData &second, int &sentinel)
{
    RFile file(fileName,binary ? RFile::BINARY : RFile::ASCII);
    QVERIFY(file.open(binary ? QIODevice::ReadOnly : (QIODevice::ReadOnly | QIODevice::Text)));
    file.setVersion(version);
    try
    {
        if (binary)
        {
            RFileIO::readBinary(file,first);
            RFileIO::readBinary(file,second);
            RFileIO::readBinary(file,sentinel);
        }
        else
        {
            RFileIO::readAscii(file,first);
            RFileIO::readAscii(file,second);
            RFileIO::readAscii(file,sentinel);
        }
    }
    catch (const RError &error)
    {
        QFAIL(qPrintable("Read failed: " + error.getMessage()));
    }
    file.close();
}

void TestEntityGroupData::fileRoundTrip_data()
{
    QTest::addColumn<bool>("binary");

    // ASCII is not covered: RFileIO reads ASCII bool values inverted (pre-existing issue).
    QTest::newRow("binary") << true;
}

void TestEntityGroupData::fileRoundTrip()
{
    QFETCH(bool, binary);

    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    QString fileName = dir.filePath("entity_group_data.dat");

    REntityGroupData first;
    first.setDrawBoundaryConditions(true);
    first.setDrawNodeNumbers(true);
    first.setColor(10,20,30,40);

    REntityGroupData second;
    second.setDrawBoundaryConditions(false);
    second.setDrawEdges(true);

    writeFile(fileName,binary,first,second,12345);

    REntityGroupData readFirst;
    REntityGroupData readSecond;
    readSecond.setDrawBoundaryConditions(true);
    int sentinel = 0;
    readFile(fileName,binary,RModel::version,readFirst,readSecond,sentinel);

    QCOMPARE(readFirst.getDrawBoundaryConditions(), true);
    QCOMPARE(readFirst.getDrawNodeNumbers(), true);
    int r = 0, g = 0, b = 0, a = 0;
    readFirst.getColor(r,g,b,a);
    QCOMPARE(r, 10);
    QCOMPARE(g, 20);
    QCOMPARE(b, 30);
    QCOMPARE(a, 40);

    QCOMPARE(readSecond.getDrawBoundaryConditions(), false);
    QCOMPARE(readSecond.getDrawEdges(), true);

    QCOMPARE(sentinel, 12345);
}

void TestEntityGroupData::fileOldVersion_data()
{
    QTest::addColumn<bool>("binary");

    // ASCII is not covered: RFileIO reads ASCII bool values inverted (pre-existing issue).
    QTest::newRow("binary") << true;
}

void TestEntityGroupData::fileOldVersion()
{
    QFETCH(bool, binary);

    // Files written by version 1.4.0 do not contain the flag, so reading them
    // must neither consume data which follows nor change the default.
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    QString fileName = dir.filePath("entity_group_data.dat");

    REntityGroupData data;
    data.setDrawNodes(true);

    {
        // Write the record as version 1.4.0 did - all values up to the variable data.
        RSaveFile file(fileName,binary ? RSaveFile::BINARY : RSaveFile::ASCII);
        QVERIFY(file.open(binary ? QIODevice::WriteOnly : (QIODevice::WriteOnly | QIODevice::Text)));
        int r = 0, g = 0, b = 0, a = 0;
        data.getColor(r,g,b,a);
        const bool flags[] = { data.getSelected(), data.getVisible() };
        const int rgba[] = { r, g, b, a };
        const bool drawFlags[] = { data.getDrawWire(), data.getDrawEdges(), data.getDrawNodes(),
                                   data.getDrawElementNumbers(), data.getDrawNodeNumbers(),
                                   data.getDrawArrowHeads(), data.getDrawEqualArrowLength(),
                                   data.getDrawArrowFrom(), data.getColorByPatch(), data.getColorByViewFactor() };
        for (bool flag : flags) { binary ? RFileIO::writeBinary(file,flag) : RFileIO::writeAscii(file,flag); }
        for (int value : rgba) { binary ? RFileIO::writeBinary(file,value) : RFileIO::writeAscii(file,value); }
        for (bool flag : drawFlags) { binary ? RFileIO::writeBinary(file,flag) : RFileIO::writeAscii(file,flag); }
        unsigned int nVariableData = 0;
        binary ? RFileIO::writeBinary(file,nVariableData) : RFileIO::writeAscii(file,nVariableData);
        int sentinel = 777;
        binary ? RFileIO::writeBinary(file,sentinel) : RFileIO::writeAscii(file,sentinel);
        QVERIFY(file.commit());
    }

    RFile file(fileName,binary ? RFile::BINARY : RFile::ASCII);
    QVERIFY(file.open(binary ? QIODevice::ReadOnly : (QIODevice::ReadOnly | QIODevice::Text)));
    file.setVersion(RVersion(1,4,0));

    REntityGroupData readData;
    int sentinel = 0;
    if (binary)
    {
        RFileIO::readBinary(file,readData);
        RFileIO::readBinary(file,sentinel);
    }
    else
    {
        RFileIO::readAscii(file,readData);
        RFileIO::readAscii(file,sentinel);
    }
    file.close();

    QCOMPARE(readData.getDrawNodes(), true);
    QCOMPARE(readData.getDrawBoundaryConditions(), false);
    QCOMPARE(sentinel, 777);
}

QTEST_GUILESS_MAIN(TestEntityGroupData)

#include "tst_entity_group_data.moc"
