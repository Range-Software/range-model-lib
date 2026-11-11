#include <QTest>
#include <QTemporaryDir>

#include <rbl_error.h>

#include "rml_file_header.h"
#include "rml_file_io.h"
#include "rml_file_utils.h"
#include "rml_model.h"
#include "rml_save_file.h"

class TestModelFileVersion : public QObject
{
    Q_OBJECT

private slots:

    void readVersion();
    void readVersion_data();
    void followLink();
    void followLink_data();
    void wrongFileType();
    void unknownExtension();
    void missingFile();
    void linkCycle();
    void supportReactionFileIO_data();
    void supportReactionFileIO();
    void supportReactionsRoundTrip_data();
    void supportReactionsRoundTrip();
    void fileWithoutSupportReactions_data();
    void fileWithoutSupportReactions();
    void clearingResultsClearsSupportReactions();

private:

    //! Write file containing only the file header.
    static void writeHeader(const QString &fileName, bool binary, const RFileHeader &header);

    //! Return model file name with extension for given format.
    static QString modelFileName(const QTemporaryDir &dir, const QString &baseName, bool binary);
};

void TestModelFileVersion::writeHeader(const QString &fileName, bool binary, const RFileHeader &header)
{
    RSaveFile file(fileName,binary ? RSaveFile::BINARY : RSaveFile::ASCII);
    QVERIFY(file.open(binary ? QIODevice::WriteOnly : (QIODevice::WriteOnly | QIODevice::Text)));
    if (binary)
    {
        RFileIO::writeBinary(file,header);
    }
    else
    {
        RFileIO::writeAscii(file,header);
    }
    QVERIFY(file.commit());
}

QString TestModelFileVersion::modelFileName(const QTemporaryDir &dir, const QString &baseName, bool binary)
{
    return dir.filePath(baseName + "." + RModel::getDefaultFileExtension(binary));
}

void TestModelFileVersion::readVersion_data()
{
    QTest::addColumn<bool>("binary");
    QTest::addColumn<RVersion>("version");
    QTest::addColumn<bool>("newer");

    const RVersion current(RModel::version);
    const RVersion newerRelease(current.getMajor(),current.getMinor(),current.getRelease()+1);
    const RVersion newerMajor(current.getMajor()+1,0,0);
    const RVersion older(1,0,0);

    for (bool binary : {true, false})
    {
        const char *format = binary ? "binary" : "ascii";
        QTest::newRow(qPrintable(QString("%1 current").arg(format))) << binary << current << false;
        QTest::newRow(qPrintable(QString("%1 older").arg(format))) << binary << older << false;
        QTest::newRow(qPrintable(QString("%1 newer release").arg(format))) << binary << newerRelease << true;
        QTest::newRow(qPrintable(QString("%1 newer major").arg(format))) << binary << newerMajor << true;
    }
}

void TestModelFileVersion::readVersion()
{
    QFETCH(bool, binary);
    QFETCH(RVersion, version);
    QFETCH(bool, newer);

    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    QString fileName = modelFileName(dir,"model",binary);

    writeHeader(fileName,binary,RFileHeader(R_FILE_TYPE_MODEL,version));

    RVersion readVersion;
    try
    {
        readVersion = RModel::readFileVersion(fileName);
    }
    catch (const RError &error)
    {
        QFAIL(qPrintable("Read failed: " + error.getMessage()));
    }

    QCOMPARE(readVersion.toString(), version.toString());
    QCOMPARE(readVersion > RModel::version, newer);
}

void TestModelFileVersion::followLink_data()
{
    QTest::addColumn<bool>("binary");

    QTest::newRow("binary") << true;
    QTest::newRow("ascii") << false;
}

void TestModelFileVersion::followLink()
{
    QFETCH(bool, binary);

    QTemporaryDir dir;
    QVERIFY(dir.isValid());

    // The link has the current version, the model it points to a newer one.
    const RVersion newer(RModel::version.getMajor(),RModel::version.getMinor(),RModel::version.getRelease()+1);
    QString targetFileName = modelFileName(dir,"target",binary);
    writeHeader(targetFileName,binary,RFileHeader(R_FILE_TYPE_MODEL,newer));

    QString linkFileName = modelFileName(dir,"link",binary);
    RFileUtils::writeLink(linkFileName,QFileInfo(targetFileName).fileName(),RModel::version,binary);

    RVersion readVersion;
    try
    {
        readVersion = RModel::readFileVersion(linkFileName);
    }
    catch (const RError &error)
    {
        QFAIL(qPrintable("Read failed: " + error.getMessage()));
    }

    QCOMPARE(readVersion.toString(), newer.toString());
}

void TestModelFileVersion::wrongFileType()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    QString fileName = modelFileName(dir,"material",true);

    writeHeader(fileName,true,RFileHeader(R_FILE_TYPE_MATERIAL,RModel::version));

    QVERIFY_THROWS_EXCEPTION(RError, RModel::readFileVersion(fileName));
}

void TestModelFileVersion::unknownExtension()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    QString fileName = dir.filePath("model.stl");

    writeHeader(fileName,true,RFileHeader(R_FILE_TYPE_MODEL,RModel::version));

    QVERIFY_THROWS_EXCEPTION(RError, RModel::readFileVersion(fileName));
}

void TestModelFileVersion::missingFile()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());

    QVERIFY_THROWS_EXCEPTION(RError, RModel::readFileVersion(modelFileName(dir,"missing",true)));
}

void TestModelFileVersion::linkCycle()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());

    QString linkFileName = modelFileName(dir,"self",true);
    RFileUtils::writeLink(linkFileName,QFileInfo(linkFileName).fileName(),RModel::version,true);

    QVERIFY_THROWS_EXCEPTION(RError, RModel::readFileVersion(linkFileName));
}

void TestModelFileVersion::supportReactionFileIO_data()
{
    QTest::addColumn<bool>("binary");

    QTest::newRow("binary") << true;
    QTest::newRow("ascii") << false;
}

//! A support reaction record, name with spaces included, survives both formats.
void TestModelFileVersion::supportReactionFileIO()
{
    QFETCH(bool, binary);

    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    QString fileName = dir.filePath("reaction.dat");

    // Values with at most six significant digits survive the ASCII format.
    RSupportReaction written(R_ENTITY_GROUP_SURFACE,"Mid span roller",RR3Vector(-1000.0,-2.5e-7,3.25e6));
    RSupportReaction second(R_ENTITY_GROUP_POINT,"Fixed end",RR3Vector(1.5,0.0,-7.0));
    {
        RSaveFile file(fileName,binary ? RSaveFile::BINARY : RSaveFile::ASCII);
        QVERIFY(file.open(binary ? QIODevice::WriteOnly : (QIODevice::WriteOnly | QIODevice::Text)));
        if (binary)
        {
            RFileIO::writeBinary(file,written);
            RFileIO::writeBinary(file,second);
        }
        else
        {
            RFileIO::writeAscii(file,written);
            RFileIO::writeAscii(file,second);
        }
        QVERIFY(file.commit());
    }

    RSupportReaction read, readSecond;
    {
        RFile file(fileName,binary ? RFile::BINARY : RFile::ASCII);
        QVERIFY(file.open(binary ? QIODevice::ReadOnly : (QIODevice::ReadOnly | QIODevice::Text)));
        try
        {
            if (binary)
            {
                RFileIO::readBinary(file,read);
                RFileIO::readBinary(file,readSecond);
            }
            else
            {
                RFileIO::readAscii(file,read);
                RFileIO::readAscii(file,readSecond);
            }
        }
        catch (const RError &error)
        {
            QFAIL(qPrintable("Read failed: " + error.getMessage()));
        }
    }

    for (const auto &pair : {std::make_pair(written,read),std::make_pair(second,readSecond)})
    {
        QCOMPARE(pair.second.getEntityType(), pair.first.getEntityType());
        QCOMPARE(pair.second.getName(), pair.first.getName());
        for (uint c=0;c<3;c++)
        {
            QCOMPARE(pair.second.getForce()[c], pair.first.getForce()[c]);
        }
    }
}

void TestModelFileVersion::supportReactionsRoundTrip_data()
{
    QTest::addColumn<bool>("binary");

    QTest::newRow("binary") << true;
    QTest::newRow("ascii") << false;
}

//! Support reactions are saved with the results of a model and read back
//! unchanged.
void TestModelFileVersion::supportReactionsRoundTrip()
{
    QFETCH(bool, binary);

    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    QString fileName = modelFileName(dir,"reactions",binary);

    // Values with at most six significant digits survive the ASCII format.
    std::vector<RSupportReaction> supportReactions;
    supportReactions.push_back(RSupportReaction(R_ENTITY_GROUP_POINT,"Fixed end",RR3Vector(-1000.0,0.5,1569.5)));
    supportReactions.push_back(RSupportReaction(R_ENTITY_GROUP_SURFACE,"Mid span roller",RR3Vector(0.0,-2.5e-7,3.25e6)));
    RR3Vector appliedLoad(1000.0,-0.5,-1569.5);

    RModel model;
    model.setProblemTaskTree(RProblemTaskItem(R_PROBLEM_STRESS));
    model.setSupportReactions(supportReactions,appliedLoad);
    try
    {
        model.write(fileName,false);
    }
    catch (const RError &error)
    {
        QFAIL(qPrintable("Write failed: " + error.getMessage()));
    }

    RModel readModel;
    try
    {
        readModel.read(fileName);
    }
    catch (const RError &error)
    {
        QFAIL(qPrintable("Read failed: " + error.getMessage()));
    }

    const std::vector<RSupportReaction> &readReactions = readModel.getSupportReactions();
    QCOMPARE(readReactions.size(), supportReactions.size());
    for (size_t i=0;i<supportReactions.size();i++)
    {
        QCOMPARE(readReactions[i].getEntityType(), supportReactions[i].getEntityType());
        QCOMPARE(readReactions[i].getName(), supportReactions[i].getName());
        for (uint c=0;c<3;c++)
        {
            QCOMPARE(readReactions[i].getForce()[c], supportReactions[i].getForce()[c]);
        }
    }
    for (uint c=0;c<3;c++)
    {
        QCOMPARE(readModel.getAppliedLoad()[c], appliedLoad[c]);
    }
}

void TestModelFileVersion::fileWithoutSupportReactions_data()
{
    QTest::addColumn<bool>("binary");

    QTest::newRow("binary") << true;
    QTest::newRow("ascii") << false;
}

//! Support reactions are the last part of a 1.5.0 model file, so a file saved
//! before they were added, which ends right before them, reads without any.
void TestModelFileVersion::fileWithoutSupportReactions()
{
    QFETCH(bool, binary);

    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    QString fileName = modelFileName(dir,"old",binary);

    RModel model;
    model.setProblemTaskTree(RProblemTaskItem(R_PROBLEM_STRESS));
    try
    {
        model.write(fileName,false);
    }
    catch (const RError &error)
    {
        QFAIL(qPrintable("Write failed: " + error.getMessage()));
    }

    // The support reaction part of a model without them, written on its own.
    QString trailerFileName = dir.filePath("trailer.dat");
    {
        RSaveFile file(trailerFileName,binary ? RSaveFile::BINARY : RSaveFile::ASCII);
        QVERIFY(file.open(binary ? QIODevice::WriteOnly : (QIODevice::WriteOnly | QIODevice::Text)));
        if (binary)
        {
            RFileIO::writeBinary(file,uint(0));
            RFileIO::writeBinary(file,RR3Vector(0.0,0.0,0.0));
        }
        else
        {
            RFileIO::writeAscii(file,uint(0));
            RFileIO::writeAscii(file,RR3Vector(0.0,0.0,0.0));
        }
        QVERIFY(file.commit());
    }

    QByteArray content, trailer;
    {
        QFile file(fileName);
        QVERIFY(file.open(QIODevice::ReadOnly));
        content = file.readAll();
        QFile trailerFile(trailerFileName);
        QVERIFY(trailerFile.open(QIODevice::ReadOnly));
        trailer = trailerFile.readAll();
    }
    QVERIFY(!trailer.isEmpty());
    QVERIFY2(content.endsWith(trailer),"Support reactions are not at the end of the model file");

    // Cut them off, as in a file saved before support reactions were added.
    {
        QFile file(fileName);
        QVERIFY(file.open(QIODevice::WriteOnly | QIODevice::Truncate));
        QCOMPARE(file.write(content.left(content.size() - trailer.size())), content.size() - trailer.size());
    }

    RModel readModel;
    readModel.setSupportReactions({RSupportReaction(R_ENTITY_GROUP_POINT,"Stale",RR3Vector(1.0,2.0,3.0))},RR3Vector(-1.0,-2.0,-3.0));
    try
    {
        readModel.read(fileName);
    }
    catch (const RError &error)
    {
        QFAIL(qPrintable("Read failed: " + error.getMessage()));
    }
    QVERIFY(readModel.getSupportReactions().empty());
    QCOMPARE(readModel.getAppliedLoad().length(), 0.0);
}

//! Support reactions are results, so removing the results removes them too.
void TestModelFileVersion::clearingResultsClearsSupportReactions()
{
    RModel model;
    model.setSupportReactions({RSupportReaction(R_ENTITY_GROUP_POINT,"Fixed end",RR3Vector(1.0,2.0,3.0))},RR3Vector(-1.0,-2.0,-3.0));
    QCOMPARE(model.getSupportReactions().size(), size_t(1));

    RModel copy(model);
    QCOMPARE(copy.getSupportReactions().size(), size_t(1));

    model.removeAllVariables();
    QVERIFY(model.getSupportReactions().empty());
    QCOMPARE(model.getAppliedLoad().length(), 0.0);
}

QTEST_GUILESS_MAIN(TestModelFileVersion)

#include "tst_model_file_version.moc"
