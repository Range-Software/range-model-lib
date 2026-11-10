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

QTEST_GUILESS_MAIN(TestModelFileVersion)

#include "tst_model_file_version.moc"
