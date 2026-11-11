#include <QTest>
#include <QTemporaryDir>

#include <cstring>

#include <rbl_error.h>

#include "rml_environment_condition.h"
#include "rml_file.h"
#include "rml_file_io.h"
#include "rml_model.h"
#include "rml_save_file.h"

class TestFileIOBool : public QObject
{
    Q_OBJECT

private slots:

    void roundTrip_data();
    void roundTrip();
    void writesDigits();
    void readsLegacyCharacters_data();
    void readsLegacyCharacters();
    void rejectsUnexpectedCharacter();
    void readsAnyNonZeroByteAsTrue();
    void writesInvalidBoolAsTrue_data();
    void writesInvalidBoolAsTrue();
    void defaultConditionIsInitialized();
    void asciiModelRoundTrip();

private:

    //! Write raw bytes to a file.
    static void writeBytes(const QString &fileName, const QByteArray &bytes);

    //! Read whole file.
    static QByteArray readBytes(const QString &fileName);

};

void TestFileIOBool::writeBytes(const QString &fileName, const QByteArray &bytes)
{
    QFile file(fileName);
    QVERIFY(file.open(QIODevice::WriteOnly));
    QCOMPARE(file.write(bytes), bytes.size());
}

QByteArray TestFileIOBool::readBytes(const QString &fileName)
{
    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly))
    {
        return QByteArray();
    }
    QByteArray content = file.readAll();
    content.replace("\r","");
    return content;
}

void TestFileIOBool::roundTrip_data()
{
    QTest::addColumn<bool>("binary");
    QTest::addColumn<bool>("addNewLine");

    QTest::newRow("binary") << true << true;
    QTest::newRow("ascii") << false << true;
    QTest::newRow("ascii without new line") << false << false;
}

//! Bool values surrounded by other values read back as written.
void TestFileIOBool::roundTrip()
{
    QFETCH(bool, binary);
    QFETCH(bool, addNewLine);

    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    QString fileName = dir.filePath("bool.dat");

    const bool values[] = { false, true, true, false };
    {
        RSaveFile file(fileName,binary ? RSaveFile::BINARY : RSaveFile::ASCII);
        QVERIFY(file.open(binary ? QIODevice::WriteOnly : (QIODevice::WriteOnly | QIODevice::Text)));
        for (bool value : values)
        {
            if (binary)
            {
                RFileIO::writeBinary(file,value);
                RFileIO::writeBinary(file,uint(7));
            }
            else
            {
                RFileIO::writeAscii(file,value,addNewLine);
                if (!addNewLine)
                {
                    RFileIO::writeAscii(file,' ',false);
                }
                RFileIO::writeAscii(file,uint(7),addNewLine);
                if (!addNewLine)
                {
                    RFileIO::writeAscii(file,' ',false);
                }
            }
        }
        QVERIFY(file.commit());
    }

    RFile file(fileName,binary ? RFile::BINARY : RFile::ASCII);
    QVERIFY(file.open(binary ? QIODevice::ReadOnly : (QIODevice::ReadOnly | QIODevice::Text)));
    try
    {
        for (bool expected : values)
        {
            bool value = !expected;
            uint number = 0;
            if (binary)
            {
                RFileIO::readBinary(file,value);
                RFileIO::readBinary(file,number);
            }
            else
            {
                RFileIO::readAscii(file,value);
                RFileIO::readAscii(file,number);
            }
            QCOMPARE(value, expected);
            QCOMPARE(number, uint(7));
        }
    }
    catch (const RError &error)
    {
        QFAIL(qPrintable("Read failed: " + error.getMessage()));
    }
}

//! ASCII files hold readable digits, not control characters.
void TestFileIOBool::writesDigits()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    QString fileName = dir.filePath("bool.txt");
    {
        RSaveFile file(fileName,RSaveFile::ASCII);
        QVERIFY(file.open(QIODevice::WriteOnly | QIODevice::Text));
        RFileIO::writeAscii(file,false);
        RFileIO::writeAscii(file,true);
        QVERIFY(file.commit());
    }

    QCOMPARE(readBytes(fileName), QByteArray("0\n1\n"));
}

void TestFileIOBool::readsLegacyCharacters_data()
{
    QTest::addColumn<QByteArray>("bytes");

    // Older versions wrote false as 0x00 and true as 0x01 and read them back
    // inverted, losing track of the following values.
    QTest::newRow("lf") << QByteArray("\x00\n\x01\n7\n\x00\n9\n",11);
    QTest::newRow("crlf") << QByteArray("\x00\r\n\x01\r\n7\r\n\x00\r\n9\r\n",15);
}

//! Files written by older versions are read with the correct values.
void TestFileIOBool::readsLegacyCharacters()
{
    QFETCH(QByteArray, bytes);

    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    QString fileName = dir.filePath("legacy.txt");
    writeBytes(fileName,bytes);

    RFile file(fileName,RFile::ASCII);
    QVERIFY(file.open(QIODevice::ReadOnly | QIODevice::Text));

    bool first = true, second = false, third = true;
    uint seven = 0, nine = 0;
    try
    {
        RFileIO::readAscii(file,first);
        RFileIO::readAscii(file,second);
        RFileIO::readAscii(file,seven);
        RFileIO::readAscii(file,third);
        RFileIO::readAscii(file,nine);
    }
    catch (const RError &error)
    {
        QFAIL(qPrintable("Read failed: " + error.getMessage()));
    }
    QCOMPARE(first, false);
    QCOMPARE(second, true);
    QCOMPARE(seven, uint(7));
    QCOMPARE(third, false);
    QCOMPARE(nine, uint(9));
}

//! Anything else than 0 and 1 is reported, not taken as a value.
void TestFileIOBool::rejectsUnexpectedCharacter()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    QString fileName = dir.filePath("bad.txt");
    writeBytes(fileName,"x\n");

    RFile file(fileName,RFile::ASCII);
    QVERIFY(file.open(QIODevice::ReadOnly | QIODevice::Text));
    bool value = false;
    QVERIFY_THROWS_EXCEPTION(RError, RFileIO::readAscii(file,value));
}

//! Older binary files may hold any byte for a bool which was not initialized
//! when saved. Any byte other than 0 reads as true.
void TestFileIOBool::readsAnyNonZeroByteAsTrue()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    QString fileName = dir.filePath("bool.bin");
    writeBytes(fileName,QByteArray("\x50\x00\x01\xF3",4));

    RFile file(fileName,RFile::BINARY);
    QVERIFY(file.open(QIODevice::ReadOnly));
    bool values[4] = { false, true, false, false };
    for (bool &value : values)
    {
        RFileIO::readBinary(file,value);
    }

    QCOMPARE(values[0], true);
    QCOMPARE(values[1], false);
    QCOMPARE(values[2], true);
    QCOMPARE(values[3], true);
    // The stored byte is a proper bool, so it is written as 1.
    unsigned char byte = 0;
    std::memcpy(&byte,&values[3],1);
    QCOMPARE(byte, static_cast<unsigned char>(1));
}

void TestFileIOBool::writesInvalidBoolAsTrue_data()
{
    QTest::addColumn<bool>("binary");

    QTest::newRow("binary") << true;
    QTest::newRow("ascii") << false;
}

//! A bool holding a byte other than 0 or 1, as an uninitialized member does,
//! is written as a proper true value.
void TestFileIOBool::writesInvalidBoolAsTrue()
{
    QFETCH(bool, binary);

    bool invalid = false;
    const unsigned char byte = 0xF3;
    std::memcpy(&invalid,&byte,1);

    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    QString fileName = dir.filePath("bool.dat");
    {
        RSaveFile file(fileName,binary ? RSaveFile::BINARY : RSaveFile::ASCII);
        QVERIFY(file.open(binary ? QIODevice::WriteOnly : (QIODevice::WriteOnly | QIODevice::Text)));
        if (binary)
        {
            RFileIO::writeBinary(file,invalid);
        }
        else
        {
            RFileIO::writeAscii(file,invalid);
        }
        QVERIFY(file.commit());
    }

    QCOMPARE(readBytes(fileName), binary ? QByteArray("\x01",1) : QByteArray("1\n"));
}

//! Conditions which do not set where they apply, environment and initial
//! conditions, must not carry undefined values.
void TestFileIOBool::defaultConditionIsInitialized()
{
    REnvironmentCondition condition(R_ENVIRONMENT_CONDITION_G_ACCELERATION);
    for (const bool flag : {condition.getApplyOnPoint(),condition.getApplyOnLine(),condition.getApplyOnSurface(),condition.getApplyOnVolume()})
    {
        unsigned char byte = 0xFF;
        std::memcpy(&byte,&flag,1);
        QVERIFY(byte == 0 || byte == 1);
    }
}

//! A model saved in the ASCII format can be read back. It has an entity with
//! an environment condition, which used to hold undefined bool values.
void TestFileIOBool::asciiModelRoundTrip()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    QString fileName = dir.filePath("model." + RModel::getDefaultFileExtension(false));

    RModel model;
    model.setName("ASCII model");
    model.getTimeSolver().setEnabled(true);
    model.setNNodes(3);
    model.getNode(0).set(0.0,0.0,0.0);
    model.getNode(1).set(1.0,0.0,0.0);
    model.getNode(2).set(0.0,1.0,0.0);
    RElement triangle(R_ELEMENT_TRI1);
    triangle.setNodeId(0,0);
    triangle.setNodeId(1,1);
    triangle.setNodeId(2,2);
    model.addElement(triangle,true,0);
    RSurface &surface = model.getSurface(0);
    surface.setName("Top face");
    surface.getData().setVisible(false);
    surface.addEnvironmentCondition(REnvironmentCondition(R_ENVIRONMENT_CONDITION_G_ACCELERATION));
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
    QCOMPARE(readModel.getName(), model.getName());
    QCOMPARE(readModel.getTimeSolver().getEnabled(), true);
    QCOMPARE(readModel.getNNodes(), uint(3));
    QCOMPARE(readModel.getNElements(), uint(1));
    QCOMPARE(readModel.getNSurfaces(), uint(1));
    QCOMPARE(readModel.getSurface(0).getName(), QString("Top face"));
    QCOMPARE(readModel.getSurface(0).getData().getVisible(), false);
    QVERIFY(model.getSurface(0).getNEnvironmentConditions() > 0);
    QCOMPARE(readModel.getSurface(0).getNEnvironmentConditions(), model.getSurface(0).getNEnvironmentConditions());
}

QTEST_GUILESS_MAIN(TestFileIOBool)

#include "tst_file_io_bool.moc"
