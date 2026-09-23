#include <QtTest>
#include <QTemporaryDir>

#include "rml_material.h"

class TestMaterial : public QObject
{
    Q_OBJECT

private slots:

    void defaultFileExtension();
    void jsonRoundTrip();
    void fileRoundTrip();
    void fileRoundTrip_data();
    void isFluid();
};

void TestMaterial::defaultFileExtension()
{
    QCOMPARE(RMaterial::getDefaultFileExtension(), RMaterial::File::Json::extension);
}

void TestMaterial::jsonRoundTrip()
{
    RMaterial material = RMaterial::generateDefault();
    material.setName("Test material");
    material.setState(RMaterial::Solid);

    RMaterialProperty property(RMaterialProperty::Density);
    // The constructor seeds the table with an initial value - drop it so that
    // the table content is known exactly.
    property.clear();
    property.add(300.0, 1000.0);
    property.add(400.0, 900.0);
    material.add(property);

    RMaterial readMaterial = RMaterial::fromJson(material.toJson());

    QCOMPARE(readMaterial.getID(), material.getID());
    QCOMPARE(readMaterial.getName(), material.getName());
    QCOMPARE(readMaterial.getState(), material.getState());
    QCOMPARE(readMaterial.size(), material.size());
    QVERIFY(readMaterial.propertiesEqual(material));

    // Property types must survive the round trip - not silently fall back to None.
    for (uint i = 0; i < material.size(); i++)
    {
        QCOMPARE(readMaterial.get(i).getType(), material.get(i).getType());
        QCOMPARE(readMaterial.get(i).size(), material.get(i).size());
    }

    uint position = readMaterial.findPosition(RMaterialProperty::Density);
    QVERIFY(position < readMaterial.size());
    QCOMPARE(readMaterial.get(position).size(), 2u);
    QCOMPARE(readMaterial.get(position).getKey(0), 300.0);
    QCOMPARE(readMaterial.get(position).getValue(0), 1000.0);
    QCOMPARE(readMaterial.get(position).getKey(1), 400.0);
    QCOMPARE(readMaterial.get(position).getValue(1), 900.0);
}

void TestMaterial::fileRoundTrip_data()
{
    QTest::addColumn<QString>("extension");

    QTest::newRow("json") << RMaterial::File::Json::extension;
    QTest::newRow("ascii") << RMaterial::File::Ascii::extension;
    QTest::newRow("binary") << RMaterial::File::Binary::extension;
}

void TestMaterial::fileRoundTrip()
{
    QFETCH(QString, extension);

    QTemporaryDir dir;
    QVERIFY(dir.isValid());

    RMaterial material = RMaterial::generateDefault();
    material.setName("Test material");
    material.setState(RMaterial::Liquid);

    QString fileName = dir.filePath("material." + extension);

    material.write(fileName);
    QVERIFY(QFile::exists(fileName));

    RMaterial readMaterial;
    readMaterial.read(fileName);

    QCOMPARE(readMaterial.getID(), material.getID());
    QCOMPARE(readMaterial.getName(), material.getName());
    QCOMPARE(readMaterial.getState(), material.getState());
    QCOMPARE(readMaterial.size(), material.size());
    QVERIFY(readMaterial.propertiesEqual(material));
}

void TestMaterial::isFluid()
{
    RMaterialProperty viscosity(RMaterialProperty::DynamicViscosity);
    RMaterialProperty conductivity(RMaterialProperty::ThermalConductivity);

    // An explicit state decides on its own.
    RMaterial gas(RMaterial::Gas);
    QVERIFY(gas.isFluid());
    RMaterial liquid(RMaterial::Liquid);
    QVERIFY(liquid.isFluid());
    RMaterial solid(RMaterial::Solid);
    solid.add(viscosity);
    QVERIFY(!solid.isFluid());

    // With no state the dynamic viscosity tells a fluid from a solid.
    RMaterial unknownSolid;
    unknownSolid.add(conductivity);
    QVERIFY(!unknownSolid.isFluid());
    RMaterial unknownFluid;
    unknownFluid.add(conductivity);
    unknownFluid.add(viscosity);
    QVERIFY(unknownFluid.isFluid());
}

QTEST_APPLESS_MAIN(TestMaterial)

#include "tst_material.moc"
