#include <algorithm>

#include <QJsonArray>

#include <rbl_error.h>
#include <rbl_logger.h>

#include "rml_material.h"
#include "rml_file_io.h"
#include "rml_file_utils.h"
#include "rml_problem.h"

const RVersion RMaterial::version = RVersion(FILE_MAJOR_VERSION,FILE_MINOR_VERSION,FILE_RELEASE_VERSION);

const QString RMaterial::File::Json::name = "JSON";
const QString RMaterial::File::Json::extension = "json";

const QString RMaterial::File::Binary::name = "Binary";
const QString RMaterial::File::Binary::extension = "rbmt";

const QString RMaterial::File::Ascii::name = "ASCII";
const QString RMaterial::File::Ascii::extension = "ramt";

static bool propertyTypeCompareFunc(const RMaterialProperty &p1,const RMaterialProperty &p2);

void RMaterial::_init (const RMaterial *pMaterial)
{
    this->id = QUuid::createUuid();
    if (pMaterial)
    {
        this->id = pMaterial->id;
        this->name = pMaterial->name;
        this->state = pMaterial->state;
        this->properties = pMaterial->properties;
    }
}

RMaterial::RMaterial(State state)
{
    this->setState(state);
    this->_init();
}

RMaterial::RMaterial(const RMaterial &material)
{
    this->_init (&material);
}

RMaterial::~RMaterial()
{
}

RMaterial &RMaterial::operator =(const RMaterial &material)
{
    this->_init(&material);
    return (*this);
}

bool RMaterial::propertiesEqual(const RMaterial &material) const
{
    if (this->properties.size() != material.properties.size())
    {
        return false;
    }
    for (uint i=0;i<this->properties.size();i++)
    {
        if (this->properties[i] != material.properties[i])
        {
            return false;
        }
    }
    return true;
}

QUuid RMaterial::getID() const
{
    return this->id;
}

void RMaterial::setID(QUuid id)
{
    this->id = id;
}

const QString &RMaterial::getName() const
{
    return this->name;
}

void RMaterial::setName(const QString &name)
{
    this->name = name;
}

RMaterial::State RMaterial::getState() const
{
    return this->state;
}

void RMaterial::setState(State state)
{
    this->state = state;
}

uint RMaterial::size() const
{
    return (uint)this->properties.size();
}

bool RMaterial::add(const RMaterialProperty &property)
{
    uint position = 0;

    position = this->findPosition(property.getType());

    if (position >= this->size())
    {
        this->properties.push_back(property);
        std::sort(this->properties.begin(),this->properties.end(),propertyTypeCompareFunc);
        return true;
    }
    else
    {
        this->properties[position] = property;
        return false;
    }
}

const RMaterialProperty &RMaterial::get(uint position) const
{
    R_ERROR_ASSERT (position < this->size());
    return this->properties[position];
}

RMaterialProperty &RMaterial::get(uint position)
{
    R_ERROR_ASSERT (position < this->size());
    return this->properties[position];
}

void RMaterial::remove(uint position)
{
    this->properties.remove(position);
}

void RMaterial::clear()
{
    this->id = QUuid::createUuid();
    this->name.clear();
    this->state = None;
    this->properties.clear();
}

uint RMaterial::findPosition(RMaterialProperty::Type type) const
{
    uint pos = 0;

    for (auto iter = this->properties.cbegin();iter != this->properties.cend();++iter)
    {
        if (iter->getType() == type)
        {
           return pos;
        }
        pos++;
    }

    return this->size();
}

bool RMaterial::hasProperties(const QList<RMaterialProperty::Type> &propertyTypes) const
{
    if (propertyTypes.size() == 0)
    {
        return false;
    }

    for (const RMaterialProperty::Type &propertyType : propertyTypes)
    {
        if (this->findPosition(propertyType) == this->size())
        {
            return false;
        }
    }
    return true;
}

RMaterial RMaterial::generateDefault()
{
    RMaterial material;

    material.setName("Default");
    for (RMaterialProperty::Type type=RMaterialProperty::Type(None+1);
         type<RMaterialProperty::Type(nTypes-1);
         type++)
    {
        RMaterialProperty property(type);
        material.add(property);
    }

    return material;
}

RMaterial RMaterial::fromJson(const QJsonObject &json)
{
    RMaterial material;

    if (const QJsonValue v = json["id"]; v.isString())
    {
        material.id = QUuid(v.toString());
    }

    if (const QJsonValue v = json["name"]; v.isString())
    {
        material.name = v.toString();
    }

    if (const QJsonValue v = json["state"]; v.isString())
    {
        material.state = RMaterial::State(v.toString().toInt());
    }

    if (const QJsonValue v = json["properties"]; v.isArray())
    {
        QJsonArray jArray = v.toArray();

        for (qsizetype i=0; i<jArray.count(); i++)
        {
            material.add(RMaterialProperty::fromJson(jArray.at(i).toObject()));
        }
    }

    return material;
}

QJsonObject RMaterial::toJson() const
{
    QJsonObject jObject;

    jObject["id"] = this->id.toString(QUuid::WithoutBraces);
    jObject["name"] = this->name;
    jObject["state"] = QString::number(this->state);

    QJsonArray jArray;
    for (const RMaterialProperty &property : this->properties)
    {
        jArray.append(property.toJson());
    }

    jObject["properties"] = jArray;

    return jObject;
}

void RMaterial::read(const QString &fileName)
{
    if (fileName.isEmpty())
    {
        throw RError(RError::Type::InvalidFileName,R_ERROR_REF,"No file name was provided.");
    }

    QString ext = RFileUtils::getExtension(fileName).toLower();

    try
    {
        if (ext == RMaterial::File::Json::extension)
        {
            this->readJson(fileName);
        }
        else if (ext == RMaterial::File::Ascii::extension)
        {
            this->readAscii(fileName);
        }
        else if (ext == RMaterial::File::Binary::extension)
        {
            this->readBinary(fileName);
        }
        else
        {
            throw RError(RError::Type::InvalidFileName,R_ERROR_REF, "Unknown extension \"" + ext + "\".");
        }
    }
    catch (RError &error)
    {
        throw error;
    }
    catch (const std::exception &error)
    {
        throw RError(RError::Type::Application,R_ERROR_REF, "%s", error.what());
    }
    catch (...)
    {
        throw RError(RError::Type::Application,R_ERROR_REF, "Unknown exception.");
    }
}

void RMaterial::write(const QString &fileName) const
{
    if (fileName.isEmpty())
    {
        throw RError(RError::Type::InvalidFileName,R_ERROR_REF,"No file was name provided.");
    }

    QString ext = RFileUtils::getExtension(fileName).toLower();

    try
    {
        if (ext == RMaterial::File::Json::extension)
        {
            this->writeJson(fileName);
        }
        else if (ext == RMaterial::File::Ascii::extension)
        {
            this->writeAscii(fileName);
        }
        else if (ext == RMaterial::File::Binary::extension)
        {
            this->writeBinary(fileName);
        }
        else
        {
            throw RError(RError::Type::InvalidFileName,R_ERROR_REF, "Unknown extension \"" + ext + "\".");
        }
    }
    catch (RError &error)
    {
        throw error;
    }
    catch (const std::exception &error)
    {
        throw RError(RError::Type::Application,R_ERROR_REF, "%s", error.what());
    }
    catch (...)
    {
        throw RError(RError::Type::Application,R_ERROR_REF, "Unknown exception.");
    }
}

void RMaterial::import(const QString &fileName)
{
    if (fileName.isEmpty())
    {
        throw RError(RError::Type::InvalidFileName,R_ERROR_REF,"No file was name provided.");
    }

    try
    {
        RFile file(fileName,RFile::ASCII);

        if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            throw RError(RError::Type::OpenFile,R_ERROR_REF,"Failed to open the file \'%s\'.",fileName.toUtf8().constData());
        }

        QString flag;
        QString fVersion;
        uint nProperties = 0;
        QVector<QString> tableFlags;
        QVector<uint> tableSizes;

        while (true)
        {

            file.getTextStream() >> flag;
            if (file.getTextStream().status() != QTextStream::Ok)
            {
                if (file.getTextStream().atEnd())
                {
                    break;
                }
                throw RError(RError::Type::ReadFile,R_ERROR_REF,"Failed to read the file \'%s\'",fileName.toUtf8().constData());
            }

            if (flag == "!VERSION")
            {
                RFileIO::readAscii(file,fVersion);
            }
            else if (flag == "NAME:")
            {
                RFileIO::readAscii(file,this->name);
            }
            else if (flag == "NPROP:")
            {
                RFileIO::readAscii(file,nProperties);
            }
            else if (flag == "properties:")
            {
                QString propertyName;
                QString propertyDimension;
                double propertyValue;
                uint propertyNTableValues;
                for (uint i=0;i<nProperties;i++)
                {
                    RFileIO::readAscii(file,propertyName);
                    RFileIO::readAscii(file,propertyValue);
                    RFileIO::readAscii(file,propertyDimension);
                    RFileIO::readAscii(file,propertyNTableValues);

                    tableFlags.push_back(QString(propertyName + "/temperature:").replace(' ','_'));
                    tableSizes.push_back(std::max(propertyNTableValues,uint(0)));

                    RMaterialProperty property;

                    propertyName = propertyName.toLower();
                    if (propertyName == "density")
                    {
                        property.setType(RMaterialProperty::Density);
                    }
                    else if (propertyName == "dynamic viscosity")
                    {
                        property.setType(RMaterialProperty::DynamicViscosity);
                    }
                    else if (propertyName == "electrical conductivity")
                    {
                        property.setType(RMaterialProperty::ElectricalConductivity);
                    }
                    else if (propertyName == "emissivity")
                    {
                        property.setType(RMaterialProperty::Emissivity);
                    }
                    else if (propertyName == "heat capacity")
                    {
                        property.setType(RMaterialProperty::HeatCapacity);
                    }
                    else if (propertyName == "modulus of elasticity")
                    {
                        property.setType(RMaterialProperty::ModulusOfElasiticity);
                    }
                    else if (propertyName == "permeability")
                    {
                        property.setType(RMaterialProperty::PermeabilityToFluids);
                    }
                    else if (propertyName == "poissons ratio")
                    {
                        property.setType(RMaterialProperty::PoissonRatio);
                    }
                    else if (propertyName == "relative permittivity")
                    {
                        property.setType(RMaterialProperty::RelativePermittivity);
                    }
                    else if (propertyName == "speed of sound")
                    {
                        property.setType(RMaterialProperty::SoundSpeed);
                    }
                    else if (propertyName == "thermal conductivity")
                    {
                        property.setType(RMaterialProperty::ThermalConductivity);
                    }
                    else if (propertyName == "thermal expansion coefficient")
                    {
                        property.setType(RMaterialProperty::ThermalExpansionCoefficient);
                    }
                    if (propertyNTableValues < 1)
                    {
                        property.add(293.15,propertyValue);
                    }

                    this->add(property);
                }
            }
            else
            {
                for (int i=0;i<tableFlags.size();i++)
                {
                    if (flag == tableFlags[i])
                    {
                        for (uint j=0;j<tableSizes[i];j++)
                        {
                            double key, value;
                            RFileIO::readAscii(file,key);
                            RFileIO::readAscii(file,value);
                            this->get(i).add(key,value);
                        }
                    }
                }
            }
        }
    }
    catch (RError &error)
    {
        throw error;
    }
    catch (const std::exception &error)
    {
        throw RError(RError::Type::Application,R_ERROR_REF, "%s", error.what());
    }
    catch (...)
    {
        throw RError(RError::Type::Application,R_ERROR_REF, "Unknown exception.");
    }
}

bool RMaterial::validForProblemType(RProblemTypeMask problemTypeMask) const
{
    return this->hasProperties(RMaterialProperty::getTypes(problemTypeMask));
}

QString RMaterial::getDefaultFileExtension()
{
    return RMaterial::File::Json::extension;
}

QByteArray RMaterial::getJsonFileSpecs()
{
    const std::vector<RProblemType> problemTypes = RProblem::getTypes(R_PROBLEM_ALL);

    const struct
    {
        RMaterial::State state;
        const char *description;
    } states[] =
    {
        { RMaterial::None,   "unspecified / unknown" },
        { RMaterial::Gas,    "gas"                   },
        { RMaterial::Liquid, "liquid"                },
        { RMaterial::Solid,  "solid"                 }
    };

    const QString keyTypeId = RVariable::getId(R_VARIABLE_TEMPERATURE);

    QString spec;

    spec += "RANGE FEA MATERIAL FILE (JSON) - FORMAT SPECIFICATION\n";
    spec += "====================================================\n";
    spec += "\n";
    spec += "1. GENERAL\n";
    spec += "\n";
    spec += "   Format name    : " + RMaterial::File::Json::name + "\n";
    spec += "   File extension : ." + RMaterial::File::Json::extension + "\n";
    spec += "   File version   : " + RMaterial::version.toString() + "\n";
    spec += "   Encoding       : UTF-8 text holding one JSON object (RFC 8259).\n";
    spec += "   Content        : A single material - its identification and a list of material\n";
    spec += "                    properties. Every property is a table of values given as a\n";
    spec += "                    function of a key variable (temperature).\n";
    spec += "   Units          : SI units only. Units are implied by the property type and are\n";
    spec += "                    never stored in the file (see section 5).\n";
    spec += "   Member order   : Irrelevant when reading. The writer emits members sorted\n";
    spec += "                    alphabetically and indented by 4 spaces.\n";
    spec += "\n";
    spec += "2. TOP-LEVEL OBJECT\n";
    spec += "\n";
    spec += "   Member        Type    Required  Description\n";
    spec += "   ------------  ------  --------  ---------------------------------------------------\n";
    spec += "   \"id\"          string  no        Material UUID, lower case, without braces\n";
    spec += "                                   (8-4-4-4-12 hexadecimal digits). When missing, a new\n";
    spec += "                                   random UUID is generated on read.\n";
    spec += "   \"name\"        string  yes       Human readable material name, e.g. \"Water\".\n";
    spec += "   \"state\"       string  no        State of matter (section 3). CAUTION: it is a\n";
    spec += "                                   decimal integer WRITTEN AS A JSON STRING, e.g. \"3\".\n";
    spec += "                                   A plain JSON number is ignored on read.\n";
    spec += "   \"properties\"  array   yes       List of property objects (section 4).\n";
    spec += "\n";
    spec += "   Unknown members are ignored. Members of a wrong JSON type are ignored and the\n";
    spec += "   corresponding default is kept.\n";
    spec += "\n";
    spec += "3. MATERIAL STATE VALUES\n";
    spec += "\n";
    spec += "   Value  Meaning\n";
    spec += "   -----  ---------------------\n";
    for (size_t i=0;i<sizeof(states)/sizeof(states[0]);i++)
    {
        spec += "   " + QString("\"%1\"").arg(int(states[i].state)).leftJustified(7)
              + QString::fromLatin1(states[i].description) + "\n";
    }
    spec += "\n";
    spec += "   Any other value is read as-is but has no meaning to the solvers.\n";
    spec += "\n";
    spec += "4. PROPERTY OBJECT\n";
    spec += "\n";
    spec += "   Member       Type    Required  Description\n";
    spec += "   -----------  ------  --------  ----------------------------------------------------\n";
    spec += "   \"type\"       string  yes       Property type ID (section 5). An unknown ID is read\n";
    spec += "                                  as \"" + RMaterialProperty::getId(RMaterialProperty::None) + "\".\n";
    spec += "   \"keyType\"    string  yes       Variable ID of the table key. In practice always\n";
    spec += "                                  \"" + keyTypeId + "\" (unit " + RVariable::getUnits(R_VARIABLE_TEMPERATURE) + "). An unknown ID is read as\n";
    spec += "                                  \"" + RVariable::getId(R_VARIABLE_NONE) + "\".\n";
    spec += "   \"table\"      array   yes       Key-value pairs, at least one entry.\n";
    spec += "\n";
    spec += "   Each entry of \"table\" is an object with two members:\n";
    spec += "\n";
    spec += "   Member       Type    Required  Description\n";
    spec += "   -----------  ------  --------  ----------------------------------------------------\n";
    spec += "   \"key\"        number  yes       Value of the key variable, e.g. temperature in " + RVariable::getUnits(R_VARIABLE_TEMPERATURE) + ".\n";
    spec += "   \"value\"      number  yes       Property value at that key, in the property units.\n";
    spec += "\n";
    spec += "   Both members must be JSON numbers; an entry with a string or a missing member is\n";
    spec += "   silently skipped. Entries are stored in a map keyed by \"key\", therefore:\n";
    spec += "     - entries are sorted by ascending \"key\" when written,\n";
    spec += "     - a repeated \"key\" overwrites the previous entry,\n";
    spec += "     - a property with an empty table is invalid and must not be produced.\n";
    spec += "   A value between two keys is linearly interpolated, a value outside of the table\n";
    spec += "   range is clamped to the nearest entry. A single-entry table means a constant.\n";
    spec += "\n";
    spec += "   A material holds at most one property of each type - a repeated \"type\" replaces\n";
    spec += "   the previously read property.\n";
    spec += "\n";
    spec += "5. PROPERTY TYPE IDS\n";
    spec += "\n";
    spec += "   ID                                  Name                            Units      Default\n";
    spec += "   ----------------------------------  ------------------------------  ---------  -------\n";
    for (uint i=uint(RMaterialProperty::None)+1;i<uint(RMaterialProperty::nTypes);i++)
    {
        RMaterialProperty::Type type = RMaterialProperty::Type(i);

        spec += "   " + RMaterialProperty::getId(type).leftJustified(36)
              + RMaterialProperty::getName(type).leftJustified(32)
              + RMaterialProperty::getUnits(type).leftJustified(11)
              + QString::number(RMaterialProperty::getInitValue(type)) + "\n";
    }
    spec += "\n";
    spec += "   Units \"N/A\" mark a dimensionless quantity. The \"Default\" column holds the value\n";
    spec += "   used when a property is created without data - it is not a physical value.\n";
    spec += "   \"" + RMaterialProperty::getId(RMaterialProperty::Custom) + "\" carries a user defined quantity and is ignored by the solvers.\n";
    spec += "\n";
    spec += "6. PROPERTIES REQUIRED PER PROBLEM TYPE\n";
    spec += "\n";
    spec += "   A material is valid for a problem only when it holds all listed properties.\n";
    spec += "\n";
    qsizetype problemNameWidth = 0;
    for (RProblemType problemType : problemTypes)
    {
        problemNameWidth = std::max(problemNameWidth,RProblem::getName(problemType).length());
    }
    for (RProblemType problemType : problemTypes)
    {
        QList<RMaterialProperty::Type> propertyTypes = RMaterialProperty::getTypes(problemType);

        QStringList propertyIds;
        for (RMaterialProperty::Type type : propertyTypes)
        {
            propertyIds.append(RMaterialProperty::getId(type));
        }

        spec += "   " + RProblem::getName(problemType).leftJustified(problemNameWidth+2)
              + (propertyIds.isEmpty() ? QString("(none)") : propertyIds.join(", ")) + "\n";
    }
    spec += "\n";
    spec += "7. EXAMPLE\n";
    spec += "\n";
    spec += "   Liquid water with a temperature dependent density and a constant heat capacity\n";
    spec += "   (shown compacted - the writer places every member on its own line):\n";
    spec += "\n";
    spec += "   {\n";
    spec += "       \"id\": \"6f9619ff-8b86-d011-b42d-00cf4fc964ff\",\n";
    spec += "       \"name\": \"Water\",\n";
    spec += "       \"properties\": [\n";
    spec += "           {\n";
    spec += "               \"keyType\": \"" + keyTypeId + "\",\n";
    spec += "               \"table\": [\n";
    spec += "                   { \"key\": 273.15, \"value\": 999.8 },\n";
    spec += "                   { \"key\": 293.15, \"value\": 998.2 },\n";
    spec += "                   { \"key\": 373.15, \"value\": 958.4 }\n";
    spec += "               ],\n";
    spec += "               \"type\": \"" + RMaterialProperty::getId(RMaterialProperty::Density) + "\"\n";
    spec += "           },\n";
    spec += "           {\n";
    spec += "               \"keyType\": \"" + keyTypeId + "\",\n";
    spec += "               \"table\": [\n";
    spec += "                   { \"key\": 293.15, \"value\": 4182 }\n";
    spec += "               ],\n";
    spec += "               \"type\": \"" + RMaterialProperty::getId(RMaterialProperty::HeatCapacity) + "\"\n";
    spec += "           }\n";
    spec += "       ],\n";
    spec += "       \"state\": \"" + QString::number(int(RMaterial::Liquid)) + "\"\n";
    spec += "   }\n";

    return spec.toLatin1();
}

void RMaterial::readJson(const QString &fileName)
{
    if (fileName.isEmpty())
    {
        throw RError(RError::Type::InvalidFileName,R_ERROR_REF,"No file name was provided.");
    }

    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        throw RError(RError::Type::OpenFile,R_ERROR_REF,"Failed to open the file \"%s\".",fileName.toUtf8().constData());
    }

    RLogger::info("Reading JSON file \"%s\"\n",fileName.toUtf8().constData());
    RLogger::indent();

    QByteArray byteArray = file.readAll();
    RLogger::info("Successfuly read \"%ld\" bytes from \"%s\".\n",byteArray.size(),file.fileName().toUtf8().constData());

    (*this) = RMaterial::fromJson(QJsonDocument::fromJson(byteArray).object());

    RLogger::unindent();

    file.close();
}

void RMaterial::readAscii(const QString &fileName)
{
    if (fileName.isEmpty())
    {
        throw RError(RError::Type::InvalidFileName,R_ERROR_REF,"No file name was provided");
    }

    RFile materialFile(fileName,RFile::ASCII);

    if (!materialFile.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        throw RError(RError::Type::OpenFile,R_ERROR_REF,"Failed to open the material file \'%s\'.",fileName.toUtf8().constData());
    }

    RFileHeader fileHeader;

    RFileIO::readAscii(materialFile,fileHeader);
    if (fileHeader.getType() != R_FILE_TYPE_MATERIAL)
    {
        throw RError(RError::Type::InvalidFileFormat,R_ERROR_REF,"File type of the file \'" + fileName + "\' is not MATERIAL.");
    }

    // Set file version
    materialFile.setVersion(fileHeader.getVersion());

    RFileIO::readAscii(materialFile,*this);

    materialFile.close();
}

void RMaterial::readBinary(const QString &fileName)
{
    if (fileName.isEmpty())
    {
        throw RError(RError::Type::InvalidFileName,R_ERROR_REF,"No file name was provided");
    }

    RFile materialFile(fileName,RFile::BINARY);

    if (!materialFile.open(QIODevice::ReadOnly))
    {
        throw RError(RError::Type::OpenFile,R_ERROR_REF,"Failed to open the material file \'%s\'.",fileName.toUtf8().constData());
    }

    RFileHeader fileHeader;

    RFileIO::readBinary(materialFile,fileHeader);
    if (fileHeader.getType() != R_FILE_TYPE_MATERIAL)
    {
        throw RError(RError::Type::InvalidFileFormat,R_ERROR_REF,"Filed type of the file \'" + fileName + "\' is not MATERIAL.");
    }

    // Set file version
    materialFile.setVersion(fileHeader.getVersion());

    RFileIO::readBinary(materialFile,*this);

    materialFile.close();
}

void RMaterial::writeJson(const QString &fileName) const
{
    if (fileName.isEmpty())
    {
        throw RError(RError::Type::InvalidFileName,R_ERROR_REF,"No file name was provided.");
    }

    QSaveFile file(fileName);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        throw RError(RError::Type::OpenFile,R_ERROR_REF,"Failed to open the file \"%s\".",fileName.toUtf8().constData());
    }

    RLogger::info("Writing JSON file \"%s\"\n",fileName.toUtf8().constData());
    RLogger::indent();

    qint64 bytesOut = file.write(QJsonDocument(this->toJson()).toJson());

    RLogger::info("Successfuly wrote \"%ld\" bytes to \"%s\".\n",bytesOut,file.fileName().toUtf8().constData());

    RLogger::unindent();

    file.commit();
}

void RMaterial::writeAscii(const QString &fileName) const
{
    if (fileName.isEmpty())
    {
        throw RError(RError::Type::InvalidFileName,R_ERROR_REF,"No file name was provided.");
    }

    RSaveFile materialFile(fileName,RSaveFile::ASCII);

    if (!materialFile.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        throw RError(RError::Type::OpenFile,R_ERROR_REF,"Failed to open the material file \'%s\'.",fileName.toUtf8().constData());
    }

    RFileIO::writeAscii(materialFile,RFileHeader(R_FILE_TYPE_MATERIAL,RMaterial::version));
    RFileIO::writeAscii(materialFile,*this);

    materialFile.commit();
}

void RMaterial::writeBinary(const QString &fileName) const
{
    if (fileName.isEmpty())
    {
        throw RError(RError::Type::InvalidFileName,R_ERROR_REF,"No file name was provided.");
    }

    RSaveFile materialFile(fileName,RSaveFile::BINARY);

    if (!materialFile.open(QIODevice::WriteOnly))
    {
        throw RError(RError::Type::OpenFile,R_ERROR_REF,"Failed to open the material file \'%s\'.",fileName.toUtf8().constData());
    }

    RFileIO::writeBinary(materialFile,RFileHeader(R_FILE_TYPE_MATERIAL,RMaterial::version));
    RFileIO::writeBinary(materialFile,*this);

    materialFile.commit();
}

static bool propertyTypeCompareFunc(const RMaterialProperty &p1,const RMaterialProperty &p2)
{
    return (p1.getType()<p2.getType());
}
