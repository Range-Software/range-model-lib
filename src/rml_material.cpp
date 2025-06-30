#include <algorithm>

#include <QJsonArray>

#include <rbl_error.h>

#include "rml_material.h"
#include "rml_file_io.h"
#include "rml_file_manager.h"

const RVersion RMaterial::version = RVersion(FILE_MAJOR_VERSION,FILE_MINOR_VERSION,FILE_RELEASE_VERSION);

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
} /* RMaterial::_init */


RMaterial::RMaterial(State state)
{
    this->setState(state);
    this->_init();
} /* RMaterial::RMaterial */


RMaterial::RMaterial(const RMaterial &material)
{
    this->_init (&material);
} /* RMaterial::RMaterial (copy) */


RMaterial::~RMaterial()
{
} /* RMaterial::~RMaterial */


RMaterial &RMaterial::operator =(const RMaterial &material)
{
    this->_init(&material);
    return (*this);
} /* RMaterial::operator = */

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
} /* RMaterial::propertiesEqual */


QUuid RMaterial::getID() const
{
    return this->id;
} /* RMaterial::getID */


void RMaterial::setID(QUuid id)
{
    this->id = id;
} /* RMaterial::setID */


const QString &RMaterial::getName() const
{
    return this->name;
} /* RMaterial::getName */


void RMaterial::setName(const QString &name)
{
    this->name = name;
} /* RMaterial::setName */


RMaterial::State RMaterial::getState() const
{
    return this->state;
} /* RMaterial::getState */


void RMaterial::setState(State state)
{
    this->state = state;
} /* RMaterial::setState */


uint RMaterial::size() const
{
    return (uint)this->properties.size();
} /* RMaterial::size */


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
} /* RMaterial::add */


const RMaterialProperty &RMaterial::get(uint position) const
{
    R_ERROR_ASSERT (position < this->size());
    return this->properties[position];
} /* RMaterial::get */


RMaterialProperty &RMaterial::get(uint position)
{
    R_ERROR_ASSERT (position < this->size());
    return this->properties[position];
} /* RMaterial::get */


void RMaterial::remove(uint position)
{
    this->properties.remove(position);
} /* RMaterial::remove */


void RMaterial::clear()
{
    this->id = QUuid::createUuid();
    this->name.clear();
    this->state = None;
    this->properties.clear();
} /* RMaterial::clear */


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
} /* RMaterial::find_position */


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
} /* RMaterial::hasProperties */


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
} /* RMaterial::generateDefault */


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
} /* RMaterial::fromJson */


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
} /* RMaterial::toJson */


void RMaterial::read(const QString &fileName)
{
    if (fileName.isEmpty())
    {
        throw RError(RError::Type::InvalidFileName,R_ERROR_REF,"No file name was provided.");
    }

    QString ext = RFileManager::getExtension(fileName);

    try
    {
        if (ext == RMaterial::getDefaultFileExtension(false))
        {
            this->readAscii(fileName);
        }
        else if (ext == RMaterial::getDefaultFileExtension(true))
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
} /* RMaterial::read */


void RMaterial::write(const QString &fileName) const
{
    if (fileName.isEmpty())
    {
        throw RError(RError::Type::InvalidFileName,R_ERROR_REF,"No file was name provided.");
    }

    QString ext = RFileManager::getExtension(fileName);

    try
    {
        if (ext == RMaterial::getDefaultFileExtension(false))
        {
            this->writeAscii(fileName);
        }
        else if (ext == RMaterial::getDefaultFileExtension(true))
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
} /* RMaterial::write */


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
} /* RMaterial::import */


QString RMaterial::findMaterialFileName(bool binary) const
{
    return this->getID().toString(QUuid::WithoutBraces) + QString(".") + RMaterial::getDefaultFileExtension(binary);
} /* RMaterial::findMaterialFileName */


bool RMaterial::validForProblemType(RProblemTypeMask problemTypeMask) const
{
    return this->hasProperties(RMaterialProperty::getTypes(problemTypeMask));
} /* RMaterial::findMaterialFileName */


QString RMaterial::getDefaultFileExtension(bool binary)
{
    if (binary)
    {
        return "rbmt";
    }
    else
    {
        return "ramt";
    }
} /* RMaterial::getDefaultFileExtension */


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
} /* RMaterial::readAscii */


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
} /* RMaterial::readBinary */


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
} /* RMaterial::writeAscii */


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
} /* RMaterial::writeBinary */


static bool propertyTypeCompareFunc(const RMaterialProperty &p1,const RMaterialProperty &p2)
{
    return (p1.getType()<p2.getType());
}
