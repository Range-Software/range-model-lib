#include "rml_file_header.h"

static const char* fileTypeDesc [] =
{

    "None",
    "Model",
    "Material",
    "ViewFactorMatrix",
    "DisplayProperties",
    "Link"
};

void RFileHeader::_init(const RFileHeader *pHeader)
{
    if (pHeader)
    {
        this->version = pHeader->version;
        this->type = pHeader->type;
        this->information = pHeader->information;
    }
}

RFileHeader::RFileHeader(RFileType type,
                         RVersion version,
                         const QString &information) :
    version(version),
    type(type),
    information(information)
{
    this->_init();
}

RFileHeader::RFileHeader(const RFileHeader &header)
{
    this->_init(&header);
}

RFileHeader::~RFileHeader()
{
}

RFileHeader &RFileHeader::operator =(const RFileHeader &header)
{
    this->_init(&header);
    return (*this);
}

const RVersion &RFileHeader::getVersion(void) const
{
    return this->version;
}

void RFileHeader::setVersion(const RVersion &version)
{
    this->version = version;
}

RFileType RFileHeader::getType(void) const
{
    return this->type;
}

void RFileHeader::setType(RFileType type)
{
    this->type = type;
}

const QString &RFileHeader::getInformation(void) const
{
    return this->information;
}

void RFileHeader::setInformation(const QString &information)
{
    this->information = information;
}

QString RFileHeader::toString() const
{
    return "{ Version: " + this->version.toString() + ", File type: " + fileTypeDesc[this->type] + ", Information: " + this->information + " }";
}
