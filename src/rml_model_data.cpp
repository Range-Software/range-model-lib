#include "rml_model_data.h"

RModelData::RModelData()
{
    this->_init();
}

RModelData::RModelData(const RModelData &modelData)
{
    this->_init(&modelData);
}

RModelData::~RModelData()
{
}

RModelData & RModelData::operator =(const RModelData &modelData)
{
    this->_init(&modelData);
    return (*this);
}

void RModelData::_init(const RModelData *pModelData)
{
    if (pModelData)
    {
        this->selected = pModelData->selected;
    }
}

bool RModelData::getSelected(void) const
{
    return this->selected;
}

void RModelData::setSelected(bool selected)
{
    this->selected = selected;
}
