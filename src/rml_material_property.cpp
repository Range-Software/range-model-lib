#include <QJsonArray>

#include <rbl_error.h>

#include "rml_material_property.h"
#include "rml_problem.h"
#include "rml_variable.h"

typedef struct _RMaterialPropertyDesc
{
    QString          id;
    RVariableType    variableType;
    RProblemTypeMask problemTypeMask;
} RMaterialPropertyDesc;

const RMaterialPropertyDesc materialPropertyDesc [] =
{
    { "prop-none",                          R_VARIABLE_NONE,                          R_PROBLEM_NONE },
    { "prop-density",                       R_VARIABLE_DENSITY,                       R_PROBLEM_ACOUSTICS | R_PROBLEM_FLUID_PARTICLE | R_PROBLEM_ELECTROSTATICS | R_PROBLEM_FLUID | R_PROBLEM_FLUID_HEAT | R_PROBLEM_HEAT | R_PROBLEM_STRESS | R_PROBLEM_STRESS_MODAL | R_PROBLEM_POTENTIAL | R_PROBLEM_WAVE },
    { "prop-dynamic_viscosity",             R_VARIABLE_DYNAMIC_VISCOSITY,             R_PROBLEM_FLUID | R_PROBLEM_FLUID_HEAT },
    { "prop-electrical_conductivity",       R_VARIABLE_ELECTRICAL_CONDUCTIVITY,       R_PROBLEM_ELECTROSTATICS },
    { "prop-emissivity",                    R_VARIABLE_EMISSIVITY,                    R_PROBLEM_HEAT },
    { "prop-heat_capacity",                 R_VARIABLE_HEAT_CAPACITY,                 R_PROBLEM_HEAT | R_PROBLEM_FLUID_HEAT },
    { "prop-modulus_of_elasticity",         R_VARIABLE_MODULUS_OF_ELASTICITY,         R_PROBLEM_ACOUSTICS | R_PROBLEM_STRESS | R_PROBLEM_STRESS_MODAL },
    { "prop-permeability_to_fluids",        R_VARIABLE_PERMEABILITY_TO_FLUIDS,        R_PROBLEM_POTENTIAL },
    { "prop-poisson_ratio",                 R_VARIABLE_POISSON_RATIO,                 R_PROBLEM_STRESS | R_PROBLEM_STRESS_MODAL },
    { "prop-relative_permittivity",         R_VARIABLE_RELATIVE_PERMITTIVITY,         R_PROBLEM_ELECTROSTATICS },
    { "prop-soubd_speed",                   R_VARIABLE_SOUND_SPEED,                   R_PROBLEM_NONE },
    { "prop-thermal_conductivity",          R_VARIABLE_THERMAL_CONDUCTIVITY,          R_PROBLEM_HEAT | R_PROBLEM_FLUID_HEAT },
    { "prop-thermal_expansion_coefficient", R_VARIABLE_THERMAL_EXPANSION_COEFFICIENT, R_PROBLEM_STRESS | R_PROBLEM_STRESS_MODAL },
    { "prop-custom",                        R_VARIABLE_CUSTOM,                        R_PROBLEM_NONE }
};


void RMaterialProperty::_init(const RMaterialProperty *pMaterialProperty)
{
    this->keyType = R_VARIABLE_TEMPERATURE;
    this->RValueTable::setKeyName(RVariable::getName (this->getKeyType()));
    this->RValueTable::setKeyUnits(RVariable::getUnits (this->getKeyType()));
    if (pMaterialProperty)
    {
        this->type = pMaterialProperty->getType();
        this->RValueTable::setValueName(RMaterialProperty::getName (this->type));
        this->RValueTable::setValueUnits(RMaterialProperty::getUnits (this->type));
    }
} /* RMaterialProperty::_init */


RMaterialProperty::RMaterialProperty(RMaterialProperty::Type type)
{
    this->_init ();
    this->setType (type);
} /* RMaterialProperty::RMaterialProperty */


RMaterialProperty::RMaterialProperty(const RMaterialProperty &materialProperty)
    : RValueTable (materialProperty)
{
    this->_init (&materialProperty);
} /* RMaterialProperty::RMaterialProperty (copy) */


RMaterialProperty::~RMaterialProperty ()
{
} /* RMaterialProperty::~RMaterialProperty */


RMaterialProperty &RMaterialProperty::operator =(const RMaterialProperty &materialProperty)
{
    this->RValueTable::operator = (materialProperty);
    this->_init (&materialProperty);
    return (*this);
} /* RMaterialProperty::operator = */


bool RMaterialProperty::operator ==(const RMaterialProperty &materialProperty) const
{
    if (this->keyType != materialProperty.keyType)
    {
        return false;
    }
    if (this->type != materialProperty.type)
    {
        return false;
    }
    if (this->RValueTable::operator !=(materialProperty))
    {
        return false;
    }
    return true;
} /* RMaterialProperty::operator == */


bool RMaterialProperty::operator !=(const RMaterialProperty &materialProperty) const
{
    return !(this->operator ==(materialProperty));
} /* RMaterialProperty::operator != */


RVariableType RMaterialProperty::getKeyType() const
{
    return this->keyType;
} /* RMaterialProperty::get_key_type */


RMaterialProperty::Type RMaterialProperty::getType() const
{
    return this->type;
} /* RMaterialProperty::get_type */


void RMaterialProperty::setType(RMaterialProperty::Type type)
{
    R_ERROR_ASSERT(R_MATERIAL_PROPERTY_TYPE_IS_VALID(type));

    this->type = type;
    this->RValueTable::setValueName(RMaterialProperty::getName(type));
    this->RValueTable::setValueUnits(RMaterialProperty::getUnits(type));
    this->clear ();
    this->add (RVariable::getInitValue(this->getKeyType()),RMaterialProperty::getInitValue(this->type));
} /* RMaterialProperty::setType */


const QString &RMaterialProperty::getName() const
{
    return this->getValueName();
} /* RMaterialProperty::set_name */


void RMaterialProperty::setName(const QString &name)
{
    this->RValueTable::setValueName (name);
} /* RMaterialProperty::set_name */


const QString &RMaterialProperty::getUnits() const
{
    return this->getValueUnits();
} /* RMaterialProperty::getUnits */


void RMaterialProperty::setUnits(const QString &units)
{
    this->RValueTable::setValueUnits(units);
} /* RMaterialProperty::setUnits */


RMaterialProperty::Type RMaterialProperty::getTypeFromId(const QString &materialId)
{
    for (uint type=uint(None);type<uint(nTypes);type++)
    {
        if (materialPropertyDesc[type].id == materialId)
        {
            return RMaterialProperty::Type(type);
        }
    }

    return None;
} /* RMaterialProperty::getTypeFromId */


QString RMaterialProperty::getId(RMaterialProperty::Type type)
{
    R_ERROR_ASSERT (R_MATERIAL_PROPERTY_TYPE_IS_VALID (type));
    return materialPropertyDesc[type].id;
} /* RMaterialProperty::getId */


const QString &RMaterialProperty::getName(RMaterialProperty::Type type)
{
    R_ERROR_ASSERT (R_MATERIAL_PROPERTY_TYPE_IS_VALID (type));
    return RVariable::getName(RMaterialProperty::getVariableType(type));
} /* RMaterialProperty::getName */


const QString &RMaterialProperty::getUnits(RMaterialProperty::Type type)
{
    R_ERROR_ASSERT (R_MATERIAL_PROPERTY_TYPE_IS_VALID (type));
    return RVariable::getUnits(RMaterialProperty::getVariableType(type));
} /* RMaterialProperty::getUnits */


double RMaterialProperty::getInitValue(RMaterialProperty::Type type)
{
    R_ERROR_ASSERT (R_MATERIAL_PROPERTY_TYPE_IS_VALID (type));
    return RVariable::getInitValue(RMaterialProperty::getVariableType(type));
} /* RMaterialProperty::getInitValue */


RProblemTypeMask RMaterialProperty::getProblemTypeMask(RMaterialProperty::Type type)
{
    R_ERROR_ASSERT (R_MATERIAL_PROPERTY_TYPE_IS_VALID (type));
    return materialPropertyDesc[type].problemTypeMask;
} /* RMaterialProperty::getProblemTypeMask */


QList<RMaterialProperty::Type> RMaterialProperty::getTypes(RProblemTypeMask problemTypeMask)
{
    R_ERROR_ASSERT (R_PROBLEM_TYPE_MASK_IS_VALID (problemTypeMask));

    std::vector<RProblemType> prbTypes = RProblem::getTypes(problemTypeMask);

    std::vector<bool> bcBook;
    bcBook.resize(nTypes,false);

    for (unsigned int i=0;i<prbTypes.size();i++)
    {
        for (unsigned int j=0;j<bcBook.size();j++)
        {
            if (bcBook[j])
            {
                continue;
            }
            if (RMaterialProperty::getProblemTypeMask(RMaterialProperty::Type(j)) &prbTypes[i])
            {
                bcBook[j] = true;
            }
        }
    }

    QList<RMaterialProperty::Type> propertyTypes;
    for (unsigned int i=0;i<bcBook.size();i++)
    {
        if (bcBook[i])
        {
            propertyTypes.push_back(RMaterialProperty::Type(i));
        }
    }

    return propertyTypes;
} /* RMaterialProperty::getTypes */


RVariableType RMaterialProperty::getVariableType(RMaterialProperty::Type type)
{
    R_ERROR_ASSERT (R_MATERIAL_PROPERTY_TYPE_IS_VALID (type));
    return materialPropertyDesc[type].variableType;
} /* RMaterialProperty::getVariableType */


RMaterialProperty RMaterialProperty::fromJson(const QJsonObject &json)
{
    RMaterialProperty property;

    if (const QJsonValue v = json["keyType"]; v.isString())
    {
        property.keyType = RVariableType(v.toString().toInt());
    }

    if (const QJsonValue v = json["type"]; v.isString())
    {
        property.type = RMaterialProperty::Type(v.toString().toInt());
    }

    if (const QJsonValue v = json["table"]; v.isArray())
    {
        QJsonArray jArray = v.toArray();

        for (qsizetype i=0; i<jArray.count(); i++)
        {
            QJsonObject jItem = jArray.at(i).toObject();
            const QJsonValue vKey = jItem["key"];
            const QJsonValue vValue = jItem["value"];
            if (vKey.isDouble() && vValue.isDouble())
            {
                property.add(vKey.toDouble(),vValue.toDouble());
            }
        }
    }

    return property;
} /* RMaterialProperty::fromJson */


QJsonObject RMaterialProperty::toJson() const
{
    QJsonObject jObject;

    jObject["keyType"] = this->keyType;
    jObject["type"] = this->type;

    QJsonArray jArray;
    for (uint i=0;i<this->size();i++)
    {
        QJsonObject jItem;
        jItem["key"] = this->getKey(i);
        jItem["value"] = this->getValue(i);

        jArray.append(jItem);
    }

    jObject["table"] = jArray;

    return jObject;
} /* RMaterialProperty::toJson */


void operator ++(RMaterialProperty::Type &type, int increment)
{
    type = (RMaterialProperty::Type)((int)type+increment+1);
} /* operator ++ */
