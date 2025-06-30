#ifndef RML_MATERIAL_PROPERTY_H
#define RML_MATERIAL_PROPERTY_H

#include <QString>
#include <QJsonObject>

#include <rbl_value_table.h>

#include "rml_variable.h"

#define R_MATERIAL_PROPERTY_TYPE_IS_VALID(_type) \
( \
    _type >= RMaterialProperty::Type::None && \
    _type < RMaterialProperty::Type::nTypes \
)

//! Material property class.
class RMaterialProperty : public RValueTable
{

    public:

        enum Type
        {
            None = 0,
            Density,
            DynamicViscosity,
            ElectricalConductivity,
            Emissivity,
            HeatCapacity,
            ModulusOfElasiticity,
            PermeabilityToFluids,
            PoissonRatio,
            RelativePermittivity,
            SoundSpeed,
            ThermalConductivity,
            ThermalExpansionCoefficient,
            Custom,
            nTypes
        };

    private:

        //! Internal initialization function.
        void _init(const RMaterialProperty *pMaterialProperty = nullptr);

        //! Set table key name.
        void setKeyName(const QString &name);

        //! Set table key units.
        void setKeyUnits(const QString &units);

        //! Set table value name.
        void setValueName(const QString &name);

        //! Set table value units.
        void setValueUnits(const QString &units);

    protected:

        //! Material table key type
        RVariableType keyType;
        //! Material property type.
        RMaterialProperty::Type type;

    public:

        //! Constructor.
        RMaterialProperty(RMaterialProperty::Type type = None);

        //! Copy constructor.
        RMaterialProperty(const RMaterialProperty &materialProperty);

        //! Destructor.
        ~RMaterialProperty();

        //! Assignment operator.
        RMaterialProperty & operator =(const RMaterialProperty &materialProperty);

        //! Equal operator.
        bool operator ==(const RMaterialProperty &materialProperty) const;

        //! Not equal operator.
        bool operator !=(const RMaterialProperty &materialProperty) const;

        //! Return material table key type
        RVariableType getKeyType() const;

        //! Return material property type.
        RMaterialProperty::Type getType() const;

        //! Set material property type.
        void setType(RMaterialProperty::Type type);

        //! Return table value name.
        const QString &getName() const;

        //! Set table value name.
        void setName(const QString &name);

        //! Return table value units.
        const QString &getUnits() const;

        //! Set table value units.
        void setUnits(const QString &units);

        //! Return material type for given material ID.
        static RMaterialProperty::Type getTypeFromId(const QString &materialId);

        //! Return material ID for specified material type.
        static QString getId(RMaterialProperty::Type type);

        //! Return material property name
        static const QString & getName(RMaterialProperty::Type type);

        //! Return material property units
        static const QString & getUnits(RMaterialProperty::Type type);

        //! Return material property initial value
        static double getInitValue(RMaterialProperty::Type type);

        //! Return problem type mask for given condition type.
        static RProblemTypeMask getProblemTypeMask(RMaterialProperty::Type type);

        //! Return list of material properties required for given problem type mask.
        static QList<RMaterialProperty::Type> getTypes(RProblemTypeMask problemTypeMask);

        //! Convert material property type to variable type.
        static RVariableType getVariableType(RMaterialProperty::Type type);

        //! Create material property object from Json.
        static RMaterialProperty fromJson(const QJsonObject &json);

        //! Create Json from material property object.
        QJsonObject toJson() const;

        //! Allow RFileIO to access private members.
        friend class RFileIO;

};

//! ++ operator
void operator ++(RMaterialProperty::Type &type, int increment);

#endif /* RML_MATERIAL_PROPERTY_H */
