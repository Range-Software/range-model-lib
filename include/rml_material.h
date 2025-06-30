#ifndef RML_MATERIAL_H
#define RML_MATERIAL_H

#include <QString>
#include <QUuid>
#include <QJsonObject>

#include <rbl_version.h>

#include "rml_material_property.h"

//! Material class
class RMaterial
{

    public:

        enum State
        {
            None = 0,
            Gas,
            Liquid,
            Solid,
            nTypes
        };

    private:

        //! Internal initialization function.
        void _init(const RMaterial *pMaterial = nullptr);

    protected:

        //! Material ID.
        QUuid id;
        //! Material name.
        QString name;
        //! Material state.
        State state;
        //! Material properties.
        QList<RMaterialProperty> properties;

    public:

        static const RVersion version;

    public:

        //! Constructor.
        RMaterial(State state = None);

        //! Copy constructor.
        RMaterial(const RMaterial &material);

        //! Destructor.
        ~RMaterial();

        //! Assignment operator.
        RMaterial &operator =(const RMaterial &material);

        //! Check whether the material properties are same.
        bool propertiesEqual(const RMaterial &material) const;

        //! Return material ID.
        QUuid getID() const;

        //! Set material ID.
        void setID(QUuid id);

        //! Return material name.
        const QString &getName() const;

        //! Set material name.
        void setName(const QString &name);

        //! Return material state.
        State getState() const;

        //! Set material state.
        void setState(State state);

        //! Return number of properties.
        uint size() const;

        //! Add new material property.
        //! If a property with such type already exist an old
        //! property will be replaced.
        //! Return true if new property was added false on replace.
        bool add(const RMaterialProperty &property);

        //! Return const reference to material property at given position.
        const RMaterialProperty &get(uint position) const;

        //! Return reference to material property at given position.
        RMaterialProperty &get(uint position);

        //! Remove material property at given position.
        void remove(uint position);

        //! Clear all data.
        void clear();

        //! Find material property position by its type.
        //! If no such property is present in material number of properties is returned.
        uint findPosition(RMaterialProperty::Type type) const;

        //! Check if material contains given list of properties.
        bool hasProperties(const QList<RMaterialProperty::Type> &propertyTypes) const;

        //! Generate material with all material properties and their default values.
        static RMaterial generateDefault();

        //! Create material object from Json.
        static RMaterial fromJson(const QJsonObject &json);

        //! Create Json from material object.
        QJsonObject toJson() const;

        //! Allow RFileIO to access private members.
        friend class RFileIO;

        //! Read material from the file.
        void read(const QString &fileName);

        //! Write material to the file.
        void write(const QString &fileName) const;

        //! Import legacy material.
        void import(const QString &fileName);

        //! Find material file name.
        QString findMaterialFileName(bool binary = true) const;

        //! Validate material's properties against problem type.
        bool validForProblemType(RProblemTypeMask problemTypeMask) const;

        //! Return default file extension.
        static QString getDefaultFileExtension(bool binary = true);

    protected:

        //! Read from the ASCII file.
        void readAscii(const QString &fileName);

        //! Read from the binary file.
        void readBinary(const QString &fileName);

        //! Write to the ASCII file.
        void writeAscii(const QString &fileName) const;

        //! Write to the binary file.
        void writeBinary(const QString &fileName) const;

};

#endif /* RML_MATERIAL_H */
