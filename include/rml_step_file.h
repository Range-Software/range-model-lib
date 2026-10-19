#ifndef RML_STEP_FILE_H
#define RML_STEP_FILE_H

#include <QString>
#include <QByteArray>

#include <unordered_map>
#include <vector>

//! Single parameter value of a STEP (ISO 10303-21) entity.
class RStepValue
{

    public:

        enum Type
        {
            //! Unset value ($).
            Null = 0,
            //! Derived value (*).
            Derived,
            Integer,
            Real,
            String,
            //! Enumeration (.NAME.) - also booleans and logicals.
            Enum,
            //! Entity instance reference (#ID).
            Reference,
            //! Binary value ("...").
            Binary,
            //! Aggregate - list or set.
            List,
            //! Typed value - KEYWORD(value), e.g. LENGTH_MEASURE(1.0).
            Typed
        };

    public:

        //! Value type.
        Type type;
        //! Numeric value (integer or real).
        double number;
        //! Entity reference ID.
        unsigned int id;
        //! String, enumeration or typed value keyword.
        QString text;
        //! List items or typed value parameters.
        std::vector<RStepValue> items;

    public:

        //! Constructor.
        RStepValue();

        //! Return true if value is unset or derived.
        bool isNull() const;

        //! Return true if value is a number.
        bool isNumber() const;

        //! Return true if value is a list.
        bool isList() const;

        //! Return true if value is a reference.
        bool isReference() const;

        //! Return number of list items.
        size_t size() const;

        //! Return list item.
        //! Out of range or non-list values return null value.
        const RStepValue &at(size_t position) const;

        //! Return numeric value (typed values are unwrapped).
        double toDouble(double defaultValue = 0.0) const;

        //! Return integer value.
        int toInt(int defaultValue = 0) const;

        //! Return boolean value (.T. / .F.).
        bool toBool(bool defaultValue = true) const;

        //! Return reference ID (0 if not a reference).
        unsigned int toId() const;

        //! Return list of reference IDs.
        std::vector<unsigned int> toIdList() const;

        //! Return list of numbers.
        std::vector<double> toDoubleList() const;

    public:

        //! Static null value.
        static const RStepValue null;

};

//! Single (partial) entity record - keyword and parameters.
class RStepRecord
{

    public:

        //! Record keyword (upper case).
        QString keyword;
        //! Record parameters.
        std::vector<RStepValue> params;

    public:

        //! Return parameter.
        //! Out of range parameters return null value.
        const RStepValue &param(size_t position) const;

};

//! STEP entity instance.
//! Simple instance has one record, complex instance has one record for each partial entity.
class RStepEntity
{

    public:

        //! Entity ID.
        unsigned int id;
        //! Entity records.
        std::vector<RStepRecord> records;

    public:

        //! Constructor.
        RStepEntity();

        //! Return true if entity is a complex instance.
        bool isComplex() const;

        //! Return keyword of a simple instance (empty for complex instance).
        const QString &keyword() const;

        //! Return record with given keyword or nullptr if not found.
        const RStepRecord *find(const QString &keyword) const;

        //! Return true if entity contains record with given keyword.
        bool has(const QString &keyword) const;

};

//! STEP (ISO 10303-21) exchange file.
class RStepFile
{

    protected:

        //! Header records (FILE_DESCRIPTION, FILE_NAME, FILE_SCHEMA).
        std::vector<RStepRecord> header;
        //! Data section entities.
        std::unordered_map<unsigned int,RStepEntity> entities;

    public:

        //! Constructor.
        RStepFile();

        //! Read file.
        void read(const QString &fileName);

        //! Parse file content.
        void parse(const QByteArray &content);

        //! Return header record with given keyword or nullptr if not found.
        const RStepRecord *findHeader(const QString &keyword) const;

        //! Return entity with given ID or nullptr if not found.
        const RStepEntity *find(unsigned int id) const;

        //! Return const reference to all entities.
        const std::unordered_map<unsigned int,RStepEntity> &getEntities() const;

        //! Format real number.
        static QString formatReal(double value);

        //! Format string (with quotes and escaping).
        static QString formatString(const QString &value);

        //! Decode string escape sequences.
        static QString decodeString(const QByteArray &value);

};

#endif // RML_STEP_FILE_H
