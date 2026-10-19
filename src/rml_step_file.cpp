#include <QFile>

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include <rbl_error.h>

#include "rml_step_file.h"

const RStepValue RStepValue::null;

RStepValue::RStepValue()
    : type(Null)
    , number(0.0)
    , id(0)
{

}

bool RStepValue::isNull() const
{
    return (this->type == Null || this->type == Derived);
}

bool RStepValue::isNumber() const
{
    return (this->type == Integer || this->type == Real);
}

bool RStepValue::isList() const
{
    return (this->type == List);
}

bool RStepValue::isReference() const
{
    return (this->type == Reference);
}

size_t RStepValue::size() const
{
    return (this->type == List) ? this->items.size() : 0;
}

const RStepValue &RStepValue::at(size_t position) const
{
    if (this->type != List || position >= this->items.size())
    {
        return RStepValue::null;
    }
    return this->items[position];
}

double RStepValue::toDouble(double defaultValue) const
{
    if (this->isNumber())
    {
        return this->number;
    }
    if (this->type == Typed && this->items.size() == 1)
    {
        return this->items[0].toDouble(defaultValue);
    }
    return defaultValue;
}

int RStepValue::toInt(int defaultValue) const
{
    return this->isNumber() ? int(std::lround(this->number)) : defaultValue;
}

bool RStepValue::toBool(bool defaultValue) const
{
    if (this->type == Enum)
    {
        if (this->text == "T")
        {
            return true;
        }
        if (this->text == "F")
        {
            return false;
        }
    }
    return defaultValue;
}

unsigned int RStepValue::toId() const
{
    return (this->type == Reference) ? this->id : 0;
}

std::vector<unsigned int> RStepValue::toIdList() const
{
    std::vector<unsigned int> ids;
    if (this->type == List)
    {
        ids.reserve(this->items.size());
        for (const RStepValue &item : this->items)
        {
            if (item.type == Reference)
            {
                ids.push_back(item.id);
            }
        }
    }
    return ids;
}

std::vector<double> RStepValue::toDoubleList() const
{
    std::vector<double> values;
    if (this->type == List)
    {
        values.reserve(this->items.size());
        for (const RStepValue &item : this->items)
        {
            values.push_back(item.toDouble());
        }
    }
    return values;
}

const RStepValue &RStepRecord::param(size_t position) const
{
    return (position < this->params.size()) ? this->params[position] : RStepValue::null;
}

RStepEntity::RStepEntity()
    : id(0)
{

}

bool RStepEntity::isComplex() const
{
    return (this->records.size() > 1);
}

const QString &RStepEntity::keyword() const
{
    static const QString empty;
    return (this->records.size() == 1) ? this->records[0].keyword : empty;
}

const RStepRecord *RStepEntity::find(const QString &keyword) const
{
    for (const RStepRecord &record : this->records)
    {
        if (record.keyword == keyword)
        {
            return &record;
        }
    }
    return nullptr;
}

bool RStepEntity::has(const QString &keyword) const
{
    return (this->find(keyword) != nullptr);
}

namespace
{

//! Recursive descent parser of the exchange structure.
class StepParser
{

    public:

        const char *begin;
        const char *p;
        const char *end;

        StepParser(const QByteArray &content)
            : begin(content.constData())
            , p(content.constData())
            , end(content.constData() + content.size())
        {

        }

        [[noreturn]] void fail(const char *message) const
        {
            int line = 1;
            for (const char *c=this->begin;c<this->p && c<this->end;c++)
            {
                if (*c == '\n')
                {
                    line++;
                }
            }
            throw RError(RError::Type::InvalidFileFormat,R_ERROR_REF,"STEP syntax error on line %d: %s",line,message);
        }

        void skipSpace()
        {
            while (this->p < this->end)
            {
                char c = *this->p;
                if (c == ' ' || c == '\t' || c == '\r' || c == '\n' || c == '\f' || c == '\v')
                {
                    this->p++;
                }
                else if (c == '/' && this->p + 1 < this->end && this->p[1] == '*')
                {
                    const char *e = this->p + 2;
                    while (e + 1 < this->end && !(e[0] == '*' && e[1] == '/'))
                    {
                        e++;
                    }
                    this->p = (e + 1 < this->end) ? e + 2 : this->end;
                }
                else
                {
                    break;
                }
            }
        }

        bool atEnd()
        {
            this->skipSpace();
            return (this->p >= this->end);
        }

        bool peek(char c)
        {
            this->skipSpace();
            return (this->p < this->end && *this->p == c);
        }

        void expect(char c)
        {
            this->skipSpace();
            if (this->p >= this->end || *this->p != c)
            {
                char message[32];
                snprintf(message,sizeof(message),"expected '%c'",c);
                this->fail(message);
            }
            this->p++;
        }

        static bool isKeywordStart(char c)
        {
            return ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || c == '_' || c == '!');
        }

        static bool isKeywordChar(char c)
        {
            return ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_' || c == '-' || c == '!');
        }

        QString parseKeyword()
        {
            this->skipSpace();
            const char *start = this->p;
            if (this->p >= this->end || !StepParser::isKeywordStart(*this->p))
            {
                this->fail("expected keyword");
            }
            while (this->p < this->end && StepParser::isKeywordChar(*this->p))
            {
                this->p++;
            }
            return QString::fromLatin1(start,int(this->p - start)).toUpper();
        }

        unsigned int parseId()
        {
            this->expect('#');
            unsigned int id = 0;
            const char *start = this->p;
            while (this->p < this->end && *this->p >= '0' && *this->p <= '9')
            {
                id = id * 10 + unsigned(*this->p - '0');
                this->p++;
            }
            if (this->p == start)
            {
                this->fail("expected entity instance name");
            }
            return id;
        }

        void parseList(std::vector<RStepValue> &items)
        {
            this->expect('(');
            if (this->peek(')'))
            {
                this->p++;
                return;
            }
            while (true)
            {
                items.emplace_back();
                this->parseValue(items.back());
                this->skipSpace();
                if (this->p >= this->end)
                {
                    this->fail("unterminated list");
                }
                if (*this->p == ',')
                {
                    this->p++;
                }
                else if (*this->p == ')')
                {
                    this->p++;
                    return;
                }
                else
                {
                    this->fail("expected ',' or ')'");
                }
            }
        }

        void parseValue(RStepValue &value)
        {
            this->skipSpace();
            if (this->p >= this->end)
            {
                this->fail("unexpected end of file");
            }
            char c = *this->p;
            if (c == '$')
            {
                value.type = RStepValue::Null;
                this->p++;
            }
            else if (c == '*')
            {
                value.type = RStepValue::Derived;
                this->p++;
            }
            else if (c == '#')
            {
                value.type = RStepValue::Reference;
                value.id = this->parseId();
            }
            else if (c == '\'')
            {
                this->p++;
                QByteArray raw;
                while (true)
                {
                    if (this->p >= this->end)
                    {
                        this->fail("unterminated string");
                    }
                    char s = *this->p++;
                    if (s == '\'')
                    {
                        if (this->p < this->end && *this->p == '\'')
                        {
                            raw.append('\'');
                            this->p++;
                            continue;
                        }
                        break;
                    }
                    // Line breaks inserted by writers are not part of the value.
                    if (s != '\r' && s != '\n')
                    {
                        raw.append(s);
                    }
                }
                value.type = RStepValue::String;
                value.text = RStepFile::decodeString(raw);
            }
            else if (c == '"')
            {
                this->p++;
                const char *start = this->p;
                while (this->p < this->end && *this->p != '"')
                {
                    this->p++;
                }
                if (this->p >= this->end)
                {
                    this->fail("unterminated binary");
                }
                value.type = RStepValue::Binary;
                value.text = QString::fromLatin1(start,int(this->p - start));
                this->p++;
            }
            else if (c == '.' && this->p + 1 < this->end && StepParser::isKeywordStart(this->p[1]))
            {
                this->p++;
                const char *start = this->p;
                while (this->p < this->end && *this->p != '.')
                {
                    this->p++;
                }
                if (this->p >= this->end)
                {
                    this->fail("unterminated enumeration");
                }
                value.type = RStepValue::Enum;
                value.text = QString::fromLatin1(start,int(this->p - start)).toUpper();
                this->p++;
            }
            else if ((c >= '0' && c <= '9') || c == '-' || c == '+' || c == '.')
            {
                const char *start = this->p;
                bool isReal = false;
                this->p++;
                while (this->p < this->end)
                {
                    char d = *this->p;
                    if (d >= '0' && d <= '9')
                    {
                        this->p++;
                    }
                    else if (d == '.' || d == 'E' || d == 'e')
                    {
                        isReal = true;
                        this->p++;
                        if ((d == 'E' || d == 'e') && this->p < this->end && (*this->p == '-' || *this->p == '+'))
                        {
                            this->p++;
                        }
                    }
                    else
                    {
                        break;
                    }
                }
                value.type = isReal ? RStepValue::Real : RStepValue::Integer;
                value.number = std::strtod(QByteArray(start,int(this->p - start)).constData(),nullptr);
            }
            else if (c == '(')
            {
                value.type = RStepValue::List;
                this->parseList(value.items);
            }
            else if (StepParser::isKeywordStart(c))
            {
                value.type = RStepValue::Typed;
                value.text = this->parseKeyword();
                this->parseList(value.items);
            }
            else
            {
                this->fail("invalid parameter");
            }
        }

        void parseRecord(RStepRecord &record)
        {
            record.keyword = this->parseKeyword();
            this->parseList(record.params);
        }

        //! Parse section content until ENDSEC.
        void parseHeaderSection(std::vector<RStepRecord> &header)
        {
            while (true)
            {
                if (this->atEnd())
                {
                    this->fail("missing ENDSEC");
                }
                const char *mark = this->p;
                QString keyword = this->parseKeyword();
                if (keyword == "ENDSEC")
                {
                    this->expect(';');
                    return;
                }
                this->p = mark;
                header.emplace_back();
                this->parseRecord(header.back());
                this->expect(';');
            }
        }

        void parseDataSection(std::unordered_map<unsigned int,RStepEntity> &entities)
        {
            while (true)
            {
                if (this->atEnd())
                {
                    this->fail("missing ENDSEC");
                }
                if (*this->p != '#')
                {
                    QString keyword = this->parseKeyword();
                    if (keyword == "ENDSEC")
                    {
                        this->expect(';');
                        return;
                    }
                    this->fail("expected entity instance");
                }
                RStepEntity entity;
                entity.id = this->parseId();
                this->expect('=');
                if (this->peek('('))
                {
                    // Complex entity instance.
                    this->p++;
                    while (!this->peek(')'))
                    {
                        entity.records.emplace_back();
                        this->parseRecord(entity.records.back());
                    }
                    this->p++;
                }
                else
                {
                    entity.records.emplace_back();
                    this->parseRecord(entity.records.back());
                }
                this->expect(';');
                unsigned int id = entity.id;
                entities[id] = std::move(entity);
            }
        }

};

} // namespace

RStepFile::RStepFile()
{

}

void RStepFile::read(const QString &fileName)
{
    if (fileName.isEmpty())
    {
        throw RError(RError::Type::InvalidFileName,R_ERROR_REF,"No file name was provided.");
    }

    QFile file(fileName);

    if (!file.open(QIODevice::ReadOnly))
    {
        throw RError(RError::Type::OpenFile,R_ERROR_REF,"Failed to open the file \'%s\'.",fileName.toUtf8().constData());
    }

    QByteArray content = file.readAll();
    if (file.error() != QFile::NoError)
    {
        throw RError(RError::Type::ReadFile,R_ERROR_REF,"Failed to read from file \'%s\'.",fileName.toUtf8().constData());
    }
    file.close();

    this->parse(content);
}

void RStepFile::parse(const QByteArray &content)
{
    this->header.clear();
    this->entities.clear();

    StepParser parser(content);

    QString keyword = parser.parseKeyword();
    if (keyword != "ISO-10303-21")
    {
        throw RError(RError::Type::InvalidFileFormat,R_ERROR_REF,"Not a STEP (ISO 10303-21) file.");
    }
    parser.expect(';');

    bool hasData = false;
    while (!parser.atEnd())
    {
        keyword = parser.parseKeyword();
        if (keyword == "HEADER")
        {
            parser.expect(';');
            parser.parseHeaderSection(this->header);
        }
        else if (keyword == "DATA")
        {
            // Later editions allow named data sections - DATA('name',('schema'));
            if (parser.peek('('))
            {
                std::vector<RStepValue> dummy;
                parser.parseList(dummy);
            }
            parser.expect(';');
            parser.parseDataSection(this->entities);
            hasData = true;
        }
        else if (keyword == "ANCHOR" || keyword == "REFERENCE" || keyword == "SIGNATURE")
        {
            // Sections of later editions which do not carry geometry - skip.
            parser.expect(';');
            while (!parser.atEnd())
            {
                const char *mark = parser.p;
                if (parser.parseKeyword() == "ENDSEC")
                {
                    parser.expect(';');
                    break;
                }
                parser.p = mark;
                while (parser.p < parser.end && *parser.p != ';')
                {
                    parser.p++;
                }
                parser.p++;
            }
        }
        else if (keyword == "END-ISO-10303-21")
        {
            parser.expect(';');
            break;
        }
        else
        {
            parser.fail("unknown section");
        }
    }

    if (!hasData)
    {
        throw RError(RError::Type::InvalidFileFormat,R_ERROR_REF,"STEP file does not contain DATA section.");
    }
}

const RStepRecord *RStepFile::findHeader(const QString &keyword) const
{
    for (const RStepRecord &record : this->header)
    {
        if (record.keyword == keyword)
        {
            return &record;
        }
    }
    return nullptr;
}

const RStepEntity *RStepFile::find(unsigned int id) const
{
    auto iter = this->entities.find(id);
    return (iter == this->entities.end()) ? nullptr : &iter->second;
}

const std::unordered_map<unsigned int, RStepEntity> &RStepFile::getEntities() const
{
    return this->entities;
}

QString RStepFile::formatReal(double value)
{
    if (!std::isfinite(value))
    {
        value = 0.0;
    }
    QString text = QString::number(value,'G',15);
    // Real numbers must always contain decimal point.
    if (!text.contains('.'))
    {
        int ePos = text.indexOf('E');
        if (ePos < 0)
        {
            text += '.';
        }
        else
        {
            text.insert(ePos,'.');
        }
    }
    return text;
}

QString RStepFile::formatString(const QString &value)
{
    QString text("\'");
    for (QChar c : value)
    {
        ushort code = c.unicode();
        if (c == '\'')
        {
            text += "\'\'";
        }
        else if (c == '\\')
        {
            text += "\\\\";
        }
        else if (code >= 32 && code < 127)
        {
            text += c;
        }
        else
        {
            text += QString("\\X2\\%1\\X0\\").arg(code,4,16,QChar('0')).toUpper();
        }
    }
    text += '\'';
    return text;
}

QString RStepFile::decodeString(const QByteArray &value)
{
    if (!value.contains('\\'))
    {
        return QString::fromLatin1(value);
    }

    QString text;
    int n = value.size();
    int i = 0;
    while (i < n)
    {
        char c = value[i];
        if (c != '\\')
        {
            text += QChar(uchar(c));
            i++;
            continue;
        }
        if (value.mid(i,2) == "\\\\")
        {
            text += '\\';
            i += 2;
        }
        else if (value.mid(i,3) == "\\X\\" && i + 5 <= n)
        {
            text += QChar(ushort(value.mid(i+3,2).toUShort(nullptr,16)));
            i += 5;
        }
        else if (value.mid(i,4) == "\\X2\\")
        {
            i += 4;
            while (i + 4 <= n && value[i] != '\\')
            {
                text += QChar(ushort(value.mid(i,4).toUShort(nullptr,16)));
                i += 4;
            }
            if (value.mid(i,4) == "\\X0\\")
            {
                i += 4;
            }
        }
        else if (value.mid(i,4) == "\\X4\\")
        {
            i += 4;
            while (i + 8 <= n && value[i] != '\\')
            {
                char32_t code = char32_t(value.mid(i,8).toUInt(nullptr,16));
                text += QString::fromUcs4(&code,1);
                i += 8;
            }
            if (value.mid(i,4) == "\\X0\\")
            {
                i += 4;
            }
        }
        else if (value.mid(i,3) == "\\S\\" && i + 3 < n)
        {
            text += QChar(ushort(uchar(value[i+3]) + 128));
            i += 4;
        }
        else if (value.mid(i,2) == "\\P" && i + 3 < n && value[i+3] == '\\')
        {
            // Code page selection - ignored.
            i += 4;
        }
        else
        {
            text += '\\';
            i++;
        }
    }
    return text;
}
