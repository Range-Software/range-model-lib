#include "rml_support_reaction.h"

void RSupportReaction::_init(const RSupportReaction *pSupportReaction)
{
    if (pSupportReaction)
    {
        this->entityType = pSupportReaction->entityType;
        this->name = pSupportReaction->name;
        this->force = pSupportReaction->force;
    }
}

RSupportReaction::RSupportReaction()
    : entityType(R_ENTITY_GROUP_NONE)
    , force(0.0,0.0,0.0)
{
    this->_init();
}

RSupportReaction::RSupportReaction(REntityGroupType entityType, const QString &name, const RR3Vector &force)
    : entityType(entityType)
    , name(name)
    , force(force)
{
    this->_init();
}

RSupportReaction::RSupportReaction(const RSupportReaction &supportReaction)
{
    this->_init(&supportReaction);
}

RSupportReaction::~RSupportReaction()
{
}

RSupportReaction &RSupportReaction::operator =(const RSupportReaction &supportReaction)
{
    this->_init(&supportReaction);
    return (*this);
}

REntityGroupType RSupportReaction::getEntityType() const
{
    return this->entityType;
}

void RSupportReaction::setEntityType(REntityGroupType entityType)
{
    this->entityType = entityType;
}

const QString &RSupportReaction::getName() const
{
    return this->name;
}

void RSupportReaction::setName(const QString &name)
{
    this->name = name;
}

const RR3Vector &RSupportReaction::getForce() const
{
    return this->force;
}

void RSupportReaction::setForce(const RR3Vector &force)
{
    this->force = force;
}
