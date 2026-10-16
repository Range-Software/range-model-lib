#include <algorithm>

#include <rbl_error.h>

#include "rml_stress_setup.h"

static QString stressTransientTypeNames [R_STRESS_TRANSIENT_N_TYPES] =
{
    "Dynamic",
    "Quasi-static"
};

void RStressSetup::_init(const RStressSetup *pStressSetup)
{
    if (pStressSetup)
    {
        this->transientType = pStressSetup->transientType;
        this->hhtAlpha = pStressSetup->hhtAlpha;
        this->lumpedMass = pStressSetup->lumpedMass;
    }
}

RStressSetup::RStressSetup(void)
    : transientType(R_STRESS_TRANSIENT_DYNAMIC)
    , hhtAlpha(R_STRESS_HHT_ALPHA_DEFAULT)
    , lumpedMass(false)
{
    this->_init();
}

RStressSetup::RStressSetup(const RStressSetup &stressSetup)
{
    this->_init(&stressSetup);
}

RStressSetup::~RStressSetup(void)
{

}

RStressSetup &RStressSetup::operator =(const RStressSetup &stressSetup)
{
    this->_init(&stressSetup);
    return (*this);
}

RStressTransientType RStressSetup::getTransientType(void) const
{
    return this->transientType;
}

void RStressSetup::setTransientType(RStressTransientType transientType)
{
    R_ERROR_ASSERT(R_STRESS_TRANSIENT_TYPE_IS_VALID(transientType));
    this->transientType = transientType;
}

double RStressSetup::getHhtAlpha(void) const
{
    return this->hhtAlpha;
}

void RStressSetup::setHhtAlpha(double hhtAlpha)
{
    this->hhtAlpha = std::min(std::max(hhtAlpha,R_STRESS_HHT_ALPHA_MIN_VALUE),R_STRESS_HHT_ALPHA_MAX_VALUE);
}

bool RStressSetup::getLumpedMass(void) const
{
    return this->lumpedMass;
}

void RStressSetup::setLumpedMass(bool lumpedMass)
{
    this->lumpedMass = lumpedMass;
}

QString RStressSetup::toString() const
{
    return "{ Transient type: " + RStressSetup::getTransientTypeName(this->transientType)
            + ", HHT alpha: " + QString::number(this->hhtAlpha)
            + ", Lumped mass: " + QString(this->lumpedMass?"True":"False") + " }";
}

const QString &RStressSetup::getTransientTypeName(RStressTransientType transientType)
{
    R_ERROR_ASSERT(R_STRESS_TRANSIENT_TYPE_IS_VALID(transientType));
    return stressTransientTypeNames[transientType];
}
