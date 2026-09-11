#include <cmath>

#include <rbl_error.h>

#include "rml_acoustic_setup.h"

static QString acousticAnalysisTypeNames [R_ACOUSTIC_ANALYSIS_N_TYPES] =
{
    "Transient",
    "Harmonic"
};

void RAcousticSetup::_init(const RAcousticSetup *pAcousticSetup)
{
    if (pAcousticSetup)
    {
        this->analysisType = pAcousticSetup->analysisType;
        this->frequencyStart = pAcousticSetup->frequencyStart;
        this->frequencyStep = pAcousticSetup->frequencyStep;
        this->nFrequencies = pAcousticSetup->nFrequencies;
        this->referencePressure = pAcousticSetup->referencePressure;
        this->frequencyIndex = pAcousticSetup->frequencyIndex;
    }
}

RAcousticSetup::RAcousticSetup(void)
    : analysisType(R_ACOUSTIC_ANALYSIS_TRANSIENT)
    , frequencyStart(R_ACOUSTIC_FREQUENCY_DEFAULT_START)
    , frequencyStep(R_ACOUSTIC_FREQUENCY_DEFAULT_STEP)
    , nFrequencies(R_ACOUSTIC_FREQUENCIES_DEFAULT_N)
    , referencePressure(R_ACOUSTIC_REFERENCE_PRESSURE)
    , frequencyIndex(0)
{
    this->_init();
}

RAcousticSetup::RAcousticSetup(const RAcousticSetup &acousticSetup)
{
    this->_init(&acousticSetup);
}

RAcousticSetup::~RAcousticSetup(void)
{

}

RAcousticSetup &RAcousticSetup::operator =(const RAcousticSetup &acousticSetup)
{
    this->_init(&acousticSetup);
    return (*this);
}

RAcousticAnalysisType RAcousticSetup::getAnalysisType(void) const
{
    return this->analysisType;
}

void RAcousticSetup::setAnalysisType(RAcousticAnalysisType analysisType)
{
    this->analysisType = analysisType;
}

double RAcousticSetup::getFrequencyStart(void) const
{
    return this->frequencyStart;
}

void RAcousticSetup::setFrequencyStart(double frequencyStart)
{
    this->frequencyStart = frequencyStart;
}

double RAcousticSetup::getFrequencyStep(void) const
{
    return this->frequencyStep;
}

void RAcousticSetup::setFrequencyStep(double frequencyStep)
{
    this->frequencyStep = frequencyStep;
}

uint RAcousticSetup::getNFrequencies(void) const
{
    return this->nFrequencies;
}

void RAcousticSetup::setNFrequencies(uint nFrequencies)
{
    this->nFrequencies = nFrequencies;
}

double RAcousticSetup::getReferencePressure(void) const
{
    return this->referencePressure;
}

void RAcousticSetup::setReferencePressure(double referencePressure)
{
    this->referencePressure = referencePressure;
}

uint RAcousticSetup::getFrequencyIndex(void) const
{
    return this->frequencyIndex;
}

void RAcousticSetup::setFrequencyIndex(uint frequencyIndex)
{
    this->frequencyIndex = frequencyIndex;
}

double RAcousticSetup::getFrequency(uint frequencyIndex) const
{
    return this->frequencyStart + double(frequencyIndex) * this->frequencyStep;
}

double RAcousticSetup::getFrequency(void) const
{
    return this->getFrequency(this->frequencyIndex);
}

double RAcousticSetup::getAngularFrequency(void) const
{
    return 2.0 * std::acos(-1.0) * this->getFrequency();
}

QString RAcousticSetup::toString() const
{
    return "{ Analysis type: " + RAcousticSetup::getAnalysisTypeName(this->analysisType)
            + ", Frequency start: " + QString::number(this->frequencyStart)
            + ", Frequency step: " + QString::number(this->frequencyStep)
            + ", Number of frequencies: " + QString::number(this->nFrequencies)
            + ", Reference pressure: " + QString::number(this->referencePressure)
            + ", Frequency index: " + QString::number(this->frequencyIndex) + " }";
}

const QString &RAcousticSetup::getAnalysisTypeName(RAcousticAnalysisType analysisType)
{
    R_ERROR_ASSERT(R_ACOUSTIC_ANALYSIS_TYPE_IS_VALID(analysisType));
    return acousticAnalysisTypeNames[analysisType];
}
