#include <rbl_error.h>

#include "rml_modal_setup.h"

static QString modalMethodNames [R_MODAL_N_TYPES] =
{
    "Only most dominant mode",
    "Multiple modes"
};

void RModalSetup::_init(const RModalSetup *pModalSetup)
{
    if (pModalSetup)
    {
        this->method = pModalSetup->method;
        this->nIterations = pModalSetup->nIterations;
        this->nModesToExtract = pModalSetup->nModesToExtract;
        this->convergenceValue = pModalSetup->convergenceValue;
        this->mode = pModalSetup->mode;
        this->frequency = pModalSetup->frequency;
    }
}

RModalSetup::RModalSetup(void)
    : method(R_MODAL_MULTIPLE_MODES)
    , nIterations(R_MODAL_ITERATIONS_DEFAULT_NUMBER)
    , nModesToExtract(R_MODAL_ITERATIONS_DEFAULT_NUMBER)
    , convergenceValue(R_MODAL_CONVERGENCE_DEFAULT_VALUE)
    , mode(0)
    , frequency(0.0)
{
    this->_init();
}

RModalSetup::RModalSetup(const RModalSetup &modalSetup)
{
    this->_init(&modalSetup);
}

RModalSetup::~RModalSetup(void)
{

}

RModalSetup &RModalSetup::operator =(const RModalSetup &modalSetup)
{
    this->_init(&modalSetup);
    return (*this);
}

RModalMethod RModalSetup::getMethod(void) const
{
    return this->method;
}

void RModalSetup::setMethod(RModalMethod modalMethod)
{
    this->method = modalMethod;
}

uint RModalSetup::getNIterations(void) const
{
    return this->nIterations;
}

void RModalSetup::setNIterations(uint nModalIterations)
{
    this->nIterations = nModalIterations;
}

uint RModalSetup::getNModesToExtract(void) const
{
    return this->nModesToExtract;
}

void RModalSetup::setNModesToExtract(uint nModesToExtract)
{
    this->nModesToExtract = nModesToExtract;
}

double RModalSetup::getConvergenceValue(void) const
{
    return convergenceValue;
}

void RModalSetup::setConvergenceValue(double modalCvgValue)
{
    this->convergenceValue = modalCvgValue;
}

uint RModalSetup::getMode(void) const
{
    return this->mode;
}

void RModalSetup::setMode(uint mode)
{
    this->mode = mode;
}

double RModalSetup::getFrequency(void) const
{
    return this->frequency;
}

void RModalSetup::setFrequency(double modeFrequency)
{
    this->frequency = modeFrequency;
}

QString RModalSetup::toString() const
{
    return "{ Modal method: " + RModalSetup::getMethodName(this->method)
            + ", Number of iterations: " + QString::number(this->nIterations)
            + ", Number of modes to extract: " + QString::number(this->nModesToExtract)
            + ", Convergence value: " + QString::number(this->convergenceValue)
            + ", Mode: " + QString::number(this->mode)
            + ", Frequency: " + QString::number(this->frequency) + " }";
}

const QString &RModalSetup::getMethodName(RModalMethod modalMethod)
{
    R_ERROR_ASSERT(R_MODAL_METHOD_TYPE_IS_VALID(modalMethod));
    return modalMethodNames[modalMethod];
}
