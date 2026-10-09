#include "rml_particle_setup.h"

void RParticleSetup::_init(const RParticleSetup *pParticleSetup)
{
    if (pParticleSetup)
    {
        this->maximumSaturation = pParticleSetup->maximumSaturation;
        this->diffusionCoefficient = pParticleSetup->diffusionCoefficient;
    }
}

RParticleSetup::RParticleSetup(void)
    : maximumSaturation(R_PARTICLE_MAXIMUM_SATURATION_UNLIMITED)
    , diffusionCoefficient(0.0)
{
    this->_init();
}

RParticleSetup::RParticleSetup(const RParticleSetup &particleSetup)
{
    this->_init(&particleSetup);
}

RParticleSetup::~RParticleSetup(void)
{

}

RParticleSetup &RParticleSetup::operator =(const RParticleSetup &particleSetup)
{
    this->_init(&particleSetup);
    return (*this);
}

double RParticleSetup::getMaximumSaturation(void) const
{
    return this->maximumSaturation;
}

void RParticleSetup::setMaximumSaturation(double maximumSaturation)
{
    this->maximumSaturation = maximumSaturation;
}

bool RParticleSetup::getSaturationLimited(void) const
{
    return (this->maximumSaturation > 0.0);
}

double RParticleSetup::getDiffusionCoefficient(void) const
{
    return this->diffusionCoefficient;
}

void RParticleSetup::setDiffusionCoefficient(double diffusionCoefficient)
{
    this->diffusionCoefficient = diffusionCoefficient;
}

QString RParticleSetup::toString() const
{
    return "{ Maximum saturation: " + QString::number(this->maximumSaturation)
            + ", Diffusion coefficient: " + QString::number(this->diffusionCoefficient) + " }";
}
