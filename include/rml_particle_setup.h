#ifndef RML_PARTICLE_SETUP_H
#define RML_PARTICLE_SETUP_H

#include <QString>

//! Maximum saturation value which means that saturation is not limited.
#define R_PARTICLE_MAXIMUM_SATURATION_UNLIMITED 0.0
#define R_PARTICLE_MAXIMUM_SATURATION_MIN_VALUE 0.0
#define R_PARTICLE_MAXIMUM_SATURATION_MAX_VALUE 1.0e99
#define R_PARTICLE_DIFFUSION_COEFFICIENT_MIN_VALUE 0.0
#define R_PARTICLE_DIFFUSION_COEFFICIENT_MAX_VALUE 1.0e99

class RParticleSetup
{

    protected:

        //! Maximum possible concentration of the contaminant [kg/m^3].
        //! Value less than or equal to zero means unlimited.
        double maximumSaturation;
        //! Diffusion coefficient of the contaminant in the fluid [m^2/s].
        //! Without turbulence model this is an effective (turbulent) diffusivity.
        double diffusionCoefficient;

    private:

        //! Internal initialization function.
        void _init(const RParticleSetup *pParticleSetup = nullptr);

    public:

        //! Constructor.
        RParticleSetup();

        //! Copy constructor.
        RParticleSetup(const RParticleSetup &particleSetup);

        //! Destructor.
        ~RParticleSetup();

        //! Assignment operator.
        RParticleSetup &operator =(const RParticleSetup &particleSetup);

        //! Return maximum saturation [kg/m^3].
        double getMaximumSaturation(void) const;

        //! Set maximum saturation [kg/m^3].
        void setMaximumSaturation(double maximumSaturation);

        //! Return true if maximum saturation limit is set.
        bool getSaturationLimited(void) const;

        //! Return diffusion coefficient [m^2/s].
        double getDiffusionCoefficient(void) const;

        //! Set diffusion coefficient [m^2/s].
        void setDiffusionCoefficient(double diffusionCoefficient);

        //! Convert to printable string.
        QString toString() const;

        //! Allow RFileIO to access private members.
        friend class RFileIO;

};

#endif // RML_PARTICLE_SETUP_H
