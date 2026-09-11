#ifndef RML_ACOUSTIC_SETUP_H
#define RML_ACOUSTIC_SETUP_H

#include <QString>

#define R_ACOUSTIC_FREQUENCY_DEFAULT_START 20.0
#define R_ACOUSTIC_FREQUENCY_DEFAULT_STEP  20.0
#define R_ACOUSTIC_FREQUENCY_MIN_VALUE     0.0
#define R_ACOUSTIC_FREQUENCY_MAX_VALUE     1.0e9
#define R_ACOUSTIC_FREQUENCIES_DEFAULT_N   1
#define R_ACOUSTIC_FREQUENCIES_MIN_N       1
#define R_ACOUSTIC_FREQUENCIES_MAX_N       10000
//! Reference sound pressure in air.
#define R_ACOUSTIC_REFERENCE_PRESSURE      2.0e-5

#define R_ACOUSTIC_ANALYSIS_TYPE_IS_VALID(_type) \
( \
    _type >= R_ACOUSTIC_ANALYSIS_TRANSIENT && \
    _type < R_ACOUSTIC_ANALYSIS_N_TYPES \
)

//! Acoustic analysis type.
typedef enum _RAcousticAnalysisType
{
    R_ACOUSTIC_ANALYSIS_TRANSIENT = 0,
    R_ACOUSTIC_ANALYSIS_HARMONIC,
    R_ACOUSTIC_ANALYSIS_N_TYPES
} RAcousticAnalysisType;

class RAcousticSetup
{

    protected:

        //! Acoustic analysis type.
        RAcousticAnalysisType analysisType;
        //! First frequency of the harmonic sweep [Hz].
        double frequencyStart;
        //! Frequency increment of the harmonic sweep [Hz].
        double frequencyStep;
        //! Number of frequencies in the harmonic sweep.
        uint nFrequencies;
        //! Reference pressure used to evaluate the sound pressure level [Pa].
        double referencePressure;
        //! Currently solved frequency index.
        uint frequencyIndex;

    private:

        //! Internal initialization function.
        void _init(const RAcousticSetup *pAcousticSetup = nullptr);

    public:

        //! Constructor.
        RAcousticSetup();

        //! Copy constructor.
        RAcousticSetup(const RAcousticSetup &acousticSetup);

        //! Destructor.
        ~RAcousticSetup();

        //! Assignment operator.
        RAcousticSetup &operator =(const RAcousticSetup &acousticSetup);

        //! Return acoustic analysis type.
        RAcousticAnalysisType getAnalysisType(void) const;

        //! Set acoustic analysis type.
        void setAnalysisType(RAcousticAnalysisType analysisType);

        //! Return first frequency of the harmonic sweep.
        double getFrequencyStart(void) const;

        //! Set first frequency of the harmonic sweep.
        void setFrequencyStart(double frequencyStart);

        //! Return frequency increment of the harmonic sweep.
        double getFrequencyStep(void) const;

        //! Set frequency increment of the harmonic sweep.
        void setFrequencyStep(double frequencyStep);

        //! Return number of frequencies in the harmonic sweep.
        uint getNFrequencies(void) const;

        //! Set number of frequencies in the harmonic sweep.
        void setNFrequencies(uint nFrequencies);

        //! Return reference pressure used to evaluate the sound pressure level.
        double getReferencePressure(void) const;

        //! Set reference pressure used to evaluate the sound pressure level.
        void setReferencePressure(double referencePressure);

        //! Return currently solved frequency index.
        uint getFrequencyIndex(void) const;

        //! Set currently solved frequency index.
        void setFrequencyIndex(uint frequencyIndex);

        //! Return frequency for given index [Hz].
        double getFrequency(uint frequencyIndex) const;

        //! Return currently solved frequency [Hz].
        double getFrequency(void) const;

        //! Return currently solved angular frequency [rad/s].
        double getAngularFrequency(void) const;

        //! Convert to printable string.
        QString toString() const;

        //! Return acoustic analysis type name.
        static const QString &getAnalysisTypeName(RAcousticAnalysisType analysisType);

        //! Allow RFileIO to access private members.
        friend class RFileIO;

};

#endif // RML_ACOUSTIC_SETUP_H
