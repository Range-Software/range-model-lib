#ifndef RML_STRESS_SETUP_H
#define RML_STRESS_SETUP_H

#include <QString>

//! Hilber-Hughes-Taylor alpha of plain Newmark (no numerical dissipation).
#define R_STRESS_HHT_ALPHA_DEFAULT   0.0
//! Strongest dissipation for which the HHT method stays unconditionally stable.
#define R_STRESS_HHT_ALPHA_MIN_VALUE (-1.0/3.0)
#define R_STRESS_HHT_ALPHA_MAX_VALUE 0.0

#define R_STRESS_TRANSIENT_TYPE_IS_VALID(_type) \
( \
    _type >= R_STRESS_TRANSIENT_DYNAMIC && \
    _type < R_STRESS_TRANSIENT_N_TYPES \
)

//! Transient stress analysis type.
typedef enum _RStressTransientType
{
    //! Elastodynamics M*u'' + C*u' + K*u = f(t), inertia and damping included.
    R_STRESS_TRANSIENT_DYNAMIC = 0,
    //! Sequence of static solutions K*u = f(t), inertia and damping neglected.
    R_STRESS_TRANSIENT_QUASI_STATIC,
    R_STRESS_TRANSIENT_N_TYPES
} RStressTransientType;

class RStressSetup
{

    protected:

        //! Transient analysis type.
        RStressTransientType transientType;
        //! Hilber-Hughes-Taylor alpha, from -1/3 to 0.
        //! Zero is plain Newmark, negative values damp high frequencies.
        double hhtAlpha;
        //! Use lumped (diagonal) mass matrix in transient analysis.
        bool lumpedMass;

    private:

        //! Internal initialization function.
        void _init(const RStressSetup *pStressSetup = nullptr);

    public:

        //! Constructor.
        RStressSetup();

        //! Copy constructor.
        RStressSetup(const RStressSetup &stressSetup);

        //! Destructor.
        ~RStressSetup();

        //! Assignment operator.
        RStressSetup &operator =(const RStressSetup &stressSetup);

        //! Return transient analysis type.
        RStressTransientType getTransientType(void) const;

        //! Set transient analysis type.
        void setTransientType(RStressTransientType transientType);

        //! Return Hilber-Hughes-Taylor alpha.
        double getHhtAlpha(void) const;

        //! Set Hilber-Hughes-Taylor alpha. Value is clamped to the stable range.
        void setHhtAlpha(double hhtAlpha);

        //! Return true if lumped mass matrix is used.
        bool getLumpedMass(void) const;

        //! Set whether lumped mass matrix is used.
        void setLumpedMass(bool lumpedMass);

        //! Convert to printable string.
        QString toString() const;

        //! Return transient analysis type name.
        static const QString &getTransientTypeName(RStressTransientType transientType);

        //! Allow RFileIO to access private members.
        friend class RFileIO;

};

#endif // RML_STRESS_SETUP_H
