#ifndef RML_ELEMENT_SHAPE_FUNCTION_H
#define RML_ELEMENT_SHAPE_FUNCTION_H

#include <rbl_rmatrix.h>

class RElementShapeFunction
{

    protected:

        //! Shape function values.
        RRVector N;
        //! Shape function derivative values.
        RRMatrix dN;
        //! Weight factor.
        double w;

    private:

        //! Internal initialization function.
        void _init(const RElementShapeFunction *pElementShapeFunction = nullptr);

    public:

        //! Constructor.
        RElementShapeFunction(const RRVector &N, const RRMatrix &dN, double w);

        //! Copy constructor.
        RElementShapeFunction(const RElementShapeFunction &elementShapeFunction);

        //! Destructor.
        ~RElementShapeFunction();

        //! Assignment operator.
        RElementShapeFunction & operator =(const RElementShapeFunction &elementShapeFunction);

        //! Return number of integration points.
        unsigned int getNNodes(void) const;

        //! Return shape function vectors.
        const RRVector & getN(void) const;

        //! Return derivated function matrix.
        const RRMatrix & getDN(void) const;

        //! Return vector of weight factors.
        const double & getW(void) const;

};

#endif // RML_ELEMENT_SHAPE_FUNCTION_H
