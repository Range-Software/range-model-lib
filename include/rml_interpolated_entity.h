#ifndef RML_INTERPOLATED_ENTITY_H
#define RML_INTERPOLATED_ENTITY_H

#include <vector>

#include "rml_interpolated_element.h"
#include "rml_entity_group.h"

class RInterpolatedEntity : public REntityGroup, public std::vector<RInterpolatedElement>
{

    private:

        //! Internal initialization function.
        void _init ( const RInterpolatedEntity *pInterpolatedEntity = nullptr );

    public:

        //! Constructor.
        RInterpolatedEntity();

        //! Copy constructor.
        RInterpolatedEntity ( const RInterpolatedEntity &interpolatedEntity );

        //! Destructor.
        ~RInterpolatedEntity ();

        //! Assignment operator.
        RInterpolatedEntity & operator = ( const RInterpolatedEntity &interpolatedEntity );

};

#endif /* RML_INTERPOLATED_ENTITY_H */
