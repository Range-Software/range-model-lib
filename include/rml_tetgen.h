#ifndef RTETGEN_H
#define RTETGEN_H

#include <tetgen.h>

#include "rml_model.h"

class RTetGen : public tetgenio
{

    public:

        //! Constructor.
        explicit RTetGen(const RModel &model = RModel());

        //! Import RModel object.
        void importModel(const RModel &model, bool reconstruct, const RRVector &nodeMeshSizeValues = RRVector());

        //! Export mesh to RModel.
        void exportMesh(RModel &model, bool keepResults = true) const;

};

#endif /* RTETGEN_H */
