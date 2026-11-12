#ifndef RML_MODEL_TETRA_MESH_H
#define RML_MODEL_TETRA_MESH_H

#include <rel_tetra_mesh_data.h>

#include "rml_model.h"

//! Input and output of the Range tetrahedral mesh generator (RTetraMesher) built from and written to RModel.
class RModelTetraMesh : public RTetraMeshData
{

    public:

        //! Constructor.
        RModelTetraMesh();

        //! Import RModel object.
        //! Nodes, points, lines and surface elements are imported, quadrilaterals as two triangles.
        //! Markers are positions of points, lines, surfaces and volumes in the model.
        //! Tetrahedra are imported only if the mesh is to be reconstructed.
        void importModel(const RModel &model, bool reconstruct, const RRVector &nodeMeshSizeValues = RRVector());

        //! Export mesh to RModel.
        void exportMesh(RModel &model, bool keepResults = true) const;

};

#endif // RML_MODEL_TETRA_MESH_H
