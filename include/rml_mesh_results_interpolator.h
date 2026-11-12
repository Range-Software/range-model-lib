#ifndef RML_MESH_RESULTS_INTERPOLATOR_H
#define RML_MESH_RESULTS_INTERPOLATOR_H

#include <array>
#include <vector>

#include "rml_model.h"

//! Interpolates results of a model to a new mesh, which replaces mesh of the model.
class RMeshResultsInterpolator
{

    public:

        //! Element of a new mesh.
        struct Element
        {
            //! Positions of element nodes in a list of new nodes.
            std::array<uint,4> nodeIDs;
            //! Number of element nodes.
            uint nNodes;
            //! Entity group type which the element belongs to.
            REntityGroupTypeMask group;
        };

    private:

        //! Constructor.
        RMeshResultsInterpolator() {}

    public:

        //! Interpolate all variables of a model to nodes and elements of a new mesh.
        //! Value of a node is interpolated from nodes of an element which the node lies in.
        //! Value of an element is taken from an element of the same kind which its center lies in.
        //! Variables which are applied neither on nodes nor on elements are not returned.
        static std::vector<RVariable> interpolate(const RModel &model, const std::vector<RNode> &nodes, const std::vector<Element> &elements);

};

#endif // RML_MESH_RESULTS_INTERPOLATOR_H
