#include <cmath>
#include <algorithm>
#include <limits>

#include <rbl_logger.h>
#include <rbl_progress.h>

#include "rml_mesh_results_interpolator.h"

//! Element which results of a position are taken from.
struct ResultsLocation
{
    //! Element ID, RConstants::eod if there is none.
    uint elementID = RConstants::eod;
    //! Weights of element nodes.
    RRVector ratios;
};

//! Finds elements of a mesh which results are taken from.
class ResultsLocator
{
    private:

        const RModel &model;
        //! Element bounding boxes: min X, Y, Z and max X, Y, Z.
        std::vector<std::array<double,6>> boxes;
        //! Distance below which a position is considered to be on an element.
        double tolerance;
        //! Entity group types which have elements.
        REntityGroupTypeMask availableGroups;

    public:

        explicit ResultsLocator(const RModel &model)
            : model(model)
            , boxes(model.getNElements())
            , tolerance(0.0)
            , availableGroups(REntityGroupTypeMask(0))
        {
            std::array<double,3> modelMin = {0.0,0.0,0.0};
            std::array<double,3> modelMax = {0.0,0.0,0.0};
            for (uint i=0;i<model.getNNodes();i++)
            {
                const RNode &rNode = model.getNode(i);
                const std::array<double,3> position = {rNode.getX(),rNode.getY(),rNode.getZ()};
                for (uint j=0;j<3;j++)
                {
                    modelMin[j] = (i == 0) ? position[j] : std::min(modelMin[j],position[j]);
                    modelMax[j] = (i == 0) ? position[j] : std::max(modelMax[j],position[j]);
                }
            }
            this->tolerance = 1.0e-10 * std::hypot(modelMax[0]-modelMin[0],modelMax[1]-modelMin[1],modelMax[2]-modelMin[2]);

            for (uint i=0;i<model.getNElements();i++)
            {
                const RElement &rElement = model.getElement(i);
                this->availableGroups = REntityGroupTypeMask(this->availableGroups | RElementGroup::getGroupType(rElement.getType()));
                std::array<double,6> &rBox = this->boxes[i];
                for (uint j=0;j<rElement.size();j++)
                {
                    const RNode &rNode = model.getNode(rElement.getNodeId(j));
                    const std::array<double,3> position = {rNode.getX(),rNode.getY(),rNode.getZ()};
                    for (uint k=0;k<3;k++)
                    {
                        rBox[k] = (j == 0) ? position[k] : std::min(rBox[k],position[k]);
                        rBox[k+3] = (j == 0) ? position[k] : std::max(rBox[k+3],position[k]);
                    }
                }
            }
        }

        //! Find element of given groups containing given position or being the closest one to it.
        //! Groups without elements are replaced by all elements.
        ResultsLocation find(const RNode &rNode, REntityGroupTypeMask groups) const
        {
            if ((this->availableGroups & groups) == 0)
            {
                groups = R_ENTITY_GROUP_ELEMENT;
            }

            ResultsLocation location;
            double minDistance = std::numeric_limits<double>::max();
            RRVector ratios;

            // Rounding can put a position just outside of all elements, or the new mesh
            // can reach further than the old one. Then all elements are searched.
            for (bool useBoxes : {true,false})
            {
                for (uint i=0;i<this->model.getNElements();i++)
                {
                    const RElement &rElement = this->model.getElement(i);
                    if ((RElementGroup::getGroupType(rElement.getType()) & groups) == 0)
                    {
                        continue;
                    }
                    if (useBoxes && !this->isInsideBox(i,rNode))
                    {
                        continue;
                    }
                    const double distance = rElement.findClosestPoint(this->model.getNodes(),rNode,ratios);
                    if (distance < minDistance)
                    {
                        minDistance = distance;
                        location.elementID = i;
                        location.ratios = ratios;
                        if (minDistance <= this->tolerance)
                        {
                            return location;
                        }
                    }
                }
            }
            return location;
        }

    private:

        bool isInsideBox(uint elementID, const RNode &rNode) const
        {
            const std::array<double,6> &rBox = this->boxes[elementID];
            return rNode.getX() >= rBox[0] - this->tolerance && rNode.getX() <= rBox[3] + this->tolerance
                && rNode.getY() >= rBox[1] - this->tolerance && rNode.getY() <= rBox[4] + this->tolerance
                && rNode.getZ() >= rBox[2] - this->tolerance && rNode.getZ() <= rBox[5] + this->tolerance;
        }
};

static std::vector<ResultsLocation> findNodeLocations(const std::vector<RNode> &nodes, const ResultsLocator &locator)
{
    std::vector<ResultsLocation> locations(nodes.size());

#pragma omp parallel for default(shared)
    for (int64_t i=0;i<int64_t(nodes.size());i++)
    {
        locations[size_t(i)] = locator.find(nodes[size_t(i)],R_ENTITY_GROUP_ELEMENT);
    }
    return locations;
}

static std::vector<ResultsLocation> findElementLocations(const std::vector<RNode> &nodes,
                                                         const std::vector<RMeshResultsInterpolator::Element> &elements,
                                                         const ResultsLocator &locator)
{
    std::vector<ResultsLocation> locations(elements.size());

#pragma omp parallel for default(shared)
    for (int64_t i=0;i<int64_t(elements.size());i++)
    {
        const RMeshResultsInterpolator::Element &rElement = elements[size_t(i)];
        double x = 0.0, y = 0.0, z = 0.0;
        for (uint j=0;j<rElement.nNodes;j++)
        {
            const RNode &rNode = nodes[rElement.nodeIDs[j]];
            x += rNode.getX();
            y += rNode.getY();
            z += rNode.getZ();
        }
        // Element takes value from the same kind of element, as they can have different results.
        locations[size_t(i)] = locator.find(RNode(x/rElement.nNodes,y/rElement.nNodes,z/rElement.nNodes),
                                            rElement.group);
    }
    return locations;
}

std::vector<RVariable> RMeshResultsInterpolator::interpolate(const RModel &model, const std::vector<RNode> &nodes, const std::vector<Element> &elements)
{
    std::vector<RVariable> variables;

    RLogger::info("Interpolating results\n");
    RLogger::indent();

    const ResultsLocator locator(model);

    // Variables share the same positions, so each position is searched only once.
    std::vector<ResultsLocation> nodeLocations;
    std::vector<ResultsLocation> elementLocations;

    RProgressInitialize("Interpolating results");
    for (uint i=0;i<model.getNVariables();i++)
    {
        RProgressPrint(i,model.getNVariables());
        RVariable variable = model.getVariable(i);
        RLogger::info("Interpolating %s\n",variable.getName().toUtf8().constData());
        RLogger::indent();
        const RVariable &rOldVariable = model.getVariable(i);
        if (rOldVariable.getApplyType() == R_VARIABLE_APPLY_NODE)
        {
            if (nodeLocations.empty())
            {
                nodeLocations = findNodeLocations(nodes,locator);
            }
            variable.resize(variable.getNVectors(),uint(nodes.size()),true);
#pragma omp parallel for default(shared)
            for (int64_t j=0;j<int64_t(nodes.size());j++)
            {
                const ResultsLocation &rLocation = nodeLocations[size_t(j)];
                if (rLocation.elementID == RConstants::eod)
                {
                    continue;
                }
                const RElement &rElement = model.getElement(rLocation.elementID);
                for (uint k=0;k<variable.getNVectors();k++)
                {
                    double value = 0.0;
                    for (uint l=0;l<rLocation.ratios.size();l++)
                    {
                        value += rLocation.ratios[l] * rOldVariable.getValue(k,rElement.getNodeId(l));
                    }
                    variable.setValue(k,uint(j),value);
                }
            }
            variables.push_back(variable);
        }
        else if (rOldVariable.getApplyType() == R_VARIABLE_APPLY_ELEMENT)
        {
            if (elementLocations.empty())
            {
                elementLocations = findElementLocations(nodes,elements,locator);
            }
            variable.resize(variable.getNVectors(),uint(elementLocations.size()),true);
#pragma omp parallel for default(shared)
            for (int64_t j=0;j<int64_t(elementLocations.size());j++)
            {
                const ResultsLocation &rLocation = elementLocations[size_t(j)];
                if (rLocation.elementID == RConstants::eod)
                {
                    continue;
                }
                for (uint k=0;k<variable.getNVectors();k++)
                {
                    variable.setValue(k,uint(j),rOldVariable.getValue(k,rLocation.elementID));
                }
            }
            variables.push_back(variable);
        }
        RLogger::unindent();
    }
    RProgressFinalize();
    RLogger::unindent();

    return variables;
}
