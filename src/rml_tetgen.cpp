#include <cmath>
#include <algorithm>
#include <array>
#include <limits>
#include <vector>

#include <rbl_error.h>
#include <rbl_logger.h>
#include <rbl_progress.h>

#include "rml_tetgen.h"

RTetGen::RTetGen(const RModel &model)
{
    this->importModel(model,false);
}

void RTetGen::importModel(const RModel &model, bool reconstruct, const RRVector &nodeMeshSizeValues)
{
    this->firstnumber = 0;

    // Set nodes

    this->numberofpoints = int(model.getNNodes());
    this->numberofpointattributes = 0;
    this->numberofpointmtrs = nodeMeshSizeValues.size() == model.getNNodes() ? 1 : 0;

    if (this->numberofpoints)
    {
        this->pointlist = new REAL [uint(this->numberofpoints)*3];
//        if (this->numberofpointattributes)
//        {
//            this->pointattributelist = new REAL [this->numberofpoints*this->numberofpointattributes];
//        }
        if (this->numberofpointmtrs)
        {
            this->pointmtrlist = new REAL [uint(this->numberofpoints)*uint(this->numberofpointmtrs)];
        }
        this->pointmarkerlist = new int [uint(this->numberofpoints)];

        for (uint i=0;i<uint(this->numberofpoints);i++)
        {
            this->pointlist[3*i+0] = REAL(model.getNode(i).getX());
            this->pointlist[3*i+1] = REAL(model.getNode(i).getY());
            this->pointlist[3*i+2] = REAL(model.getNode(i).getZ());
//            if (this->numberofpointattributes)
//            {
//                this->pointattributelist[this->numberofpointattributes*i+0] = REAL(i);
//                for (int j=1;j<this->numberofpointattributes;j++)
//                {
//                    this->pointattributelist[this->numberofpointattributes*i+j] = 0.0;
//                }
//            }
            if (this->numberofpointmtrs)
            {
                this->pointmtrlist[uint(this->numberofpointmtrs)*i+0] = nodeMeshSizeValues[i];
                for (uint j=1;j<uint(this->numberofpointmtrs);j++)
                {
                    this->pointmtrlist[uint(this->numberofpointmtrs)*i+j] = 0.0;
                }
            }
            this->pointmarkerlist[i] = 0;
        }
    }

    for (uint i=0;i<model.getNPoints();i++)
    {
        int marker = int(i)+1;
        const RPoint &rPoint = model.getPoint(i);
        for (uint j=0;j<rPoint.size();j++)
        {
            this->pointmarkerlist[model.getElement(rPoint.get(j)).getNodeId(0)] = -marker;
        }
    }

    this->numberofedges = int(model.getNEntityElements(R_ENTITY_GROUP_LINE));
    if (this->numberofedges > 0)
    {
        this->edgelist = new int [uint(this->numberofedges)*2];
        this->edgemarkerlist = new int [uint(this->numberofedges)];
        uint edgeNumber = 0;
        for (uint i=0;i<model.getNLines();i++)
        {
            int marker = int(i)+1;
            const RLine &rLine = model.getLine(i);
            for (uint j=0;j<rLine.size();j++)
            {
                uint elementID = rLine.get(j);
                const RElement &rElement = model.getElement(elementID);
                this->edgelist[2*edgeNumber+0] = int(rElement.getNodeId(0)) + this->firstnumber;
                this->edgelist[2*edgeNumber+1] = int(rElement.getNodeId(1)) + this->firstnumber;
                this->edgemarkerlist[edgeNumber] = -marker;
                edgeNumber++;
            }
        }
    }

    if (reconstruct)
    {
        // Set trifacets

        this->numberoftrifaces = int(model.getNElements(R_ELEMENT_TRI1) + model.getNElements(R_ELEMENT_TRI2));

        if (this->numberoftrifaces)
        {
            this->trifacelist = new int[uint(this->numberoftrifaces)*3];
            this->trifacemarkerlist = new int[uint(this->numberoftrifaces)];

            RUVector surfaceBook(model.getNElements());
            std::fill(surfaceBook.begin(),surfaceBook.end(),RConstants::eod);

            uint ce = 0;
            for (uint i=0;i<model.getNElements();i++)
            {
                if (model.getElement(i).getType() != R_ELEMENT_TRI1 &&
                    model.getElement(i).getType() != R_ELEMENT_TRI2)
                {
                    continue;
                }
                this->trifacelist[3*ce+0] = int(model.getElement(i).getNodeId(0)) + this->firstnumber;
                this->trifacelist[3*ce+1] = int(model.getElement(i).getNodeId(1)) + this->firstnumber;
                this->trifacelist[3*ce+2] = int(model.getElement(i).getNodeId(2)) + this->firstnumber;

                this->trifacemarkerlist[ce] = -1;

                surfaceBook[i] = ce;

                ce++;
            }

            for (uint i=0;i<model.getNSurfaces();i++)
            {
                int marker = int(i+1);
                for (uint j=0;j<model.getSurface(i).size();j++)
                {
                    uint elementPosition = surfaceBook[model.getSurface(i).get(j)];
                    if (elementPosition != RConstants::eod)
                    {
                        this->trifacemarkerlist[elementPosition] = -marker;
                    }
                }
            }
        }

        // Set tetrahedras

        this->numberoftetrahedra = int(model.getNElements(R_ELEMENT_TETRA1));

        if (this->numberoftetrahedra)
        {
            this->numberofcorners = 4;
            this->numberoftetrahedronattributes = 1;
            this->tetrahedronlist = new int[uint(this->numberoftetrahedra)*4];
            this->tetrahedronattributelist = new REAL[uint(this->numberoftetrahedra)*uint(this->numberoftetrahedronattributes)];
            this->tetrahedronvolumelist = new REAL[uint(this->numberoftetrahedra)];

            RUVector volumeBook(model.getNElements());
            std::fill(volumeBook.begin(),volumeBook.end(),RConstants::eod);

            uint ce = 0;
            for (uint i=0;i<model.getNElements();i++)
            {
                if (model.getElement(i).getType() != R_ELEMENT_TETRA1)
                {
                    continue;
                }
                this->tetrahedronlist[4*ce+0] = int(model.getElement(i).getNodeId(0)) + this->firstnumber;
                this->tetrahedronlist[4*ce+1] = int(model.getElement(i).getNodeId(1)) + this->firstnumber;
                this->tetrahedronlist[4*ce+2] = int(model.getElement(i).getNodeId(2)) + this->firstnumber;
                this->tetrahedronlist[4*ce+3] = int(model.getElement(i).getNodeId(3)) + this->firstnumber;

                this->tetrahedronattributelist[uint(this->numberoftetrahedronattributes)*ce+0] = 0.0;

                this->tetrahedronvolumelist[ce] = -1.0;

                volumeBook[i] = ce;

                ce++;
            }

            // Set volume marker for each tetrahedra
            for (uint i=0;i<model.getNVolumes();i++)
            {
                int marker = int(i+1);
                for (uint j=0;j<model.getVolume(i).size();j++)
                {
                    uint elementPosition = volumeBook[model.getVolume(i).get(j)];
                    if (elementPosition != RConstants::eod)
                    {
                        this->tetrahedronattributelist[uint(this->numberoftetrahedronattributes)*elementPosition+0] = -marker;
                    }
                }
            }
        }
    }
    else
    {
        // Set facets
        this->numberoffacets = int(  model.getNElements(R_ELEMENT_TRI1)
                                   + model.getNElements(R_ELEMENT_TRI2)
                                   + model.getNElements(R_ELEMENT_QUAD1)
                                   + model.getNElements(R_ELEMENT_QUAD2));

        if (this->numberoffacets > 0)
        {
            this->facetlist = new tetgenio::facet[uint(this->numberoffacets)];
            this->facetmarkerlist = new int[uint(this->numberoffacets)];

            RUVector faceBook(model.getNElements());
            std::fill(faceBook.begin(),faceBook.end(),RConstants::eod);

            uint ce = 0;
            for (uint i=0;i<model.getNElements();i++)
            {
                uint nne = 0;
                if (model.getElement(i).getType() == R_ELEMENT_TRI1 ||
                    model.getElement(i).getType() == R_ELEMENT_TRI2)
                {
                    nne = 3;
                }
                else if (model.getElement(i).getType() == R_ELEMENT_QUAD1 ||
                         model.getElement(i).getType() == R_ELEMENT_QUAD2)
                {
                    nne = 4;
                }
                else
                {
                    continue;
                }
                this->facetlist[ce].numberofholes = 0;
                this->facetlist[ce].holelist = nullptr;
                this->facetlist[ce].numberofpolygons = 1;
                this->facetlist[ce].polygonlist = new tetgenio::polygon[1];
                this->facetlist[ce].polygonlist[0].numberofvertices = int(nne);
                this->facetlist[ce].polygonlist[0].vertexlist = new int[nne];
                for (uint j=0;j<nne;j++)
                {
                    this->facetlist[ce].polygonlist[0].vertexlist[j] = int(model.getElement(i).getNodeId(j)) + this->firstnumber;
                }

                this->facetmarkerlist[ce] = 0;

                faceBook[i] = ce;

                ce++;
            }

            for (uint i=0;i<model.getNSurfaces();i++)
            {
                int marker = int(i+1);
                for (uint j=0;j<model.getSurface(i).size();j++)
                {
                    uint elementPosition = faceBook[model.getSurface(i).get(j)];
                    if (elementPosition != RConstants::eod)
                    {
                        this->facetmarkerlist[elementPosition] = -marker;
                    }
                }
            }
        }
    }
}

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

static std::vector<ResultsLocation> findNodeLocations(const RTetGen &tetgen, const ResultsLocator &locator)
{
    std::vector<ResultsLocation> locations(static_cast<size_t>(tetgen.numberofpoints));

#pragma omp parallel for default(shared)
    for (int64_t i=0;i<tetgen.numberofpoints;i++)
    {
        locations[size_t(i)] = locator.find(RNode(tetgen.pointlist[3*i+0],tetgen.pointlist[3*i+1],tetgen.pointlist[3*i+2]),
                                            R_ENTITY_GROUP_ELEMENT);
    }
    return locations;
}

static std::vector<ResultsLocation> findElementLocations(const RTetGen &tetgen,
                                                         const ResultsLocator &locator,
                                                         uint numberOfPointElements,
                                                         uint numberOfLineElements)
{
    // Elements are ordered in the same way as when they are added to the model.
    struct NewElement
    {
        std::array<int,4> nodeIDs;
        uint nNodes;
        REntityGroupTypeMask group;
    };
    std::vector<NewElement> newElements;
    newElements.reserve(numberOfPointElements + numberOfLineElements + uint(tetgen.numberoftrifaces) + uint(tetgen.numberoftetrahedra));

    if (tetgen.pointmarkerlist)
    {
        for (int i=0;i<tetgen.numberofpoints;i++)
        {
            if (tetgen.pointmarkerlist[i] > 0)
            {
                newElements.push_back({{i,0,0,0},1,R_ENTITY_GROUP_POINT});
            }
        }
    }
    if (tetgen.edgemarkerlist)
    {
        for (int i=0;i<tetgen.numberofedges;i++)
        {
            if (tetgen.edgemarkerlist[i] > 0)
            {
                newElements.push_back({{tetgen.edgelist[2*i+0] - tetgen.firstnumber,
                                        tetgen.edgelist[2*i+1] - tetgen.firstnumber,0,0},2,R_ENTITY_GROUP_LINE});
            }
        }
    }
    for (int i=0;i<tetgen.numberoftrifaces;i++)
    {
        newElements.push_back({{tetgen.trifacelist[3*i+0] - tetgen.firstnumber,
                                tetgen.trifacelist[3*i+1] - tetgen.firstnumber,
                                tetgen.trifacelist[3*i+2] - tetgen.firstnumber,0},3,R_ENTITY_GROUP_SURFACE});
    }
    for (int i=0;i<tetgen.numberoftetrahedra;i++)
    {
        newElements.push_back({{tetgen.tetrahedronlist[4*i+0] - tetgen.firstnumber,
                                tetgen.tetrahedronlist[4*i+1] - tetgen.firstnumber,
                                tetgen.tetrahedronlist[4*i+2] - tetgen.firstnumber,
                                tetgen.tetrahedronlist[4*i+3] - tetgen.firstnumber},4,R_ENTITY_GROUP_VOLUME});
    }

    std::vector<ResultsLocation> locations(newElements.size());

#pragma omp parallel for default(shared)
    for (int64_t i=0;i<int64_t(newElements.size());i++)
    {
        const NewElement &rNewElement = newElements[size_t(i)];
        double x = 0.0, y = 0.0, z = 0.0;
        for (uint j=0;j<rNewElement.nNodes;j++)
        {
            const int nodeID = rNewElement.nodeIDs[j];
            x += tetgen.pointlist[3*nodeID+0];
            y += tetgen.pointlist[3*nodeID+1];
            z += tetgen.pointlist[3*nodeID+2];
        }
        // Element takes value from the same kind of element, as they can have different results.
        locations[size_t(i)] = locator.find(RNode(x/rNewElement.nNodes,y/rNewElement.nNodes,z/rNewElement.nNodes),
                                            rNewElement.group);
    }
    return locations;
}

void RTetGen::exportMesh(RModel &model, bool keepResults) const
{
    // Find number of point elements.
    uint numberOfPointElements = 0;
    if (this->pointmarkerlist)
    {
        for (int i=0;i<this->numberofpoints;i++)
        {
            if (this->pointmarkerlist[i] > 0)
            {
                numberOfPointElements++;
            }
        }
    }

    // Find number of line elements.
    uint numberOfLineElements = 0;
    if (this->edgemarkerlist)
    {
        for (int i=0;i<this->numberofedges;i++)
        {
            if (this->edgemarkerlist[i] > 0)
            {
                numberOfLineElements++;
            }
        }
    }

    // Interpolate results.
    std::vector<RVariable> variables;

    if (keepResults && model.getNVariables() > 0)
    {
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
                    nodeLocations = findNodeLocations(*this,locator);
                }
                variable.resize(variable.getNVectors(),uint(this->numberofpoints),true);
#pragma omp parallel for default(shared)
                for (int64_t j=0;j<this->numberofpoints;j++)
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
                    elementLocations = findElementLocations(*this,locator,numberOfPointElements,numberOfLineElements);
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
    }

    // NODES
    model.setNNodes(uint(this->numberofpoints));

    for (uint i=0;i<uint(this->numberofpoints);i++)
    {
        model.getNode(i).set(this->pointlist[3*i+0],
                             this->pointlist[3*i+1],
                             this->pointlist[3*i+2]);
    }

    model.setNVariables(uint(variables.size()));

    model.setNElements(numberOfPointElements + numberOfLineElements + uint(this->numberoftrifaces) + uint(this->numberoftetrahedra));

    std::vector<RUVector> volumeNeigs;

    model.clearSurfaceNeighbors();
    model.clearVolumeNeighbors();

    volumeNeigs.resize(numberOfPointElements + numberOfLineElements + uint(this->numberoftrifaces) + uint(this->numberoftetrahedra));

    uint nElements = 0;

    // POINTS
    for (uint i=0;i<model.getNPoints();i++)
    {
        model.getPoint(i).resize(0);
    }
    if (this->pointmarkerlist)
    {
        for (uint i=0;i<uint(this->numberofpoints);i++)
        {
            if (this->pointmarkerlist[i] > 0)
            {
                RElement element(R_ELEMENT_POINT);

                element.setNodeId(0,i);

                model.setElement(nElements,element,false);

                uint marker = uint(std::abs(this->pointmarkerlist[i])) - 1;
                if (model.getNPoints() < marker + 1)
                {
                    model.setNPoints(marker+1);
                }
                model.getPoint(marker).add(nElements);
                nElements++;
            }
        }
    }

    //LINES
    for (uint i=0;i<model.getNLines();i++)
    {
        model.getLine(i).resize(0);
    }
    if (this->edgemarkerlist)
    {
        for (uint i=0;i<uint(this->numberofedges);i++)
        {
            if (this->edgemarkerlist[i] > 0)
            {
                RElement element(R_ELEMENT_TRUSS1);

                // Line element normals are swapped
                element.setNodeId(0,uint(this->edgelist[2*i+0] - this->firstnumber));
                element.setNodeId(1,uint(this->edgelist[2*i+1] - this->firstnumber));

                model.setElement(nElements,element,false);

                uint marker = uint(std::abs(this->edgemarkerlist[i])) - 1;
                if (model.getNLines() < marker + 1)
                {
                    model.setNLines(marker+1);
                }
                model.getLine(marker).add(nElements);
                nElements++;
            }
        }
    }

    // SURFACES
    for (uint i=0;i<model.getNSurfaces();i++)
    {
        model.getSurface(i).resize(0);
    }
    for (uint i=0;i<uint(this->numberoftrifaces);i++)
    {
        RElement element(R_ELEMENT_TRI1);

        // Triangle element normals are swapped
        element.setNodeId(0,uint(this->trifacelist[3*i+0] - this->firstnumber));
        element.setNodeId(1,uint(this->trifacelist[3*i+2] - this->firstnumber));
        element.setNodeId(2,uint(this->trifacelist[3*i+1] - this->firstnumber));

        model.setElement(nElements,element,false);
        uint marker = this->trifacemarkerlist ? uint(std::abs(this->trifacemarkerlist[i])) - 1 : 0;
        if (model.getNSurfaces() < marker + 1)
        {
            model.setNSurfaces(marker+1);
        }
        model.getSurface(marker).add(nElements);
        nElements++;
    }

    // VOLUMES
    RUVector tetrahedraVolumeMarker(uint(this->numberoftetrahedra),0);
    if (this->numberoftetrahedronattributes > 0)
    {
        uint minTetrahedraVolumeMarker = 0;
        uint maxTetrahedraVolumeMarker = 0;
        for (uint i=0;i<uint(this->numberoftetrahedra);i++)
        {
            tetrahedraVolumeMarker[i] = uint(std::abs(this->tetrahedronattributelist[uint(this->numberoftetrahedronattributes)*i + 0]) - 1);
            minTetrahedraVolumeMarker = std::min(minTetrahedraVolumeMarker,tetrahedraVolumeMarker[i]);
            maxTetrahedraVolumeMarker = std::max(maxTetrahedraVolumeMarker,tetrahedraVolumeMarker[i]);
        }
        RUVector tetrahedraVolumeMarkerBook(maxTetrahedraVolumeMarker+1,RConstants::eod);
        for (uint i=0;i<uint(this->numberoftetrahedra);i++)
        {
            tetrahedraVolumeMarkerBook[tetrahedraVolumeMarker[i]] = 0;
        }
        uint tetrahedraVolumeMarkerBookCnt = 0;
        for (uint i=0;i<tetrahedraVolumeMarkerBook.size();i++)
        {
            if (tetrahedraVolumeMarkerBook[i] == 0)
            {
                tetrahedraVolumeMarkerBook[i] = tetrahedraVolumeMarkerBookCnt++;
            }
        }
        for (uint i=0;i<tetrahedraVolumeMarkerBook.size();i++)
        {
            if (tetrahedraVolumeMarkerBook[i] == RConstants::eod)
            {
                tetrahedraVolumeMarkerBook[i] = tetrahedraVolumeMarkerBookCnt++;
            }
        }
        for (uint i=0;i<uint(this->numberoftetrahedra);i++)
        {
            tetrahedraVolumeMarker[i] = tetrahedraVolumeMarkerBook[tetrahedraVolumeMarker[i]];
        }

        // Resize number of volumes
        model.setNVolumes(tetrahedraVolumeMarkerBookCnt);
    }
    else
    {
        model.setNVolumes(1);
    }

    // VOLUME NEIGHBORS
    for (uint i=0;i<uint(this->numberoftetrahedra);i++)
    {
        RElement element(R_ELEMENT_TETRA1);

        element.setNodeId(0,uint(this->tetrahedronlist[4*i+0] - this->firstnumber));
        element.setNodeId(1,uint(this->tetrahedronlist[4*i+1] - this->firstnumber));
        element.setNodeId(2,uint(this->tetrahedronlist[4*i+2] - this->firstnumber));
        element.setNodeId(3,uint(this->tetrahedronlist[4*i+3] - this->firstnumber));

        model.setElement(nElements,element,false);
        model.getVolume(tetrahedraVolumeMarker[i]).add(nElements);
        for (uint j=0;j<4;j++)
        {
            if (this->neighborlist[4*i+j] >= this->firstnumber)
            {
                volumeNeigs[nElements].push_back(uint(this->neighborlist[4*i+j]) - uint(this->firstnumber) + numberOfPointElements + numberOfLineElements + uint(this->numberoftrifaces));
            }
        }
        nElements++;
    }

    model.setVolumeNeighbors(volumeNeigs);

    for (uint i=model.getNPoints();i>0;i--)
    {
        if (model.getPoint(i-1).size() == 0)
        {
            model.removePoint(i-1);
        }
    }
    for (uint i=model.getNLines();i>0;i--)
    {
        if (model.getLine(i-1).size() == 0)
        {
            model.removeLine(i-1);
        }
    }
    for (uint i=model.getNSurfaces();i>0;i--)
    {
        if (model.getSurface(i-1).size() == 0)
        {
            model.removeSurface(i-1);
        }
    }
    for (uint i=model.getNVolumes();i>0;i--)
    {
        if (model.getVolume(i-1).size() == 0)
        {
            model.removeVolume(i-1);
        }
    }
    for (uint i=0;i<model.getNVolumes();i++)
    {
        if (model.getVolume(i).getName().isEmpty() || model.getVolume(i).getName() == RVolume::defaultName)
        {
            model.getVolume(i).setName(RVolume::defaultName + " " + QString::number(i+1));
        }
    }

    if (keepResults)
    {
        for (uint i=0;i<model.getNVariables();i++)
        {
            model.setVariable(i,variables[i]);
        }
    }

    if (model.checkMesh() != R_MODEL_PROBLEM_NONE)
    {
        throw RError(RError::Type::Application,R_ERROR_REF,"Invalid mesh was generated.");
    }

    // Synchronize normals.
    model.syncSurfaceNormals();
}
