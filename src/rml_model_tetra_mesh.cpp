#include <algorithm>
#include <vector>

#include <rbl_error.h>
#include <rbl_logger.h>
#include <rbl_progress.h>

#include "rml_mesh_results_interpolator.h"
#include "rml_model_tetra_mesh.h"

RModelTetraMesh::RModelTetraMesh()
{
}

void RModelTetraMesh::importModel(const RModel &model, bool reconstruct, const RRVector &nodeMeshSizeValues)
{
    this->clear();

    // Nodes
    // Nodes in the middle of edges of second order elements are left out: the mesh is
    // generated from corner nodes and new nodes are added to the middle of its edges.

    const bool useMeshSizes = nodeMeshSizeValues.size() == model.getNNodes();

    uint nCornerNodes = 0;
    const RUVector nodeBook = model.findCornerNodeBook(nCornerNodes);

    this->nodes.reserve(nCornerNodes);
    for (uint i=0;i<model.getNNodes();i++)
    {
        if (nodeBook[i] == RConstants::eod)
        {
            continue;
        }
        const RNode &rNode = model.getNode(i);
        this->addNode(rNode.getX(),rNode.getY(),rNode.getZ(),noMarker,useMeshSizes ? nodeMeshSizeValues[i] : 0.0);
    }

    auto findNode = [&](const RElement &rElement, uint position) -> int
    {
        return int(nodeBook[rElement.getNodeId(position)]);
    };

    // Points

    for (uint i=0;i<model.getNPoints();i++)
    {
        const RPoint &rPoint = model.getPoint(i);
        for (uint j=0;j<rPoint.size();j++)
        {
            this->nodes[size_t(findNode(model.getElement(rPoint.get(j)),0))].marker = int(i);
        }
    }

    // Lines

    for (uint i=0;i<model.getNLines();i++)
    {
        const RLine &rLine = model.getLine(i);
        for (uint j=0;j<rLine.size();j++)
        {
            const RElement &rElement = model.getElement(rLine.get(j));
            this->addEdge(findNode(rElement,0),findNode(rElement,1),int(i));
        }
    }

    // Surfaces

    std::vector<int> surfaceMarkers(model.getNElements(),noMarker);
    for (uint i=0;i<model.getNSurfaces();i++)
    {
        const RSurface &rSurface = model.getSurface(i);
        for (uint j=0;j<rSurface.size();j++)
        {
            surfaceMarkers[rSurface.get(j)] = int(i);
        }
    }
    for (uint i=0;i<model.getNElements();i++)
    {
        const RElement &rElement = model.getElement(i);
        const RElementType elementType = rElement.getType();
        if (elementType == R_ELEMENT_TRI1 || elementType == R_ELEMENT_TRI2)
        {
            this->addTriangle(findNode(rElement,0),findNode(rElement,1),findNode(rElement,2),surfaceMarkers[i]);
        }
        else if (elementType == R_ELEMENT_QUAD1 || elementType == R_ELEMENT_QUAD2)
        {
            this->addTriangle(findNode(rElement,0),findNode(rElement,1),findNode(rElement,2),surfaceMarkers[i]);
            this->addTriangle(findNode(rElement,0),findNode(rElement,2),findNode(rElement,3),surfaceMarkers[i]);
        }
    }

    // Volumes

    if (reconstruct)
    {
        for (uint i=0;i<model.getNVolumes();i++)
        {
            const RVolume &rVolume = model.getVolume(i);
            for (uint j=0;j<rVolume.size();j++)
            {
                const RElement &rElement = model.getElement(rVolume.get(j));
                if (RElement::getLinearType(rElement.getType()) != R_ELEMENT_TETRA1)
                {
                    continue;
                }
                this->addTetrahedron(findNode(rElement,0),
                                     findNode(rElement,1),
                                     findNode(rElement,2),
                                     findNode(rElement,3),
                                     int(i));
            }
        }
    }
}

void RModelTetraMesh::exportMesh(RModel &model, bool keepResults) const
{
    // Elements are ordered by their kind: points, lines, surfaces and volumes.
    // Triangles without a marker are interfaces between volumes which do not belong to any surface.

    uint numberOfPointElements = 0;
    for (const Node &rNode : this->nodes)
    {
        if (rNode.marker != noMarker)
        {
            numberOfPointElements++;
        }
    }
    uint numberOfLineElements = 0;
    for (const Edge &rEdge : this->edges)
    {
        if (rEdge.marker != noMarker)
        {
            numberOfLineElements++;
        }
    }
    uint numberOfSurfaceElements = 0;
    for (const Triangle &rTriangle : this->triangles)
    {
        if (rTriangle.marker != noMarker)
        {
            numberOfSurfaceElements++;
        }
    }
    const uint numberOfVolumeElements = uint(this->tetrahedra.size());
    const uint numberOfElements = numberOfPointElements + numberOfLineElements + numberOfSurfaceElements + numberOfVolumeElements;
    const uint firstVolumeElement = numberOfPointElements + numberOfLineElements + numberOfSurfaceElements;
    // Mid-edge nodes of the mesh are ordered the same way as those of second order elements.
    const bool secondOrder = this->getElementOrder() == 2;

    // Interpolate results.
    std::vector<RVariable> variables;

    if (keepResults && model.getNVariables() > 0)
    {
        std::vector<RNode> newNodes;
        newNodes.reserve(this->nodes.size());
        for (const Node &rNode : this->nodes)
        {
            newNodes.push_back(RNode(rNode.x,rNode.y,rNode.z));
        }

        std::vector<RMeshResultsInterpolator::Element> newElements;
        newElements.reserve(numberOfElements);
        for (size_t i=0;i<this->nodes.size();i++)
        {
            if (this->nodes[i].marker != noMarker)
            {
                newElements.push_back({{uint(i),0,0,0},1,R_ENTITY_GROUP_POINT});
            }
        }
        for (const Edge &rEdge : this->edges)
        {
            if (rEdge.marker != noMarker)
            {
                newElements.push_back({{uint(rEdge.nodeIDs[0]),uint(rEdge.nodeIDs[1]),0,0},2,R_ENTITY_GROUP_LINE});
            }
        }
        for (const Triangle &rTriangle : this->triangles)
        {
            if (rTriangle.marker != noMarker)
            {
                newElements.push_back({{uint(rTriangle.nodeIDs[0]),uint(rTriangle.nodeIDs[1]),uint(rTriangle.nodeIDs[2]),0},3,R_ENTITY_GROUP_SURFACE});
            }
        }
        for (const Tetrahedron &rTetrahedron : this->tetrahedra)
        {
            newElements.push_back({{uint(rTetrahedron.nodeIDs[0]),
                                    uint(rTetrahedron.nodeIDs[1]),
                                    uint(rTetrahedron.nodeIDs[2]),
                                    uint(rTetrahedron.nodeIDs[3])},4,R_ENTITY_GROUP_VOLUME});
        }

        variables = RMeshResultsInterpolator::interpolate(model,newNodes,newElements);
        RLogger::info("Converting generated mesh to Range model.\n");
        RProgressInitialize("Converting generated mesh to Range model.",true);
    }

    // NODES
    model.setNNodes(uint(this->nodes.size()));

    for (uint i=0;i<uint(this->nodes.size());i++)
    {
        model.getNode(i).set(this->nodes[i].x,this->nodes[i].y,this->nodes[i].z);
    }

    model.setNVariables(uint(variables.size()));

    model.setNElements(numberOfElements);

    std::vector<RUVector> volumeNeigs;

    model.clearSurfaceNeighbors();
    model.clearVolumeNeighbors();

    volumeNeigs.resize(numberOfElements);

    uint nElements = 0;

    // POINTS
    for (uint i=0;i<model.getNPoints();i++)
    {
        model.getPoint(i).resize(0);
    }
    for (uint i=0;i<uint(this->nodes.size());i++)
    {
        if (this->nodes[i].marker == noMarker)
        {
            continue;
        }
        RElement element(R_ELEMENT_POINT);

        element.setNodeId(0,i);

        model.setElement(nElements,element,false);

        const uint marker = uint(this->nodes[i].marker);
        if (model.getNPoints() < marker + 1)
        {
            model.setNPoints(marker+1);
        }
        model.getPoint(marker).add(nElements);
        nElements++;
    }

    // LINES
    for (uint i=0;i<model.getNLines();i++)
    {
        model.getLine(i).resize(0);
    }
    for (const Edge &rEdge : this->edges)
    {
        if (rEdge.marker == noMarker)
        {
            continue;
        }
        RElement element(secondOrder ? R_ELEMENT_TRUSS2 : R_ELEMENT_TRUSS1);

        element.setNodeId(0,uint(rEdge.nodeIDs[0]));
        element.setNodeId(1,uint(rEdge.nodeIDs[1]));
        if (secondOrder)
        {
            element.setNodeId(2,uint(rEdge.midNodeID));
        }

        model.setElement(nElements,element,false);

        const uint marker = uint(rEdge.marker);
        if (model.getNLines() < marker + 1)
        {
            model.setNLines(marker+1);
        }
        model.getLine(marker).add(nElements);
        nElements++;
    }

    // SURFACES
    for (uint i=0;i<model.getNSurfaces();i++)
    {
        model.getSurface(i).resize(0);
    }
    for (const Triangle &rTriangle : this->triangles)
    {
        if (rTriangle.marker == noMarker)
        {
            continue;
        }
        RElement element(secondOrder ? R_ELEMENT_TRI2 : R_ELEMENT_TRI1);

        element.setNodeId(0,uint(rTriangle.nodeIDs[0]));
        element.setNodeId(1,uint(rTriangle.nodeIDs[1]));
        element.setNodeId(2,uint(rTriangle.nodeIDs[2]));
        if (secondOrder)
        {
            for (uint j=0;j<3;j++)
            {
                element.setNodeId(3+j,uint(rTriangle.midNodeIDs[j]));
            }
        }

        model.setElement(nElements,element,false);

        const uint marker = uint(rTriangle.marker);
        if (model.getNSurfaces() < marker + 1)
        {
            model.setNSurfaces(marker+1);
        }
        model.getSurface(marker).add(nElements);
        nElements++;
    }

    // VOLUMES
    uint nVolumes = 0;
    for (const Tetrahedron &rTetrahedron : this->tetrahedra)
    {
        if (rTetrahedron.marker == noMarker)
        {
            throw RError(RError::Type::Application,R_ERROR_REF,"Generated element does not belong to any volume.");
        }
        nVolumes = std::max(nVolumes,uint(rTetrahedron.marker)+1);
    }
    // Volumes which are in the model already are kept, those without elements are removed below.
    if (model.getNVolumes() < nVolumes)
    {
        model.setNVolumes(nVolumes);
    }
    for (uint i=0;i<model.getNVolumes();i++)
    {
        model.getVolume(i).resize(0);
    }
    for (const Tetrahedron &rTetrahedron : this->tetrahedra)
    {
        RElement element(secondOrder ? R_ELEMENT_TETRA2 : R_ELEMENT_TETRA1);

        element.setNodeId(0,uint(rTetrahedron.nodeIDs[0]));
        element.setNodeId(1,uint(rTetrahedron.nodeIDs[1]));
        element.setNodeId(2,uint(rTetrahedron.nodeIDs[2]));
        element.setNodeId(3,uint(rTetrahedron.nodeIDs[3]));
        if (secondOrder)
        {
            for (uint j=0;j<6;j++)
            {
                element.setNodeId(4+j,uint(rTetrahedron.midNodeIDs[j]));
            }
        }

        model.setElement(nElements,element,false);
        model.getVolume(uint(rTetrahedron.marker)).add(nElements);
        for (uint j=0;j<4;j++)
        {
            if (rTetrahedron.neighborIDs[j] != noTetrahedron)
            {
                volumeNeigs[nElements].push_back(uint(rTetrahedron.neighborIDs[j]) + firstVolumeElement);
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
