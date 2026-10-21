#include <rbl_error.h>
#include <rbl_uvector.h>
#include <rbl_utils.h>

#include "rml_polygon.h"
#include "rml_triangle.h"
#include "rml_element.h"

void RPolygon::_init(const RPolygon *pPolygon)
{
    if (pPolygon)
    {
        this->nodes = pPolygon->nodes;
        this->normal = pPolygon->normal;
    }
}

RPolygon::RPolygon(const std::vector<RNode> &nodes)
    : nodes(nodes)
{
    this->_init();
    this->computeNormal();
}

RPolygon::RPolygon(const std::vector<RR3Vector> &nodes)
{
    this->nodes.resize(nodes.size());
    for (uint i=0;i<nodes.size();i++)
    {
        this->nodes[i].setX(nodes[i][0]);
        this->nodes[i].setY(nodes[i][1]);
        this->nodes[i].setZ(nodes[i][2]);
    }

    this->_init();
    this->computeNormal();
}

RPolygon::RPolygon(const RPolygon &polygon)
{
    this->_init(&polygon);
}

RPolygon::~RPolygon()
{

}

RPolygon &RPolygon::operator =(const RPolygon &polygon)
{
    this->_init(&polygon);
    return (*this);
}

void RPolygon::setNodes(const std::vector<RNode> &nodes)
{
    this->nodes = nodes;
    RPolygon::sortNodes(this->nodes);
    this->computeNormal();
}

void RPolygon::removeNode(uint nodePosition, bool updateNormal)
{
    R_ERROR_ASSERT (nodePosition < this->nodes.size());
    this->nodes.erase(this->nodes.begin()+nodePosition);
    if (updateNormal)
    {
        this->computeNormal();
    }
}

double RPolygon::findArea(void) const
{
    double area = 0.0;

    return area;
}

uint RPolygon::getNNodes(void) const
{
    return uint(this->nodes.size());
}

struct NodeAngleItem
{
    uint id;
    RNode node;
    double angle;
};

bool nodeAngleItemCompare(const NodeAngleItem &a, const NodeAngleItem &b)
{
    return a.angle < b.angle;
}

std::vector<uint> RPolygon::sortNodes(std::vector<RNode> &nodes)
{
    std::vector<uint> nodeIDs;
    nodeIDs.resize(nodes.size());
    for (uint i=0;i<nodeIDs.size();i++)
    {
        nodeIDs[i] = i;
    }

    if (nodes.size() < 4)
    {
        return nodeIDs;
    }

    std::vector<NodeAngleItem> nodeAngleList;

    RR3Vector v1(nodes[1].getX()-nodes[0].getX(),nodes[1].getY()-nodes[0].getY(),nodes[1].getZ()-nodes[0].getZ());

    RTriangle t(nodes[0],nodes[1],nodes[2]);
    RR3Vector firstNormal(t.getNormal());

    nodeAngleList.push_back(NodeAngleItem());
    nodeAngleList.back().id = nodeIDs[1];
    nodeAngleList.back().node = nodes[1];
    nodeAngleList.back().angle = 0.0;
    for (uint i=2;i<nodes.size();i++)
    {
        nodeAngleList.push_back(NodeAngleItem());
        nodeAngleList.back().id = nodeIDs[i];
        nodeAngleList.back().node = nodes[i];
        RR3Vector v2(nodes[i].getX()-nodes[0].getX(),nodes[i].getY()-nodes[0].getY(),nodes[i].getZ()-nodes[0].getZ());
        nodeAngleList.back().angle = RR3Vector::angle(v1,v2);

        if (i > 2)
        {
            RTriangle t(nodes[0],nodes[1],nodes[i]);
            if (RR3Vector::angle(firstNormal,t.getNormal()) > RConstants::pi/2.0)
            {
                nodeAngleList.back().angle *= -1.0;
            }
        }
    }

    std::sort(nodeAngleList.begin(), nodeAngleList.end(), nodeAngleItemCompare);

    for (uint i=0;i<nodeAngleList.size();i++)
    {
        nodeIDs[i+1] = nodeAngleList[i].id;
        nodes[i+1] = nodeAngleList[i].node;
    }

    return nodeIDs;
}

std::vector<RElement> RPolygon::triangulate(const std::vector<RNode> &nodes, bool nodesSorted)
{
    if (nodes.size() < 3)
    {
        return std::vector<RElement>();
    }
    else if (nodes.size() == 3)
    {
        RElement element;
        element.setType(R_ELEMENT_TRI1);
        element.setNodeId(0,0);
        element.setNodeId(1,1);
        element.setNodeId(2,2);
        std::vector<RElement> elements;
        elements.push_back(element);
        return elements;
    }

    std::vector<RNode> sortedNodes(nodes);
    std::vector<uint> nodeIDs;

    nodeIDs.resize(sortedNodes.size());
    for (uint i=0;i<nodeIDs.size();i++)
    {
        nodeIDs[i] = i;
    }

    if (!nodesSorted)
    {
        nodeIDs = RPolygon::sortNodes(sortedNodes);
    }

    RPolygon p(sortedNodes);
    RUVector book(p.getNNodes());

    for (uint i=0;i<book.size();i++)
    {
        book[i] = i;
    }

    std::vector<RElement> elements;

    while (true)
    {
        uint n2 = p.findEarNode();
        if (n2 == RConstants::eod)
        {
            break;
        }
        uint n1 = (n2 > 0) ? n2 - 1 : p.getNNodes() - 1;
        uint n3 = (n2 < p.getNNodes() - 1) ? n2 + 1 : 0;

        RElement element(R_ELEMENT_TRI1);
        element.setNodeId(0,nodeIDs[book[n1]]);
        element.setNodeId(1,nodeIDs[book[n2]]);
        element.setNodeId(2,nodeIDs[book[n3]]);

        elements.push_back(element);

        p.removeNode(n2,false);
        book.erase(book.begin()+n2);
    }

    return elements;
}

void RPolygon::computeNormal(void)
{
    this->normal[0] = this->normal[1] = this->normal[2] = 0.0;

    if (this->nodes.size() < 3)
    {
        return;
    }
    if (this->nodes.size() == 3)
    {
        RTriangle t(this->nodes[0],this->nodes[1],this->nodes[2]);
        this->normal = t.getNormal();
        return;
    }

    // Newell's method - unlike summing normals at the corners it is not
    // distorted by reflex corners and copes with nodes which are not coplanar.
    for (uint i=0;i<this->getNNodes();i++)
    {
        const RNode &a = this->nodes[i];
        const RNode &b = this->nodes[(i+1) % this->getNNodes()];

        this->normal[0] += (a.getY() - b.getY()) * (a.getZ() + b.getZ());
        this->normal[1] += (a.getZ() - b.getZ()) * (a.getX() + b.getX());
        this->normal[2] += (a.getX() - b.getX()) * (a.getY() + b.getY());
    }

    this->normal.normalize();
}

double RPolygon::findOrientation(const RNode &a, const RNode &b, const RNode &c) const
{
    RR3Vector ab(b.getX()-a.getX(),b.getY()-a.getY(),b.getZ()-a.getZ());
    RR3Vector ac(c.getX()-a.getX(),c.getY()-a.getY(),c.getZ()-a.getZ());

    RR3Vector n;
    RR3Vector::cross(ab,ac,n);

    return RRVector::dot(n,this->normal);
}

uint RPolygon::findEarNode(void) const
{
    if (this->getNNodes() < 3)
    {
        return RConstants::eod;
    }
    if (this->getNNodes() == 3)
    {
        return 0;
    }

    // Nodes of a hole in a curved surface are not coplanar, therefore all
    // tests are done on the projection to the plane perpendicular to the
    // polygon normal.

    uint earNode = RConstants::eod;
    double earQuality = -1.0;

    uint convexNode = RConstants::eod;
    double convexQuality = -1.0;

    for (uint i=0;i<this->getNNodes();i++)
    {
        uint n1 = (i==0) ? this->getNNodes()-1 : i-1;
        uint n2 = i;
        uint n3 = (i==this->getNNodes()-1) ? 0 : i+1;

        const RNode &p1 = this->nodes[n1];
        const RNode &p2 = this->nodes[n2];
        const RNode &p3 = this->nodes[n3];

        double orientation = this->findOrientation(p1,p2,p3);
        if (orientation <= 0.0)
        {
            // Reflex or degenerated corner.
            continue;
        }

        // Quality of the triangle is its smallest angle.
        RR3Vector v12(p2.getX()-p1.getX(),p2.getY()-p1.getY(),p2.getZ()-p1.getZ());
        RR3Vector v23(p3.getX()-p2.getX(),p3.getY()-p2.getY(),p3.getZ()-p2.getZ());
        RR3Vector v31(p1.getX()-p3.getX(),p1.getY()-p3.getY(),p1.getZ()-p3.getZ());
        double a1 = RConstants::pi - RR3Vector::angle(v31,v12);
        double a2 = RConstants::pi - RR3Vector::angle(v12,v23);
        double a3 = RConstants::pi - RR3Vector::angle(v23,v31);
        double quality = std::min(a1,std::min(a2,a3));

        if (quality > convexQuality)
        {
            convexQuality = quality;
            convexNode = n2;
        }

        // Triangle is an ear only if no other node lies inside it or on its edges.
        double tolerance = 1.0e-10 * orientation;
        bool isEar = true;
        for (uint j=0;j<this->getNNodes() && isEar;j++)
        {
            if (j == n1 || j == n2 || j == n3)
            {
                continue;
            }
            const RNode &p = this->nodes[j];
            if (this->findOrientation(p1,p2,p) >= -tolerance &&
                this->findOrientation(p2,p3,p) >= -tolerance &&
                this->findOrientation(p3,p1,p) >= -tolerance)
            {
                isEar = false;
            }
        }

        if (isEar && quality > earQuality)
        {
            earQuality = quality;
            earNode = n2;
        }
    }

    if (earNode == RConstants::eod)
    {
        // Projection of the polygon is not simple (strongly warped hole).
        // Rather than leaving the hole open clip the best convex corner.
        earNode = (convexNode == RConstants::eod) ? 0 : convexNode;
    }

    return earNode;
}
