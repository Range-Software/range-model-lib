#include <cmath>
#include <vector>

#include "rml_shape_generator.h"
#include "rml_segment.h"

void RShapeGenerator::_init(const RShapeGenerator *pShapeGenerator)
{
    if (pShapeGenerator)
    {

    }
}

RShapeGenerator::RShapeGenerator()
{
    this->_init();
}

RShapeGenerator::RShapeGenerator(const RShapeGenerator &shapeGenerator)
{
    this->_init(&shapeGenerator);
}

RShapeGenerator::~RShapeGenerator()
{
}

RShapeGenerator &RShapeGenerator::operator =(const RShapeGenerator &shapeGenerator)
{
    this->_init(&shapeGenerator);
    return (*this);
}

RModelRaw RShapeGenerator::generateArrow(const RR3Vector &v1, const RR3Vector &v2, double scale)
{
    static constexpr uint nSides = 16;
    static constexpr double xRim    = 1.0/2.0;
    static constexpr double radius  = 1.0/6.0;
    static constexpr double twoPi   = 2.0 * M_PI;

    RModelRaw arrow;

    RNode tail(0.0,        0.0, 0.0);
    RNode tip( 1.0/scale,  0.0, 0.0);

    std::vector<RNode> rim(nSides);
    for (uint k = 0; k < nSides; k++)
    {
        double angle = twoPi * k / nSides;
        rim[k] = RNode(1.0/scale - xRim, radius * std::cos(angle), radius * std::sin(angle));
        rim[k].scale(scale);
    }

    tail.scale(scale);
    tip.scale(scale);

    RSegment segment(RNode(v1[0],v1[1],v1[2]), RNode(v2[0],v2[1],v2[2]));
    RRMatrix R;
    segment.findRotationMatrix(R);
    R *= segment.findLength();

    tail.transform(R, v1);
    tip.transform(R, v1);
    for (uint k = 0; k < nSides; k++)
        rim[k].transform(R, v1);

    arrow.addSegment(tail, tip);
    for (uint k = 0; k < nSides; k++)
        arrow.addTriangle(tip, rim[(k + 1) % nSides], rim[k]);

    return arrow;
}
