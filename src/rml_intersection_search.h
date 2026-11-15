#ifndef RML_INTERSECTION_SEARCH_H
#define RML_INTERSECTION_SEARCH_H

#include <array>
#include <cstdint>
#include <vector>

#include "rml_element.h"
#include "rml_triangle.h"

//! Read-only, operation-local broad phase and cached surface geometry.
//! Slots refer to positions in elementIDs, rather than model element IDs.
class RIntersectionSearch
{
    public:

        struct Statistics;

    private:

        struct Primitive
        {
            std::array<double, 6> bounds = {};
            QList<RTriangle> triangles;
            bool active = false;
        };

        struct Branch
        {
            std::array<double, 6> bounds = {};
            size_t begin = 0, end = 0, left = 0, right = 0;
        };

        const std::vector<RElement> &elements;
        const std::vector<RNode> &nodes;
        std::vector<uint> elementIDs;
        std::vector<Primitive> primitives;
        std::vector<uint> order;
        std::vector<Branch> branches;
        double preparationSeconds = 0.0;

        size_t build(size_t begin, size_t end);
        template <typename Visit>
        void visitPairs(Visit visit, Statistics *statistics) const;
        bool isElementNode(uint slot, const RR3Vector &point, double tolerance) const;

    public:

        struct Statistics
        {
            uint64_t candidates = 0;
            uint64_t tests = 0;
            double preparationSeconds = 0.0;
            double searchSeconds = 0.0;
        };

        RIntersectionSearch(const std::vector<RElement> &elements, const std::vector<RNode> &nodes,
                            const std::vector<uint> &elementIDs, bool skipDuplicateNodes = false);
        //! Conservative, canonical pairs: only slots greater than the query slot.
        void candidates(uint slot, std::vector<uint> &matches) const;
        //! Surface pairs reuse cached triangulations; other supported kinds use the
        //! existing generic element routine. Pair scaling and tolerances are retained.
        bool findIntersection(uint first, uint second, QList<RR3Vector> &points, bool testOnly) const;
        std::vector<unsigned char> detect(Statistics *statistics = nullptr) const;
        std::vector<QList<RR3Vector>> findPoints(double tolerance, Statistics *statistics = nullptr) const;
};

#endif
