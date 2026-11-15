#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <exception>
#include <limits>
#include <map>
#include <set>
#include <mutex>
#include <omp.h>

#include <rbl_progress.h>
#include <rbl_utils.h>

#include "rml_intersection_search.h"

namespace
{

    using Bounds = std::array<double, 6>;
    using Clock = std::chrono::steady_clock;

    bool overlap(const Bounds &a, const Bounds &b)
    {
        // Same subtraction and tolerance as RLimitBox::areIntersecting().
        for (size_t axis = 0; axis < 3; axis++)
        {
            if (a[axis] - b[axis + 3] > RConstants::eps || b[axis] - a[axis + 3] > RConstants::eps)
            {
                return false;
            }
        }
        return true;
    }

    int workerCount(size_t count)
    {
        return int(std::min(size_t(std::max(1, omp_get_max_threads())), std::max(size_t(1), (count + 31) / 32)));
    }

    void addUnique(QList<RR3Vector> &points, const RR3Vector &point, double tolerance)
    {
        for (const auto &existing : std::as_const(points))
        {
            if (RR3Vector::findDistance(existing, point) < tolerance)
            {
                return;
            }
        }
        points.append(point);
    }

    class Progress
    {
        size_t total;
        std::atomic<size_t> completed{0};
        std::mutex mutex;
        Clock::time_point lastUpdate = Clock::now();

        public:

            explicit Progress(size_t total)
                : total(total)
            {
            }

            void advance(size_t count)
            {
                completed.fetch_add(count, std::memory_order_relaxed);
                if (!mutex.try_lock())
                {
                    return;
                }
                const std::lock_guard<std::mutex> lock(mutex, std::adopt_lock);
                const auto now = Clock::now();
                const size_t done = completed.load(std::memory_order_relaxed);
                if (total > 0 && (done == total || now - lastUpdate >= std::chrono::milliseconds(250)))
                {
                    RProgressPrint(uint64_t(done), uint64_t(total));
                    lastUpdate = now;
                }
            }
    };

}

RIntersectionSearch::RIntersectionSearch(const std::vector<RElement> &elements, const std::vector<RNode> &nodes,
                                         const std::vector<uint> &elementIDs, bool skipDuplicateNodes)
    : elements(elements)
    , nodes(nodes)
    , elementIDs(elementIDs)
    , primitives(elementIDs.size())
{
    const auto start = Clock::now();
    for (size_t slot = 0; slot < elementIDs.size(); slot++)
    {
        const auto &element = elements[elementIDs[slot]];
        const auto type = element.getType();
        if ((!R_ELEMENT_TYPE_IS_POINT(type) && !R_ELEMENT_TYPE_IS_LINE(type) && !R_ELEMENT_TYPE_IS_SURFACE(type)) || (skipDuplicateNodes && element.hasDuplicateNodes()))
        {
            continue;
        }
        RLimitBox box;
        element.findLimitBox(nodes, box);
        auto &primitive = primitives[slot];
        primitive.active = true;
        box.getLimits(primitive.bounds[0], primitive.bounds[3], primitive.bounds[1], primitive.bounds[4], primitive.bounds[2], primitive.bounds[5]);
        if (!std::all_of(primitive.bounds.begin(), primitive.bounds.end(), [](double value)
                         {
                             return std::isfinite(value);
                         }))
        {
            // Invalid coordinates must not turn the BVH into a false-negative filter
            // or violate the ordering used to partition the tree.
            const double infinity = std::numeric_limits<double>::infinity();
            primitive.bounds = {-infinity, -infinity, -infinity, infinity, infinity, infinity};
        }
        if (R_ELEMENT_TYPE_IS_SURFACE(type))
        {
            primitive.triangles = element.triangulate(nodes);
        }
        order.push_back(uint(slot));
    }
    branches.reserve(order.size() * 2);
    if (!order.empty())
    {
        build(0, order.size());
    }
    preparationSeconds = std::chrono::duration<double>(Clock::now() - start).count();
}

size_t RIntersectionSearch::build(size_t begin, size_t end)
{
    const size_t id = branches.size();
    branches.push_back(Branch());
    Bounds bounds = primitives[order[begin]].bounds;
    for (size_t i = begin + 1; i < end; i++)
    {
        for (size_t axis = 0; axis < 3; axis++)
        {
            const auto &box = primitives[order[i]].bounds;
            bounds[axis] = std::min(bounds[axis], box[axis]);
            bounds[axis + 3] = std::max(bounds[axis + 3], box[axis + 3]);
        }
    }
    branches[id].bounds = bounds;
    branches[id].begin = begin;
    branches[id].end = end;
    if (end - begin <= 8)
    {
        return id;
    }
    size_t axis = 0;
    for (size_t i = 1; i < 3; i++)
    {
        if (bounds[i + 3] - bounds[i] > bounds[axis + 3] - bounds[axis])
        {
            axis = i;
        }
    }
    const size_t middle = begin + (end - begin) / 2;
    auto center = [&](uint index)
    {
        const auto &box = primitives[index].bounds;
        const double value = box[axis] * .5 + box[axis + 3] * .5;
        return std::isfinite(value) ? value : 0.0;
    };
    std::nth_element(order.begin() + begin, order.begin() + middle, order.begin() + end, [&](uint a, uint b)
                     {
                         const double ca = center(a), cb = center(b);
                         return ca == cb ? a < b : ca < cb;
                     });
    const size_t left = build(begin, middle), right = build(middle, end);
    branches[id].left = left;
    branches[id].right = right;
    return id;
}

void RIntersectionSearch::candidates(uint slot, std::vector<uint> &matches) const
{
    matches.clear();
    if (branches.empty() || !primitives[slot].active)
    {
        return;
    }
    const auto &query = primitives[slot].bounds;
    // Median splits bound the depth by the number of bits in size_t.
    std::array<size_t, 128> stack;
    size_t count = 1;
    stack[0] = 0;
    while (count > 0)
    {
        const auto &branch = branches[stack[--count]];
        if (!overlap(query, branch.bounds))
        {
            continue;
        }
        if (branch.left == 0)
        {
            for (size_t i = branch.begin; i < branch.end; i++)
            {
                const uint other = order[i];
                if (other > slot && overlap(query, primitives[other].bounds))
                {
                    matches.push_back(other);
                }
            }
        }
        else
        {
            stack[count++] = branch.left;
            stack[count++] = branch.right;
        }
    }
    std::sort(matches.begin(), matches.end());
}

bool RIntersectionSearch::findIntersection(uint first, uint second, QList<RR3Vector> &points, bool testOnly) const
{
    const auto &a = primitives[first], &b = primitives[second];
    if (a.triangles.empty() || b.triangles.empty() || !std::isfinite(a.bounds[0]) || !std::isfinite(b.bounds[0]))
    {
        return RElement::findIntersectionPoints(elements[elementIDs[first]], elements[elementIDs[second]], nodes, points, testOnly);
    }
    double scale = std::numeric_limits<double>::infinity();
    for (size_t axis = 0; axis < 3; axis++)
    {
        const double span = std::abs(std::max(a.bounds[axis + 3], b.bounds[axis + 3]) - std::min(a.bounds[axis], b.bounds[axis]));
        scale = std::min(scale, span);
    }
    scale = scale <= RConstants::eps ? 1.0 : 1.0 / scale;
    bool found = false;
    QList<RR3Vector> intersections;
    for (const auto &originalA : a.triangles)
    {
        RTriangle triangleA(originalA);
        triangleA.getNode1().scale(scale);
        triangleA.getNode2().scale(scale);
        triangleA.getNode3().scale(scale);
        for (const auto &originalB : b.triangles)
        {
            RTriangle triangleB(originalB);
            triangleB.getNode1().scale(scale);
            triangleB.getNode2().scale(scale);
            triangleB.getNode3().scale(scale);
            if (testOnly)
            {
                if (triangleA.intersects(triangleB))
                {
                    return true;
                }
            }
            else if (triangleA.findTriangleIntersection(triangleB, intersections, false))
            {
                found = true;
            }
        }
    }
    if (!testOnly)
    {
        for (auto point : intersections)
        {
            point.scale(1.0 / scale);
            points.append(point);
        }
    }
    return found;
}

template <typename Visit>
void RIntersectionSearch::visitPairs(Visit visit, Statistics *statistics) const
{
    const auto start = Clock::now();
    Progress progress(elementIDs.size());
    std::atomic<bool> failed{false};
    std::exception_ptr failure;
    std::mutex failureMutex;

    struct Work
    {
        uint64_t candidates = 0, tests = 0;
    };

    const int nWorkers = workerCount(elementIDs.size());
    std::vector<Work> work(static_cast<size_t>(nWorkers));
#pragma omp parallel num_threads(nWorkers) if (nWorkers > 1)
    {
        const int thread = omp_get_thread_num();
        Work local;
        std::vector<uint> matches;
#pragma omp for schedule(dynamic, 1)
        for (int64_t begin = 0; begin < int64_t(elementIDs.size()); begin += 32)
        {
            if (failed.load(std::memory_order_relaxed))
            {
                continue;
            }
            const size_t end = std::min(size_t(begin) + 32, elementIDs.size());
            try
            {
                for (size_t i = size_t(begin); i < end; i++)
                {
                    candidates(uint(i), matches);
                    local.candidates += matches.size();
                    for (uint other : matches)
                    {
                        local.tests += visit(thread, uint(i), other);
                    }
                }
                progress.advance(end - size_t(begin));
            }
            catch (...)
            {
                const std::lock_guard<std::mutex> lock(failureMutex);
                if (!failure)
                {
                    failure = std::current_exception();
                }
                failed.store(true, std::memory_order_relaxed);
            }
        }
        work[size_t(thread)] = local;
    }
    if (failure)
    {
        std::rethrow_exception(failure);
    }
    if (statistics)
    {
        *statistics = Statistics();
        statistics->preparationSeconds = preparationSeconds;
        statistics->searchSeconds = std::chrono::duration<double>(Clock::now() - start).count();
        for (const auto &item : work)
        {
            statistics->candidates += item.candidates;
            statistics->tests += item.tests;
        }
    }
}

std::vector<unsigned char> RIntersectionSearch::detect(Statistics *statistics) const
{
    std::vector<std::vector<unsigned char>> flags(size_t(workerCount(elementIDs.size())), std::vector<unsigned char>(elementIDs.size(), 0));
    visitPairs([&](int thread, uint first, uint second)
               {
                   auto &local = flags[size_t(thread)];
                   if (local[first] && local[second])
                   {
                       return false;
                   }
                   QList<RR3Vector> ignored;
                   if (findIntersection(first, second, ignored, true))
                   {
                       local[first] = local[second] = 1;
                   }
                   return true;
               },
               statistics);
    std::vector<unsigned char> result(elementIDs.size(), 0);
    for (const auto &local : flags)
    {
        for (size_t i = 0; i < result.size(); i++)
        {
            result[i] |= local[i];
        }
    }
    return result;
}

bool RIntersectionSearch::isElementNode(uint slot, const RR3Vector &point, double tolerance) const
{
    const RNode candidate(point[0], point[1], point[2]);
    const auto &element = elements[elementIDs[slot]];
    for (uint i = 0; i < element.size(); i++)
    {
        if (nodes[element.getNodeId(i)].getDistance(candidate) < tolerance)
        {
            return true;
        }
    }
    return false;
}

std::vector<QList<RR3Vector>> RIntersectionSearch::findPoints(double tolerance, Statistics *statistics) const
{
    using Point = std::array<double, 3>;
    using Points = std::map<uint, std::set<Point>>;
    std::vector<Points> local(size_t(workerCount(elementIDs.size())));
    visitPairs([&](int thread, uint first, uint second)
               {
                   QList<RR3Vector> points;
                   if (findIntersection(first, second, points, false))
                   {
                       for (auto point = points.crbegin(); point != points.crend(); ++point)
                       {
                           for (uint slot : {first, second})
                           {
                               if (!isElementNode(slot, *point, tolerance))
                               {
                                   local[size_t(thread)][slot].insert({(*point)[0], (*point)[1], (*point)[2]});
                               }
                           }
                       }
                   }
                   return true;
               },
               statistics);
    std::vector<QList<RR3Vector>> result(elementIDs.size());
    // Only exact duplicates are removed in workers. Sorting the complete union
    // before tolerance merging keeps representatives independent of scheduling.
    for (const auto &worker : local)
    {
        for (const auto &entry : worker)
        {
            for (const auto &point : entry.second)
            {
                result[entry.first].append(RR3Vector(point[0], point[1], point[2]));
            }
        }
    }
    for (auto &points : result)
    {
        std::sort(points.begin(), points.end(), [](const RR3Vector &a, const RR3Vector &b)
                  {
                      for (size_t i = 0; i < 3; i++)
                      {
                          if (a[i] != b[i])
                          {
                              return a[i] < b[i];
                          }
                      }
                      return false;
                  });
        QList<RR3Vector> unique;
        for (const auto &point : std::as_const(points))
        {
            addUnique(unique, point, tolerance);
        }
        points = std::move(unique);
    }
    return result;
}
