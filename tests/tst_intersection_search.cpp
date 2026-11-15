#include <QTest>
#include <QScopeGuard>
#include <array>
#include <cmath>
#include <numeric>
#include <random>
#include <set>
#include <omp.h>

#include <rbl_progress.h>
#include <rbl_utils.h>

#include "rml_model.h"
#include "../src/rml_intersection_search.h"

namespace
{

    using Point = std::array<double, 3>;
    using Triangle = std::array<Point, 3>;

    void add(RModel &model, const Triangle &points, bool share = false)
    {
        RElement element(R_ELEMENT_TRI1);
        for (uint i = 0; i < 3; i++)
        {
            uint id = model.getNNodes();
            if (share)
            {
                for (uint n = 0; n < model.getNNodes(); n++)
                {
                    const auto &node = model.getNode(n);
                    if (node.getX() == points[i][0] && node.getY() == points[i][1] && node.getZ() == points[i][2])
                    {
                        id = n;
                        break;
                    }
                }
            }
            if (id == model.getNNodes())
            {
                model.addNode(RNode(points[i][0], points[i][1], points[i][2]));
            }
            element.setNodeId(i, id);
        }
        model.addElement(element, true, 0);
    }

    std::vector<uint> ids(const RModel &model)
    {
        std::vector<uint> result(model.getNElements());
        std::iota(result.begin(), result.end(), uint(0));
        return result;
    }

    std::vector<unsigned char> brute(const RModel &model)
    {
        std::vector<unsigned char> result(model.getNElements(), 0);
        for (uint i = 0; i < model.getNElements(); i++)
        {
            for (uint j = i + 1; j < model.getNElements(); j++)
            {
                RLimitBox a, b;
                model.getElement(i).findLimitBox(model.getNodes(), a);
                model.getElement(j).findLimitBox(model.getNodes(), b);
                if (!RLimitBox::areIntersecting(a, b))
                {
                    continue;
                }
                QList<RR3Vector> points;
                if (RElement::findIntersectionPoints(model.getElement(i), model.getElement(j), model.getNodes(), points, true))
                {
                    result[i] = result[j] = 1;
                }
            }
        }
        return result;
    }

    RModel randomModel(double scale = 1.0)
    {
        RModel model;
        std::mt19937 random(51237);
        std::uniform_real_distribution<double> value(-2, 2);
        for (int i = 0; i < 100; i++)
        {
            Triangle triangle;
            for (auto &point : triangle)
            {
                for (double &coordinate : point)
                {
                    coordinate = value(random) * scale;
                }
            }
            if (i % 4 == 0)
            {
                for (auto &point : triangle)
                {
                    point[2] = 0;
                }
            }
            add(model, triangle);
        }
        add(model, Triangle{{Point{20 * scale, 0, 0}, Point{21 * scale, 0, 0}, Point{20 * scale, scale, 0}}});
        add(model, Triangle{{Point{-20 * scale, 0, 0}, Point{-21 * scale, 0, 0}, Point{-20 * scale, scale, 0}}});
        return model;
    }

}

class TestIntersectionSearch : public QObject
{
    Q_OBJECT

    private slots:

        void candidatesMatchBoxes_data()
        {
            QTest::addColumn<double>("scale");
            QTest::newRow("small") << 1e-6;
            QTest::newRow("unit") << 1.0;
            QTest::newRow("large") << 1e6;
        }

        void candidatesMatchBoxes()
        {
            QFETCH(double, scale);
            auto model = randomModel(scale);
            // Zero-thickness boxes separated by less than the existing tolerance.
            add(model, Triangle{{Point{10, 0, 0}, Point{11, 0, 0}, Point{10, 1, 0}}});
            add(model, Triangle{{Point{10, 0, RConstants::eps * .5}, Point{11, 0, RConstants::eps * .5}, Point{10, 1, RConstants::eps * .5}}});
            const RIntersectionSearch search(model.getElements(), model.getNodes(), ids(model));
            for (uint i = 0; i < model.getNElements(); i++)
            {
                std::vector<uint> actual, expected;
                search.candidates(i, actual);
                RLimitBox a;
                model.getElement(i).findLimitBox(model.getNodes(), a);
                for (uint j = i + 1; j < model.getNElements(); j++)
                {
                    RLimitBox b;
                    model.getElement(j).findLimitBox(model.getNodes(), b);
                    if (RLimitBox::areIntersecting(a, b))
                    {
                        expected.push_back(j);
                    }
                }
                QCOMPARE(actual, expected);
            }
        }

        void predicatesMatchLegacy()
        {
            std::mt19937 random(87324);
            std::uniform_real_distribution<double> value(-2, 2);
            for (int sample = 0; sample < 2000; sample++)
            {
                Triangle first, second;
                double scale = sample % 3 == 0 ? 1e-6 : sample % 3 == 1 ? 1.0
                                                                        : 1e6;
                for (auto *triangle : {&first, &second})
                {
                    for (auto &point : *triangle)
                    {
                        for (double &coordinate : point)
                        {
                            coordinate = value(random) * scale;
                        }
                    }
                }
                if (sample % 5 == 0)
                {
                    for (auto *triangle : {&first, &second})
                    {
                        for (auto &point : *triangle)
                        {
                            point[2] = 0;
                        }
                    }
                }
                if (sample % 7 == 0)
                {
                    second[0] = first[0];
                }
                if (sample % 11 == 0)
                {
                    second[0] = first[0];
                    second[1] = first[1];
                }
                if (sample % 13 == 0)
                {
                    second = first;
                }
                if (sample % 17 == 0)
                {
                    second[2] = second[1];
                }
                RModel model;
                add(model, first, true);
                add(model, second, true);
                const auto a = model.getElement(0).triangulate(model.getNodes()).front();
                const auto b = model.getElement(1).triangulate(model.getNodes()).front();
                QList<RR3Vector> ignored;
                QCOMPARE(a.intersects(b), a.findTriangleIntersection(b, ignored, true));
                QCOMPARE(b.intersects(a), b.findTriangleIntersection(a, ignored, true));
                const RIntersectionSearch search(model.getElements(), model.getNodes(), ids(model));
                QList<RR3Vector> expected, actual;
                const bool reference = RElement::findIntersectionPoints(model.getElement(0), model.getElement(1), model.getNodes(), expected, false);
                QCOMPARE(search.findIntersection(0, 1, actual, false), reference);
                QCOMPARE(actual.size(), expected.size());
                for (qsizetype p = 0; p < actual.size(); p++)
                {
                    for (int axis = 0; axis < 3; axis++)
                    {
                        QCOMPARE(actual[p][axis], expected[p][axis]);
                    }
                }
                QCOMPARE(search.findIntersection(0, 1, ignored, true), RElement::findIntersectionPoints(model.getElement(0), model.getElement(1), model.getNodes(), ignored, true));
            }
        }

        void nearParallelAndTouching()
        {
            for (double scale : {1e-6, 1.0, 1e6})
            {
                for (double slope : {0.0, 0.5e-5, 1e-5, 2e-5})
                {
                    for (double offset : {-RConstants::eps, 0.0, RConstants::eps, slope * .5})
                    {
                        RModel model;
                        add(model, Triangle{{Point{0, 0, 0}, Point{2 * scale, 0, 0}, Point{0, 2 * scale, 0}}});
                        add(model, Triangle{{Point{.2 * scale, .2 * scale, -offset * scale}, Point{1.2 * scale, .2 * scale, -offset * scale}, Point{.2 * scale, 1.2 * scale, (slope - offset) * scale}}});
                        const auto first = model.getElement(0).triangulate(model.getNodes()).front();
                        const auto second = model.getElement(1).triangulate(model.getNodes()).front();
                        QList<RR3Vector> ignored;
                        QCOMPARE(first.intersects(second), first.findTriangleIntersection(second, ignored, true));
                        QCOMPARE(second.intersects(first), second.findTriangleIntersection(first, ignored, true));
                        const RIntersectionSearch search(model.getElements(), model.getNodes(), ids(model));
                        QCOMPARE(search.findIntersection(0, 1, ignored, true), RElement::findIntersectionPoints(model.getElement(0), model.getElement(1), model.getNodes(), ignored, true));
                    }
                }
            }
        }

        void parallelDetection()
        {
            auto model = randomModel();
            const auto expected = brute(model);
            const RIntersectionSearch search(model.getElements(), model.getNodes(), ids(model));
            const int original = omp_get_max_threads();
            const auto restore = qScopeGuard([&]()
                                             {
                                                 omp_set_num_threads(original);
                                             });
            for (int workers : {1, 2, 4})
            {
                omp_set_num_threads(workers);
                RIntersectionSearch::Statistics statistics;
                QCOMPARE(search.detect(&statistics), expected);
                QVERIFY(statistics.candidates > 0);
                QVERIFY(statistics.tests <= statistics.candidates);
                QList<uint> indices;
                for (uint i = 0; i < expected.size(); i++)
                {
                    if (expected[i])
                    {
                        indices.append(i);
                    }
                }
                QCOMPARE(model.findIntersectedElements(), indices);
            }
        }

        void deterministicPointMerge()
        {
            const auto model = randomModel();
            const RIntersectionSearch search(model.getElements(), model.getNodes(), ids(model));
            const int original = omp_get_max_threads();
            const auto restore = qScopeGuard([&]()
                                             {
                                                 omp_set_num_threads(original);
                                             });
            omp_set_num_threads(1);
            const auto expected = search.findPoints(1e-9);
            omp_set_num_threads(4);
            const auto actual = search.findPoints(1e-9);
            QCOMPARE(actual.size(), expected.size());
            for (size_t i = 0; i < actual.size(); i++)
            {
                QCOMPARE(actual[i].size(), expected[i].size());
                for (qsizetype j = 0; j < actual[i].size(); j++)
                {
                    for (int axis = 0; axis < 3; axis++)
                    {
                        QCOMPARE(actual[i][j][axis], expected[i][j][axis]);
                    }
                }
            }
        }

        void mixedElements()
        {
            auto model = randomModel();
            // A point and a line use the established generic element predicate.
            const uint start = model.getNNodes();
            for (const auto &point : std::vector<Point>{{0, 0, 0}, {0, 0, -3}, {0, 0, 3}})
            {
                model.addNode(RNode(point[0], point[1], point[2]));
            }
            RElement point(R_ELEMENT_POINT);
            point.setNodeId(0, start);
            model.addElement(point, true, 0);
            RElement line(R_ELEMENT_TRUSS1);
            line.setNodeId(0, start + 1);
            line.setNodeId(1, start + 2);
            model.addElement(line, true, 0);
            const uint q = model.getNNodes();
            for (const auto &point : std::vector<Point>{{-2, -2, 0}, {2, -2, 0}, {2, 2, 0}, {-2, 2, 0}})
            {
                model.addNode(RNode(point[0], point[1], point[2]));
            }
            RElement quad(R_ELEMENT_QUAD1);
            for (uint i = 0; i < 4; i++)
            {
                quad.setNodeId(i, q + i);
            }
            model.addElement(quad, true, 0);
            const RIntersectionSearch search(model.getElements(), model.getNodes(), ids(model));
            QCOMPARE(search.detect(), brute(model));
        }

        void splittingPointsAndRebuild()
        {
            RModel model;
            add(model, Triangle{{Point{0, 0, 0}, Point{2, 0, 0}, Point{0, 2, 0}}});
            add(model, Triangle{{Point{.5, .5, -1}, Point{.5, .5, 1}, Point{1.5, .5, 0}}});
            const RIntersectionSearch search(model.getElements(), model.getNodes(), ids(model));
            const int original = omp_get_max_threads();
            const auto restore = qScopeGuard([&]()
                                             {
                                                 omp_set_num_threads(original);
                                             });
            std::vector<QList<RR3Vector>> first;
            for (int workers : {1, 4})
            {
                omp_set_num_threads(workers);
                const auto points = search.findPoints(1e-9);
                QCOMPARE(points.size(), size_t(2));
                QCOMPARE(points[0].size(), qsizetype(2));
                QCOMPARE(points[1].size(), qsizetype(1)); // Its existing vertex is not inserted again.
                if (first.empty())
                {
                    first = points;
                }
                else
                {
                    for (size_t i = 0; i < points.size(); i++)
                    {
                        for (qsizetype j = 0; j < points[i].size(); j++)
                        {
                            QVERIFY(RR3Vector::findDistance(points[i][j], first[i][j]) < 1e-12);
                        }
                    }
                }
            }
            QVERIFY(model.breakIntersectedElements(3) > 0);
            QVERIFY(model.findIntersectedElements().empty());
            double area = 0;
            for (const auto &element : model.getElements())
            {
                for (const auto &triangle : element.triangulate(model.getNodes()))
                {
                    area += triangle.findArea();
                }
            }
            QVERIFY(std::fabs(area - 3.0) < 1e-9);
        }

        void emptyAndSingleton()
        {
            RModel empty;
            QCOMPARE(empty.findIntersectedElements().size(), qsizetype(0));
            const RIntersectionSearch none(empty.getElements(), empty.getNodes(), ids(empty));
            QCOMPARE(none.detect().size(), size_t(0));
            QVERIFY(none.findPoints(1e-9).empty());
            add(empty, Triangle{{Point{0, 0, 0}, Point{1, 0, 0}, Point{0, 1, 0}}});
            QVERIFY(empty.findIntersectedElements().empty());
        }
};

QTEST_APPLESS_MAIN(TestIntersectionSearch)
#include "tst_intersection_search.moc"
