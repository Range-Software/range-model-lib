#include <QElapsedTimer>
#include <QTest>

#include <algorithm>
#include <cmath>
#include <map>
#include <numeric>
#include <set>
#include <random>
#include <vector>

#include "rml_model.h"
#include "rml_model_stl.h"
#include "rml_surface_reconstructor.h"

class TestSurfaceReconstruction : public QObject
{
    Q_OBJECT

    private:

        //! Node ID of a point - nodes are shared by coordinates.
        struct Builder
        {
            RModel &model;
            std::map<std::array<long long,3>,uint> nodeMap;

            explicit Builder(RModel &model) : model(model) {}

            uint node(double x, double y, double z)
            {
                std::array<long long,3> key = {std::llround(x*1e9),std::llround(y*1e9),std::llround(z*1e9)};
                auto iter = this->nodeMap.find(key);
                if (iter != this->nodeMap.end())
                {
                    return iter->second;
                }
                this->model.addNode(RNode(x,y,z));
                uint nodeID = this->model.getNNodes() - 1;
                this->nodeMap[key] = nodeID;
                return nodeID;
            }

            void triangle(uint n1, uint n2, uint n3, uint surfaceID)
            {
                RElement element(R_ELEMENT_TRI1);
                element.setNodeId(0,n1);
                element.setNodeId(1,n2);
                element.setNodeId(2,n3);
                this->model.addElement(element,true,surfaceID);
            }

            //! Grid of n x n quads (2 triangles each) over parallelogram origin + s*u + t*v, normal = u x v.
            void grid(const RR3Vector &origin, const RR3Vector &u, const RR3Vector &v, uint n, uint surfaceID)
            {
                auto point = [&](uint i, uint j) -> uint
                {
                    double s = double(i)/double(n);
                    double t = double(j)/double(n);
                    return this->node(origin[0] + s*u[0] + t*v[0],origin[1] + s*u[1] + t*v[1],origin[2] + s*u[2] + t*v[2]);
                };
                for (uint i=0;i<n;i++)
                {
                    for (uint j=0;j<n;j++)
                    {
                        // Alternate diagonals to get irregular connectivity.
                        if ((i + j) % 2 == 0)
                        {
                            this->triangle(point(i,j),point(i+1,j),point(i+1,j+1),surfaceID);
                            this->triangle(point(i,j),point(i+1,j+1),point(i,j+1),surfaceID);
                        }
                        else
                        {
                            this->triangle(point(i,j),point(i+1,j),point(i,j+1),surfaceID);
                            this->triangle(point(i+1,j),point(i+1,j+1),point(i,j+1),surfaceID);
                        }
                    }
                }
            }
        };

        //! Outward oriented unit box made of 6 gridded faces.
        static void buildBox(Builder &builder, uint n, uint topSurfaceID = 0)
        {
            builder.grid(RR3Vector(0,0,0),RR3Vector(0,1,0),RR3Vector(1,0,0),n,0); // bottom (-z)
            builder.grid(RR3Vector(0,0,1),RR3Vector(1,0,0),RR3Vector(0,1,0),n,topSurfaceID); // top (+z)
            builder.grid(RR3Vector(0,0,0),RR3Vector(1,0,0),RR3Vector(0,0,1),n,0); // front (-y)
            builder.grid(RR3Vector(0,1,0),RR3Vector(0,0,1),RR3Vector(1,0,0),n,0); // back (+y)
            builder.grid(RR3Vector(0,0,0),RR3Vector(0,0,1),RR3Vector(0,1,0),n,0); // left (-x)
            builder.grid(RR3Vector(1,0,0),RR3Vector(0,1,0),RR3Vector(0,0,1),n,0); // right (+x)
        }

        static RR3Vector triangleNormal(const RModel &model, const RElement &element)
        {
            RR3Vector p0 = model.getNode(element.getNodeId(0)).toVector();
            RR3Vector p1 = model.getNode(element.getNodeId(1)).toVector();
            RR3Vector p2 = model.getNode(element.getNodeId(2)).toVector();
            RR3Vector e1(p1[0]-p0[0],p1[1]-p0[1],p1[2]-p0[2]);
            RR3Vector e2(p2[0]-p0[0],p2[1]-p0[1],p2[2]-p0[2]);
            RR3Vector n;
            RR3Vector::cross(e1,e2,n);
            return n;
        }

        //! Area vector (sum of raw normals / 2) and total area of the surface.
        static void surfaceArea(const RModel &model, uint surfaceID, RR3Vector &areaVector, double &area)
        {
            areaVector = RR3Vector(0,0,0);
            area = 0.0;
            const RSurface &surface = model.getSurface(surfaceID);
            for (uint i=0;i<surface.size();i++)
            {
                RR3Vector n = triangleNormal(model,model.getElement(surface.get(i)));
                for (uint k=0;k<3;k++)
                {
                    areaVector[k] += 0.5*n[k];
                }
                area += 0.5*n.length();
            }
        }

        //! Enclosed volume of closed surfaces (divergence theorem).
        static double enclosedVolume(const RModel &model)
        {
            double volume = 0.0;
            for (uint i=0;i<model.getNElements();i++)
            {
                const RElement &element = model.getElement(i);
                if (element.getType() != R_ELEMENT_TRI1)
                {
                    continue;
                }
                RR3Vector p0 = model.getNode(element.getNodeId(0)).toVector();
                RR3Vector n = triangleNormal(model,element);
                volume += RR3Vector::dot(p0,n) / 6.0;
            }
            return volume;
        }

        //! Smallest angle of all triangles in model.
        static double minimumAngle(const RModel &model)
        {
            double minAngle = RConstants::pi;
            for (uint i=0;i<model.getNElements();i++)
            {
                const RElement &element = model.getElement(i);
                if (element.getType() != R_ELEMENT_TRI1)
                {
                    continue;
                }
                for (uint k=0;k<3;k++)
                {
                    RR3Vector p = model.getNode(element.getNodeId(k)).toVector();
                    RR3Vector a = model.getNode(element.getNodeId((k+1)%3)).toVector();
                    RR3Vector b = model.getNode(element.getNodeId((k+2)%3)).toVector();
                    RR3Vector e1(a[0]-p[0],a[1]-p[1],a[2]-p[2]);
                    RR3Vector e2(b[0]-p[0],b[1]-p[1],b[2]-p[2]);
                    minAngle = std::min(minAngle,RR3Vector::angle(e1,e2));
                }
            }
            return minAngle;
        }

        //! Number of triangles with smallest angle below given value.
        static uint countBadTriangles(const RModel &model, double angle)
        {
            uint nBad = 0;
            for (uint i=0;i<model.getNElements();i++)
            {
                const RElement &element = model.getElement(i);
                if (element.getType() != R_ELEMENT_TRI1)
                {
                    continue;
                }
                for (uint k=0;k<3;k++)
                {
                    RR3Vector p = model.getNode(element.getNodeId(k)).toVector();
                    RR3Vector a = model.getNode(element.getNodeId((k+1)%3)).toVector();
                    RR3Vector b = model.getNode(element.getNodeId((k+2)%3)).toVector();
                    RR3Vector e1(a[0]-p[0],a[1]-p[1],a[2]-p[2]);
                    RR3Vector e2(b[0]-p[0],b[1]-p[1],b[2]-p[2]);
                    if (RR3Vector::angle(e1,e2) < angle)
                    {
                        nBad++;
                        break;
                    }
                }
            }
            return nBad;
        }

        static double findDiagonal(const std::vector<RNode> &nodes)
        {
            if (nodes.empty())
            {
                return 0.0;
            }
            double l[3] = {nodes[0].getX(),nodes[0].getY(),nodes[0].getZ()};
            double u[3] = {l[0],l[1],l[2]};
            for (const RNode &node : nodes)
            {
                const double p[3] = {node.getX(),node.getY(),node.getZ()};
                for (uint k=0;k<3;k++)
                {
                    l[k] = std::min(l[k],p[k]);
                    u[k] = std::max(u[k],p[k]);
                }
            }
            return std::sqrt((u[0]-l[0])*(u[0]-l[0]) + (u[1]-l[1])*(u[1]-l[1]) + (u[2]-l[2])*(u[2]-l[2]));
        }

        //! Number of interior edges which are not locally Delaunay (sum of opposite angles above 180 degrees)
        //! and could be flipped without changing geometry by more than distance tolerance.
        static uint countDelaunayViolations(const RModel &model, double distanceTolerance)
        {
            std::map<std::pair<uint,uint>,std::vector<uint>> edges;
            std::set<std::pair<uint,uint>> edgeSet;
            for (uint i=0;i<model.getNElements();i++)
            {
                const RElement &e = model.getElement(i);
                if (e.getType() != R_ELEMENT_TRI1)
                {
                    continue;
                }
                for (uint k=0;k<3;k++)
                {
                    uint a = e.getNodeId(k), b = e.getNodeId((k+1)%3);
                    edges[{std::min(a,b),std::max(a,b)}].push_back(i);
                }
            }
            auto point = [&](uint n) { return model.getNode(n).toVector(); };
            auto sub = [](const RR3Vector &a, const RR3Vector &b) { return RR3Vector(a[0]-b[0],a[1]-b[1],a[2]-b[2]); };
            auto opposite = [&](const RElement &e, uint a, uint b) -> uint
            {
                for (uint k=0;k<3;k++)
                {
                    if (e.getNodeId(k) != a && e.getNodeId(k) != b) return e.getNodeId(k);
                }
                return RConstants::eod;
            };
            uint nViolations = 0;
            for (const auto &edge : edges)
            {
                if (edge.second.size() != 2)
                {
                    continue;
                }
                uint a = edge.first.first, b = edge.first.second;
                uint r = opposite(model.getElement(edge.second[0]),a,b);
                uint s = opposite(model.getElement(edge.second[1]),a,b);
                if (edges.count({std::min(r,s),std::max(r,s)}))
                {
                    continue;
                }
                double alpha = RR3Vector::angle(sub(point(a),point(r)),sub(point(b),point(r)));
                double beta = RR3Vector::angle(sub(point(a),point(s)),sub(point(b),point(s)));
                if (alpha + beta <= RConstants::pi * (1.0 + 1.0e-9))
                {
                    continue;
                }
                RR3Vector nt = triangleNormal(model,model.getElement(edge.second[0]));
                RR3Vector nu = triangleNormal(model,model.getElement(edge.second[1]));
                double deviation = (nt.length() >= nu.length()) ? std::abs(RR3Vector::dot(sub(point(s),point(a)),nt)) / nt.length()
                                                                : std::abs(RR3Vector::dot(sub(point(r),point(a)),nu)) / nu.length();
                if (deviation <= distanceTolerance)
                {
                    nViolations++;
                }
            }
            return nViolations;
        }

        //! Sheared grid on gently curved surface z = k x^2 - long diagonals are not Delaunay.
        static void buildCurvedShearedGrid(Builder &builder, uint n, double k)
        {
            auto point = [&](uint i, uint j) -> uint
            {
                double x = double(i) + 0.8*double(j);
                double y = 0.3*double(j);
                return builder.node(x,y,k*x*x);
            };
            for (uint i=0;i<n;i++)
            {
                for (uint j=0;j<n;j++)
                {
                    builder.triangle(point(i,j),point(i+1,j),point(i+1,j+1),0);
                    builder.triangle(point(i,j),point(i+1,j+1),point(i,j+1),0);
                }
            }
        }

        //! Each edge is shared by exactly two triangles with opposite traversal.
        static bool isClosedAndConsistent(const RModel &model)
        {
            std::map<std::pair<uint,uint>,int> directedEdges;
            for (uint i=0;i<model.getNElements();i++)
            {
                const RElement &element = model.getElement(i);
                if (element.getType() != R_ELEMENT_TRI1)
                {
                    continue;
                }
                for (uint k=0;k<3;k++)
                {
                    directedEdges[{element.getNodeId(k),element.getNodeId((k+1)%3)}]++;
                }
            }
            for (const auto &edge : directedEdges)
            {
                if (edge.second != 1)
                {
                    return false;
                }
                auto reverse = directedEdges.find({edge.first.second,edge.first.first});
                if (reverse == directedEdges.end() || reverse->second != 1)
                {
                    return false;
                }
            }
            return true;
        }

    private slots:

        void planarPlateIsMergedToTwoTriangles()
        {
            RModel model;
            Builder builder(model);
            builder.grid(RR3Vector(0,0,0),RR3Vector(2,0,0),RR3Vector(0,1,0),8,0);
            QCOMPARE(model.getNElements(),128u);

            RSurfaceReconstructor::Statistics statistics = model.reconstructSurfaces({0});

            QCOMPARE(model.getNNodes(),4u);
            QCOMPARE(model.getNElements(),2u);
            QCOMPARE(model.getSurface(0).size(),2u);
            QCOMPARE(statistics.nInputTriangles,128u);
            QCOMPARE(statistics.nOutputTriangles,2u);
            QCOMPARE(statistics.nRemovedNodes,81u - 4u);

            RR3Vector areaVector;
            double area;
            surfaceArea(model,0,areaVector,area);
            QVERIFY(std::abs(area - 2.0) < 1e-12);
            // Orientation is preserved (+z).
            QVERIFY(std::abs(areaVector[2] - 2.0) < 1e-12);
        }

        void closedBoxIsReducedToCorners()
        {
            RModel model;
            Builder builder(model);
            buildBox(builder,6);
            QVERIFY(isClosedAndConsistent(model));
            QVERIFY(std::abs(enclosedVolume(model) - 1.0) < 1e-12);

            model.reconstructSurfaces({0});

            QCOMPARE(model.getNNodes(),8u);
            QCOMPARE(model.getNElements(),12u);
            QVERIFY(isClosedAndConsistent(model));
            QVERIFY(std::abs(enclosedVolume(model) - 1.0) < 1e-12);
        }

        void noisyBoxIsReducedToCorners_data()
        {
            QTest::addColumn<uint>("seed");
            for (uint seed=1;seed<=5;seed++)
            {
                QTest::newRow(qPrintable(QString("seed %1").arg(seed))) << seed;
            }
        }

        void noisyBoxIsReducedToCorners()
        {
            // Box faces with random out of plane noise well below the distance tolerance
            // and randomly flipped diagonals.
            QFETCH(uint,seed);
            std::mt19937 generator(seed);
            std::uniform_real_distribution<double> noise(-1.0e-8,1.0e-8);

            RModel model;
            Builder builder(model);
            buildBox(builder,7);
            for (uint i=0;i<model.getNNodes();i++)
            {
                RNode &node = model.getNode(i);
                double p[3] = {node.getX(),node.getY(),node.getZ()};
                uint nOnBoundary = 0;
                for (uint k=0;k<3;k++)
                {
                    nOnBoundary += (p[k] == 0.0 || p[k] == 1.0) ? 1 : 0;
                }
                if (nOnBoundary == 3)
                {
                    continue;
                }
                for (uint k=0;k<3;k++)
                {
                    if (p[k] == 0.0 || p[k] == 1.0)
                    {
                        p[k] += noise(generator);
                    }
                }
                node.set(p[0],p[1],p[2]);
            }
            QVERIFY(isClosedAndConsistent(model));

            model.reconstructSurfaces({0});

            QCOMPARE(model.getNNodes(),8u);
            QCOMPARE(model.getNElements(),12u);
            QVERIFY(isClosedAndConsistent(model));
            QVERIFY(std::abs(enclosedVolume(model) - 1.0) < 1e-12);
        }

        void boundaryBetweenSelectedSurfacesIsSimplified()
        {
            RModel model;
            Builder builder(model);
            buildBox(builder,4,1);

            model.reconstructSurfaces({0,1});

            QCOMPARE(model.getNNodes(),8u);
            QCOMPARE(model.getNElements(),12u);
            QCOMPARE(model.getSurface(1).size(),2u);
            QCOMPARE(model.getSurface(0).size(),10u);
            QVERIFY(isClosedAndConsistent(model));
        }

        void unselectedSurfaceIsPreserved()
        {
            RModel model;
            Builder builder(model);
            const uint n = 4;
            buildBox(builder,n,1);
            uint nTopElements = model.getSurface(1).size();

            model.reconstructSurfaces({0});

            // Top surface keeps all its elements and the box stays watertight - all nodes
            // of the top surface (including its boundary) must be preserved.
            QCOMPARE(model.getSurface(1).size(),nTopElements);
            QCOMPARE(model.getNNodes(),(n+1)*(n+1) + 4u);
            QVERIFY(isClosedAndConsistent(model));
            QVERIFY(std::abs(enclosedVolume(model) - 1.0) < 1e-12);
        }

        void lineElementIsPreserved()
        {
            RModel model;
            Builder builder(model);
            builder.grid(RR3Vector(0,0,0),RR3Vector(1,0,0),RR3Vector(0,1,0),4,0);

            // Line along y = 0.5 through 5 grid nodes.
            for (uint i=0;i<4;i++)
            {
                RElement line(R_ELEMENT_TRUSS1);
                line.setNodeId(0,builder.node(i/4.0,0.5,0.0));
                line.setNodeId(1,builder.node((i+1)/4.0,0.5,0.0));
                model.addElement(line,true,0);
            }

            model.reconstructSurfaces({0});

            QCOMPARE(model.getNLines(),1u);
            QCOMPARE(model.getLine(0).size(),4u);
            // 4 corners + 5 line nodes.
            QCOMPARE(model.getNNodes(),9u);

            // Every line segment is an edge of the surface triangulation.
            for (uint i=0;i<model.getLine(0).size();i++)
            {
                const RElement &line = model.getElement(model.getLine(0).get(i));
                uint nSharing = 0;
                for (uint j=0;j<model.getSurface(0).size();j++)
                {
                    const RElement &triangle = model.getElement(model.getSurface(0).get(j));
                    if (triangle.hasNodeId(line.getNodeId(0)) && triangle.hasNodeId(line.getNodeId(1)))
                    {
                        nSharing++;
                    }
                }
                QCOMPARE(nSharing,2u);
            }

            RR3Vector areaVector;
            double area;
            surfaceArea(model,0,areaVector,area);
            QVERIFY(std::abs(area - 1.0) < 1e-12);
        }

        void curvedSurfaceKeepsShape()
        {
            // Open cylinder - 16 segments, 4 rings. Each facet strip is planar, so the middle
            // rings (collinear along generators) are removed, the shape is kept.
            RModel model;
            Builder builder(model);
            const uint nSegments = 16;
            const uint nRings = 4;
            const double radius = 1.0;
            for (uint i=0;i<nSegments;i++)
            {
                double a0 = 2.0*RConstants::pi*double(i)/double(nSegments);
                double a1 = 2.0*RConstants::pi*double(i+1)/double(nSegments);
                for (uint j=0;j<nRings;j++)
                {
                    double z0 = double(j)/double(nRings);
                    double z1 = double(j+1)/double(nRings);
                    uint n00 = builder.node(radius*std::cos(a0),radius*std::sin(a0),z0);
                    uint n10 = builder.node(radius*std::cos(a1),radius*std::sin(a1),z0);
                    uint n11 = builder.node(radius*std::cos(a1),radius*std::sin(a1),z1);
                    uint n01 = builder.node(radius*std::cos(a0),radius*std::sin(a0),z1);
                    builder.triangle(n00,n10,n11,0);
                    builder.triangle(n00,n11,n01,0);
                }
            }
            RR3Vector areaVectorBefore;
            double areaBefore;
            surfaceArea(model,0,areaVectorBefore,areaBefore);

            model.reconstructSurfaces({0});

            QCOMPARE(model.getNNodes(),2u*nSegments);
            QCOMPARE(model.getNElements(),2u*nSegments);
            for (uint i=0;i<model.getNNodes();i++)
            {
                const RNode &node = model.getNode(i);
                QVERIFY(std::abs(std::hypot(node.getX(),node.getY()) - radius) < 1e-12);
            }
            RR3Vector areaVector;
            double area;
            surfaceArea(model,0,areaVector,area);
            QVERIFY(std::abs(area - areaBefore) < 1e-12);
            // Outward orientation is preserved.
            for (uint i=0;i<model.getNElements();i++)
            {
                const RElement &element = model.getElement(i);
                RR3Vector n = triangleNormal(model,element);
                RR3Vector p = model.getNode(element.getNodeId(0)).toVector();
                QVERIFY(n[0]*p[0] + n[1]*p[1] > 0.0);
            }
        }

        void sliversAndTinyTrianglesAreRemoved()
        {
            // Two non-coplanar faces (sharp 90 degree edge) so that the slivers sit
            // on a feature line and in curved-like configuration.
            RModel model;
            Builder builder(model);

            // Horizontal face z = 0, x in [0,1], y in [0,1]; edge y = 1 is shared with vertical face.
            uint a = builder.node(0,0,0);
            uint b = builder.node(1,0,0);
            uint c = builder.node(1,1,0);
            uint d = builder.node(0,1,0);
            // Node almost on edge c-d (cap sliver) and node very close to corner a (tiny edge).
            uint e = builder.node(0.5,1.0-1e-9,0);
            uint f = builder.node(1e-9,1e-9,0);
            builder.triangle(f,b,e,0);
            builder.triangle(b,c,e,0);
            builder.triangle(e,d,f,0);
            builder.triangle(a,b,f,0);
            builder.triangle(d,a,f,0);
            // Vertical face y = 1, z in [0,1].
            uint g = builder.node(1,1,1);
            uint h = builder.node(0,1,1);
            builder.triangle(c,g,h,0);
            builder.triangle(c,h,e,0);
            builder.triangle(e,h,d,0);

            QVERIFY(minimumAngle(model) < R_DEG_TO_RAD(1.0));

            model.reconstructSurfaces({0});

            QCOMPARE(model.getNNodes(),6u);
            QCOMPARE(model.getNElements(),4u);
            QVERIFY(minimumAngle(model) > R_DEG_TO_RAD(30.0));
            RR3Vector areaVector;
            double area;
            surfaceArea(model,0,areaVector,area);
            QVERIFY(std::abs(area - 2.0) < 1e-6);
        }

        void needleOnCurvedSurfaceIsCollapsed()
        {
            // Sliver triangle inside a non-planar (tent) surface, short edge is collapsed
            // only when geometry stays within tolerance.
            RModel model;
            Builder builder(model);
            uint a = builder.node(0,0,0);
            uint b = builder.node(2,0,0);
            uint c = builder.node(2,2,0);
            uint d = builder.node(0,2,0);
            uint top = builder.node(1,1,0.5);
            uint topNear = builder.node(1.0,1.0+1e-3,0.5);
            // Apex split into two nodes connected by a short ridge edge.
            builder.triangle(a,b,top,0);
            builder.triangle(b,c,topNear,0);
            builder.triangle(b,topNear,top,0);
            builder.triangle(c,d,topNear,0);
            builder.triangle(d,a,top,0);
            builder.triangle(d,top,topNear,0);

            RSurfaceReconstructor::Options options;
            options.distanceTolerance = 1e-6;
            options.minEdgeLength = 1e-2;
            model.reconstructSurfaces({0},options);

            QCOMPARE(model.getNElements(),4u);
            QCOMPARE(model.getNNodes(),5u);
        }

        void realModels_data()
        {
            QTest::addColumn<QString>("fileName");
            QTest::addColumn<double>("relativeTolerance");
            QTest::addColumn<double>("angleTolerance");
            QTest::addColumn<bool>("delaunay");
            QTest::addColumn<bool>("refine");
            const QStringList names = {"SimpleUnderBonnetGeom.stl","Shaft.stl","Chabot.stl","Multi-body.stl",
                                       "Angle.rbm","Beam.rbm","Channel.rbm","Connecting_Rod.rbm",
                                       "Core.rbm","Dam.rbm","Exhaust.rbm","Handle.rbm","Plane.rbm","Rod.rbm",
                                       "Sample.rbm","Sphere.rbm","Spindle.rbm","Support.rbm"};
            for (const QString &name : names)
            {
                QTest::newRow(qPrintable(name + " default")) << name << 0.0 << 1.0 << false << false;
                QTest::newRow(qPrintable(name + " default delaunay")) << name << 0.0 << 1.0 << true << false;
                QTest::newRow(qPrintable(name + " default refine")) << name << 0.0 << 1.0 << false << true;
                QTest::newRow(qPrintable(name + " coarse")) << name << 1.0e-3 << 5.0 << false << false;
                QTest::newRow(qPrintable(name + " coarse delaunay")) << name << 1.0e-3 << 5.0 << true << false;
                QTest::newRow(qPrintable(name + " coarse refine")) << name << 1.0e-3 << 5.0 << false << true;
                QTest::newRow(qPrintable(name + " aggressive")) << name << 1.0e-2 << 15.0 << false << false;
            }
        }

        void realModels()
        {
            QFETCH(QString,fileName);
            QFETCH(double,relativeTolerance);
            QFETCH(double,angleTolerance);
            QFETCH(bool,delaunay);
            QFETCH(bool,refine);
            const QString filePath = QFINDTESTDATA("../../fea/data/" + fileName);
            if (filePath.isEmpty())
            {
                QSKIP("Model file not found");
            }

            RModel model;
            if (fileName.endsWith(".stl"))
            {
                RModelStl modelStl;
                modelStl.read(filePath);
                model = RModel(modelStl);
            }
            else
            {
                model.read(filePath);
            }
            // Drop volume mesh - surfaces are reconstructed before meshing.
            while (model.getNVolumes() > 0)
            {
                model.removeVolume(0);
            }
            model.purgeUnusedElements();
            model.purgeUnusedNodes();

            std::vector<uint> surfaceIDs(model.getNSurfaces());
            std::iota(surfaceIDs.begin(),surfaceIDs.end(),0u);
            if (surfaceIDs.empty())
            {
                QSKIP("Model has no surfaces");
            }

            const bool closedBefore = isClosedAndConsistent(model);
            const double volumeBefore = enclosedVolume(model);
            double areaBefore = 0.0;
            for (uint surfaceID : surfaceIDs)
            {
                RR3Vector areaVector;
                double area;
                surfaceArea(model,surfaceID,areaVector,area);
                areaBefore += area;
            }
            const uint nElementsBefore = model.getNElements();
            const uint nNodesBefore = model.getNNodes();
            const double minAngleBefore = minimumAngle(model);
            const uint nBadBefore = countBadTriangles(model,R_DEG_TO_RAD(20.0));
            const qsizetype nIntersectedBefore = model.findIntersectedElements().size();
            std::vector<RNode> nodesBefore = model.getNodes();

            QElapsedTimer timer;
            timer.start();
            RSurfaceReconstructor::Options options;
            options.angleTolerance = R_DEG_TO_RAD(angleTolerance);
            options.delaunay = delaunay;
            options.refine = refine;
            options.distanceTolerance = relativeTolerance * findDiagonal(nodesBefore);
            const double distanceTolerance = (relativeTolerance > 0.0 ? relativeTolerance : 1.0e-5) * findDiagonal(nodesBefore);
            RSurfaceReconstructor::Statistics statistics = model.reconstructSurfaces(surfaceIDs,options);
            const qint64 elapsed = timer.elapsed();

            double areaAfter = 0.0;
            for (uint surfaceID : surfaceIDs)
            {
                RR3Vector areaVector;
                double area;
                surfaceArea(model,surfaceID,areaVector,area);
                areaAfter += area;
            }
            const double volumeAfter = enclosedVolume(model);
            const qsizetype nIntersectedAfter = model.findIntersectedElements().size();

            qInfo("%s: elements %u -> %u, nodes %u -> %u (inserted %u), min. angle %.3g -> %.3g deg, below 20 deg %u -> %u, regions %u (failed %u), collapsed %u, flipped %u (delaunay %u), intersected %lld -> %lld, %lld ms",
                  qPrintable(fileName),nElementsBefore,model.getNElements(),nNodesBefore,model.getNNodes(),statistics.nInsertedNodes,
                  R_RAD_TO_DEG(minAngleBefore),R_RAD_TO_DEG(minimumAngle(model)),nBadBefore,countBadTriangles(model,R_DEG_TO_RAD(20.0)),
                  statistics.nPlanarRegions,statistics.nFailedRegions,statistics.nCollapsedEdges,statistics.nFlippedEdges,statistics.nDelaunayFlips,
                  qlonglong(nIntersectedBefore),qlonglong(nIntersectedAfter),qlonglong(elapsed));

            // Remaining nodes are original nodes, unchanged.
            std::vector<bool> found(model.getNNodes(),false);
            {
                std::map<std::array<double,3>,uint> positions;
                for (uint i=0;i<model.getNNodes();i++)
                {
                    positions[{model.getNode(i).getX(),model.getNode(i).getY(),model.getNode(i).getZ()}] = i;
                }
                for (const RNode &node : nodesBefore)
                {
                    auto iter = positions.find({node.getX(),node.getY(),node.getZ()});
                    if (iter != positions.end())
                    {
                        found[iter->second] = true;
                    }
                }
            }
            // Only inserted nodes are new (inserted node may coincide with a removed original node).
            QVERIFY(uint(std::count(found.begin(),found.end(),false)) <= statistics.nInsertedNodes);
            QVERIFY(model.getNNodes() - statistics.nInsertedNodes <= nNodesBefore);
            if (!refine)
            {
                QCOMPARE(statistics.nInsertedNodes,0u);
                QVERIFY(model.getNElements() <= nElementsBefore);
            }
            if (closedBefore)
            {
                QVERIFY(isClosedAndConsistent(model));
                // Nodes move by at most minimum edge length (10x distance tolerance) in short edge collapses.
                QVERIFY2(std::abs(volumeAfter - volumeBefore) <= 10.0 * distanceTolerance * areaBefore,
                         qPrintable(QString("Volume %1 -> %2").arg(volumeBefore,0,'g',12).arg(volumeAfter,0,'g',12)));
            }
            QVERIFY2(std::abs(areaAfter - areaBefore) <= std::max(1.0e-3,1.0e2 * relativeTolerance) * areaBefore,
                     qPrintable(QString("Area %1 -> %2").arg(areaBefore,0,'g',12).arg(areaAfter,0,'g',12)));
            QVERIFY(nIntersectedAfter <= nIntersectedBefore);
        }

        void delaunayTriangulation()
        {
            const uint n = 10;
            const double k = 2.0e-4;
            RSurfaceReconstructor::Options options;
            options.distanceTolerance = 1.0e-3;

            RModel model;
            Builder builder(model);
            buildCurvedShearedGrid(builder,n,k);
            RR3Vector areaVector;
            double areaBefore;
            surfaceArea(model,0,areaVector,areaBefore);

            // Without Delaunay option non-Delaunay edges remain.
            RModel modelPlain(model);
            options.delaunay = false;
            RSurfaceReconstructor::Statistics statistics = modelPlain.reconstructSurfaces({0},options);
            QCOMPARE(statistics.nDelaunayFlips,0u);
            QVERIFY(countDelaunayViolations(modelPlain,options.distanceTolerance) > 0);

            options.delaunay = true;
            statistics = model.reconstructSurfaces({0},options);
            QVERIFY(statistics.nDelaunayFlips > 0);
            QCOMPARE(countDelaunayViolations(model,options.distanceTolerance),0u);
            // Flips do not remove nodes.
            QCOMPARE(model.getNNodes(),modelPlain.getNNodes());

            double areaAfter;
            surfaceArea(model,0,areaVector,areaAfter);
            QVERIFY(std::abs(areaAfter - areaBefore) < 1.0e-3 * areaBefore);
        }

        void delaunayKeepsPlanarResult()
        {
            // Planar regions are already constrained Delaunay - option must not change the result.
            RModel model;
            Builder builder(model);
            buildBox(builder,6);
            RSurfaceReconstructor::Options options;
            options.delaunay = true;
            model.reconstructSurfaces({0},options);
            QCOMPARE(model.getNNodes(),8u);
            QCOMPARE(model.getNElements(),12u);
            QVERIFY(isClosedAndConsistent(model));
            QVERIFY(std::abs(enclosedVolume(model) - 1.0) < 1e-12);
        }

        void refinementImprovesQuality()
        {
            // Thin plate is merged to two needle triangles, refinement must restore quality.
            RModel model;
            Builder builder(model);
            builder.grid(RR3Vector(0,0,0),RR3Vector(10,0,0),RR3Vector(0,1,0),10,0);

            RModel modelPlain(model);
            modelPlain.reconstructSurfaces({0});
            QCOMPARE(modelPlain.getNElements(),2u);
            QVERIFY(R_RAD_TO_DEG(minimumAngle(modelPlain)) < 10.0);

            RSurfaceReconstructor::Options options;
            options.refine = true;
            options.qualityAngle = R_DEG_TO_RAD(25.0);
            RSurfaceReconstructor::Statistics statistics = model.reconstructSurfaces({0},options);

            QVERIFY(statistics.nInsertedNodes > 0);
            QCOMPARE(model.getNNodes(),4u + statistics.nInsertedNodes);
            QVERIFY2(R_RAD_TO_DEG(minimumAngle(model)) >= 25.0,qPrintable(QString::number(R_RAD_TO_DEG(minimumAngle(model)))));

            // New nodes lie on the plate.
            for (uint i=0;i<model.getNNodes();i++)
            {
                const RNode &node = model.getNode(i);
                QVERIFY(std::abs(node.getZ()) < 1e-12);
                QVERIFY(node.getX() > -1e-12 && node.getX() < 10.0 + 1e-12);
                QVERIFY(node.getY() > -1e-12 && node.getY() < 1.0 + 1e-12);
            }
            RR3Vector areaVector;
            double area;
            surfaceArea(model,0,areaVector,area);
            QVERIFY(std::abs(area - 10.0) < 1e-9);
            QVERIFY(std::abs(areaVector[2] - 10.0) < 1e-9);
        }

        void refinementOfNoisyThinFace()
        {
            // Plate with a thin perpendicular face (1 x 0.02) whose halves deviate from each other
            // slightly more than distance tolerance (relative to the plane of one half).
            RModel model;
            Builder builder(model);
            uint a = builder.node(0,0,0);
            uint b = builder.node(1,0,0);
            uint c = builder.node(1,1,0);
            uint d = builder.node(0,1,0);
            uint e = builder.node(1,0,-0.02);
            uint f = builder.node(0,1.5e-5,-0.02);
            builder.triangle(a,b,c,0);
            builder.triangle(a,c,d,0);
            builder.triangle(b,a,f,0);
            builder.triangle(b,f,e,0);

            RSurfaceReconstructor::Options options;
            options.distanceTolerance = 1.0e-5;
            options.refine = true;
            RSurfaceReconstructor::Statistics statistics = model.reconstructSurfaces({0},options);

            QVERIFY(statistics.nInsertedNodes > 0);
            QVERIFY2(R_RAD_TO_DEG(minimumAngle(model)) >= 20.0,qPrintable(QString::number(R_RAD_TO_DEG(minimumAngle(model)))));
            RR3Vector areaVector;
            double area;
            surfaceArea(model,0,areaVector,area);
            QVERIFY(std::abs(area - 1.02) < 1e-6);
        }

        void refinementRespectsMaxEdgeLength()
        {
            RModel model;
            Builder builder(model);
            buildBox(builder,6);

            RSurfaceReconstructor::Options options;
            options.refine = true;
            options.maxEdgeLength = 0.25;
            RSurfaceReconstructor::Statistics statistics = model.reconstructSurfaces({0},options);
            QVERIFY(statistics.nInsertedNodes > 0);

            double longest = 0.0;
            for (uint i=0;i<model.getNElements();i++)
            {
                const RElement &element = model.getElement(i);
                for (uint k=0;k<3;k++)
                {
                    longest = std::max(longest,model.getNode(element.getNodeId(k)).getDistance(model.getNode(element.getNodeId((k+1)%3))));
                }
            }
            QVERIFY2(longest <= 0.25 * (1.0 + 1e-9),qPrintable(QString::number(longest)));
            QVERIFY(R_RAD_TO_DEG(minimumAngle(model)) >= 20.0);
            QVERIFY(isClosedAndConsistent(model));
            QVERIFY(std::abs(enclosedVolume(model) - 1.0) < 1e-12);

            // All nodes lie on the box surface.
            for (uint i=0;i<model.getNNodes();i++)
            {
                const RNode &node = model.getNode(i);
                const double p[3] = {node.getX(),node.getY(),node.getZ()};
                bool onFace = false;
                for (uint k=0;k<3;k++)
                {
                    QVERIFY(p[k] > -1e-12 && p[k] < 1.0 + 1e-12);
                    onFace = onFace || std::abs(p[k]) < 1e-12 || std::abs(p[k] - 1.0) < 1e-12;
                }
                QVERIFY(onFace);
            }
            QCOMPARE(model.findIntersectedElements().size(),qsizetype(0));
        }

        void refinementKeepsUnselectedSurfaceConforming()
        {
            // Two plates sharing an edge, only the first one is refined - shared edge must not be split.
            RModel model;
            Builder builder(model);
            builder.grid(RR3Vector(0,0,0),RR3Vector(4,0,0),RR3Vector(0,1,0),4,0);
            builder.grid(RR3Vector(0,1,0),RR3Vector(4,0,0),RR3Vector(0,1,0),4,1);

            RSurfaceReconstructor::Options options;
            options.refine = true;
            options.maxEdgeLength = 0.3;
            RSurfaceReconstructor::Statistics statistics = model.reconstructSurfaces({0},options);
            QVERIFY(statistics.nInsertedNodes > 0);

            // Each interior edge is shared by exactly two triangles (no hanging nodes).
            std::map<std::pair<uint,uint>,uint> edges;
            for (uint i=0;i<model.getNElements();i++)
            {
                const RElement &element = model.getElement(i);
                for (uint k=0;k<3;k++)
                {
                    uint a = element.getNodeId(k);
                    uint b = element.getNodeId((k+1)%3);
                    edges[{std::min(a,b),std::max(a,b)}]++;
                }
            }
            uint nBoundaryEdges = 0;
            double boundaryLength = 0.0;
            for (const auto &edge : edges)
            {
                QVERIFY(edge.second <= 2);
                if (edge.second == 1)
                {
                    nBoundaryEdges++;
                    boundaryLength += model.getNode(edge.first.first).getDistance(model.getNode(edge.first.second));
                }
            }
            // Outer boundary of the 4 x 2 rectangle.
            QVERIFY2(std::abs(boundaryLength - 12.0) < 1e-9,qPrintable(QString::number(boundaryLength)));
            QCOMPARE(model.getSurface(1).size(),32u);
        }

        void volumeElementsAreRejected()
        {
            RModel model;
            Builder builder(model);
            uint a = builder.node(0,0,0);
            uint b = builder.node(1,0,0);
            uint c = builder.node(0,1,0);
            uint d = builder.node(0,0,1);
            builder.triangle(a,c,b,0);
            RElement tetra(R_ELEMENT_TETRA1);
            tetra.setNodeId(0,a);
            tetra.setNodeId(1,b);
            tetra.setNodeId(2,c);
            tetra.setNodeId(3,d);
            model.addElement(tetra,true,0);

            QVERIFY_THROWS_EXCEPTION(RError,model.reconstructSurfaces({0}));
        }

};

QTEST_GUILESS_MAIN(TestSurfaceReconstruction)
#include "tst_surface_reconstruction.moc"
