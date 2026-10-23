#include <QtTest>
#include <QTemporaryDir>

#include <array>
#include <vector>

#include "rml_model.h"
#include "rml_model_stl.h"

class TestSurfaceNormals : public QObject
{
    Q_OBJECT

    private:

        typedef std::array<double,3> Point;
        typedef std::array<Point,3> Triangle;

        QTemporaryDir tempDir;

        //! Append two triangles of a quad (corners in given order).
        static void addQuad(std::vector<Triangle> &triangles, const Point &a, const Point &b, const Point &c, const Point &d)
        {
            triangles.push_back({a,b,c});
            triangles.push_back({a,c,d});
        }

        //! Append all six faces of an axis aligned box.
        static void addBox(std::vector<Triangle> &triangles, const Point &l, const Point &u, bool skipBottom = false, bool skipTop = false)
        {
            if (!skipBottom)
            {
                addQuad(triangles,{l[0],l[1],l[2]},{u[0],l[1],l[2]},{u[0],u[1],l[2]},{l[0],u[1],l[2]});
            }
            if (!skipTop)
            {
                addQuad(triangles,{l[0],l[1],u[2]},{u[0],l[1],u[2]},{u[0],u[1],u[2]},{l[0],u[1],u[2]});
            }
            addQuad(triangles,{l[0],l[1],l[2]},{u[0],l[1],l[2]},{u[0],l[1],u[2]},{l[0],l[1],u[2]});
            addQuad(triangles,{l[0],u[1],l[2]},{u[0],u[1],l[2]},{u[0],u[1],u[2]},{l[0],u[1],u[2]});
            addQuad(triangles,{l[0],l[1],l[2]},{l[0],u[1],l[2]},{l[0],u[1],u[2]},{l[0],l[1],u[2]});
            addQuad(triangles,{u[0],l[1],l[2]},{u[0],u[1],l[2]},{u[0],u[1],u[2]},{u[0],l[1],u[2]});
        }

        //! Write ASCII STL file, orientation of every other triangle is reversed.
        QString writeStl(const QString &name, const std::vector<Triangle> &triangles)
        {
            QString fileName = this->tempDir.filePath(name);
            QFile file(fileName);
            if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
            {
                return QString();
            }
            QTextStream out(&file);
            out << "solid " << name << "\n";
            for (size_t i=0;i<triangles.size();i++)
            {
                Triangle t = triangles[i];
                if (i % 2 == 1)
                {
                    std::swap(t[1],t[2]);
                }
                out << "  facet normal 0 0 0\n    outer loop\n";
                for (const Point &p : t)
                {
                    out << "      vertex " << p[0] << " " << p[1] << " " << p[2] << "\n";
                }
                out << "    endloop\n  endfacet\n";
            }
            out << "endsolid " << name << "\n";
            return fileName;
        }

        static bool isInsideBox(const RR3Vector &p, const Point &l, const Point &u)
        {
            return (p[0] > l[0] && p[0] < u[0] && p[1] > l[1] && p[1] < u[1] && p[2] > l[2] && p[2] < u[2]);
        }

        //! Load STL, synchronize normals and return element centers moved slightly along their normals.
        static void loadAndSync(const QString &fileName, RModel &model, std::vector<RR3Vector> &centers, std::vector<RR3Vector> &normals)
        {
            RModelStl modelStl;
            modelStl.read(fileName);
            model = RModel(modelStl);
            model.syncSurfaceNormals();

            for (uint i=0;i<model.getNElements();i++)
            {
                const RElement &rElement = model.getElement(i);
                RR3Vector c(0.0,0.0,0.0);
                for (uint k=0;k<rElement.size();k++)
                {
                    c[0] += model.getNode(rElement.getNodeId(k)).getX() / double(rElement.size());
                    c[1] += model.getNode(rElement.getNodeId(k)).getY() / double(rElement.size());
                    c[2] += model.getNode(rElement.getNodeId(k)).getZ() / double(rElement.size());
                }
                double nx, ny, nz;
                rElement.findNormal(model.getNodes(),nx,ny,nz);
                centers.push_back(c);
                normals.push_back(RR3Vector(nx,ny,nz));
            }
        }

    private slots:

        void testSingleBodyPointsOutwards()
        {
            const Point l = {0.0,0.0,0.0};
            const Point u = {2.0,3.0,1.0};
            std::vector<Triangle> triangles;
            addBox(triangles,l,u);

            RModel model;
            std::vector<RR3Vector> centers, normals;
            loadAndSync(this->writeStl("box.stl",triangles),model,centers,normals);
            QCOMPARE(model.getNElements(),uint(12));

            for (uint i=0;i<centers.size();i++)
            {
                RR3Vector outside(centers[i][0] + 1.0e-3 * normals[i][0],centers[i][1] + 1.0e-3 * normals[i][1],centers[i][2] + 1.0e-3 * normals[i][2]);
                QVERIFY2(!isInsideBox(outside,l,u),qPrintable(QString("Element %1 points inwards").arg(i)));
            }
        }

        //! Box standing on a plate, both share one internal face.
        void testMultiBodyPointsOutwards()
        {
            const Point pl = {0.0,0.0,0.0};
            const Point pu = {4.0,4.0,1.0};
            const Point bl = {1.0,1.0,1.0};
            const Point bu = {3.0,3.0,2.0};

            std::vector<Triangle> triangles;
            // Plate without top.
            addBox(triangles,pl,pu,false,true);
            // Plate top around the box footprint.
            const Point o[4] = {{0.0,0.0,1.0},{4.0,0.0,1.0},{4.0,4.0,1.0},{0.0,4.0,1.0}};
            const Point in[4] = {{1.0,1.0,1.0},{3.0,1.0,1.0},{3.0,3.0,1.0},{1.0,3.0,1.0}};
            for (uint i=0;i<4;i++)
            {
                addQuad(triangles,o[i],o[(i+1)%4],in[(i+1)%4],in[i]);
            }
            // Box including its bottom, which is the internal face.
            addBox(triangles,bl,bu);

            RModel model;
            std::vector<RR3Vector> centers, normals;
            loadAndSync(this->writeStl("multi_body.stl",triangles),model,centers,normals);
            QCOMPARE(model.getNElements(),uint(30));

            for (uint i=0;i<centers.size();i++)
            {
                if (std::fabs(centers[i][2] - 1.0) < 1.0e-9 && isInsideBox(RR3Vector(centers[i][0],centers[i][1],1.5),bl,bu))
                {
                    // Internal face has no outer side.
                    continue;
                }
                RR3Vector outside(centers[i][0] + 1.0e-3 * normals[i][0],centers[i][1] + 1.0e-3 * normals[i][1],centers[i][2] + 1.0e-3 * normals[i][2]);
                QVERIFY2(!isInsideBox(outside,pl,pu) && !isInsideBox(outside,bl,bu),qPrintable(QString("Element %1 points inwards").arg(i)));
            }
        }

};

QTEST_GUILESS_MAIN(TestSurfaceNormals)
#include "tst_surface_normals.moc"
