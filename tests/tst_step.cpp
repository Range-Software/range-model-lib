#include <QtTest>
#include <QTemporaryDir>

#include <cmath>
#include <map>

#include "rml_model.h"
#include "rml_model_step.h"
#include "rml_step_file.h"

class TestStep : public QObject
{
    Q_OBJECT

    private:

        QTemporaryDir tempDir;

        //! Write STEP file with given DATA section content (millimeter units in #1..#3).
        QString writeStep(const QString &name, const QString &data)
        {
            QString fileName = this->tempDir.filePath(name);
            QFile file(fileName);
            if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
            {
                return QString();
            }
            QTextStream out(&file);
            out << "ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('test'),'2;1');\n"
                << "FILE_NAME('" << name << "','',(''),(''),'','','');\n"
                << "FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\nENDSEC;\nDATA;\n"
                << "#1=(LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.));\n"
                << "#2=(NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.));\n"
                << "#3=(GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNIT_ASSIGNED_CONTEXT((#1,#2)) REPRESENTATION_CONTEXT('',''));\n"
                << data
                << "ENDSEC;\nEND-ISO-10303-21;\n";
            return fileName;
        }

        //! Common placements: #10 origin, #11 (0,0,20), #12 z, #13 x, #14 bottom frame, #15 top frame.
        static QString frames()
        {
            return "#10=CARTESIAN_POINT('',(0.,0.,0.));\n"
                   "#11=CARTESIAN_POINT('',(0.,0.,20.));\n"
                   "#12=DIRECTION('',(0.,0.,1.));\n"
                   "#13=DIRECTION('',(1.,0.,0.));\n"
                   "#14=AXIS2_PLACEMENT_3D('',#10,#12,#13);\n"
                   "#15=AXIS2_PLACEMENT_3D('',#11,#12,#13);\n"
                   "#16=VECTOR('',#12,1.);\n";
        }

        //! Circle edges (#first: bottom, #first+1: top) and seam (#first+2) of radius r with given id base.
        static QString circles(int base, double r)
        {
            QString s;
            s += QString("#%1=CARTESIAN_POINT('',(%2,0.,0.));\n").arg(base).arg(r,0,'f',1);
            s += QString("#%1=CARTESIAN_POINT('',(%2,0.,20.));\n").arg(base+1).arg(r,0,'f',1);
            s += QString("#%1=VERTEX_POINT('',#%2);\n").arg(base+2).arg(base);
            s += QString("#%1=VERTEX_POINT('',#%2);\n").arg(base+3).arg(base+1);
            s += QString("#%1=CIRCLE('',#14,%2);\n").arg(base+4).arg(r,0,'f',1);
            s += QString("#%1=CIRCLE('',#15,%2);\n").arg(base+5).arg(r,0,'f',1);
            s += QString("#%1=LINE('',#%2,#16);\n").arg(base+6).arg(base);
            s += QString("#%1=EDGE_CURVE('',#%2,#%2,#%3,.T.);\n").arg(base+7).arg(base+2).arg(base+4);
            s += QString("#%1=EDGE_CURVE('',#%2,#%2,#%3,.T.);\n").arg(base+8).arg(base+3).arg(base+5);
            s += QString("#%1=EDGE_CURVE('',#%2,#%3,#%4,.T.);\n").arg(base+9).arg(base+2).arg(base+3).arg(base+6);
            return s;
        }

        //! Check that all bodies are closed and consistently oriented, return total signed volume.
        static double checkClosed(const RModelStep &model, bool &closed)
        {
            std::map<std::pair<unsigned int,unsigned int>,int> edges;
            double volume = 0.0;
            for (unsigned int i=0;i<model.getNElements();i++)
            {
                const RElement &element = model.getElement(i);
                unsigned int ids[3] = {element.getNodeId(0),element.getNodeId(1),element.getNodeId(2)};
                for (int k=0;k<3;k++)
                {
                    edges[{ids[k],ids[(k+1)%3]}]++;
                }
                const RNode &a = model.getNode(ids[0]);
                const RNode &b = model.getNode(ids[1]);
                const RNode &c = model.getNode(ids[2]);
                volume += (a.getX() * (b.getY() * c.getZ() - b.getZ() * c.getY())
                         - a.getY() * (b.getX() * c.getZ() - b.getZ() * c.getX())
                         + a.getZ() * (b.getX() * c.getY() - b.getY() * c.getX())) / 6.0;
            }
            closed = true;
            for (const auto &edge : edges)
            {
                auto reverse = edges.find({edge.first.second,edge.first.first});
                if (edge.second != 1 || reverse == edges.end() || reverse->second != 1)
                {
                    closed = false;
                }
            }
            return volume;
        }

    private slots:

        void parser()
        {
            RStepFile file;
            file.parse("ISO-10303-21;\nHEADER;\nFILE_NAME('a.stp',/* comment */'',(''),(''),'','','');\nENDSEC;\nDATA;\n"
                       "#1=PRODUCT('It''s','\\X2\\00E9\\X0\\t\\\\e','',(#2));\n"
                       "#2 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );\n"
                       "#3=MEASURE_WITH_UNIT(LENGTH_MEASURE(25.4),#2);\n"
                       "#4=CARTESIAN_POINT('',(-1.5E-3,2.,3));\n"
                       "ENDSEC;\nEND-ISO-10303-21;\n");

            const RStepEntity *pProduct = file.find(1);
            QVERIFY(pProduct);
            QCOMPARE(pProduct->keyword(),QString("PRODUCT"));
            QCOMPARE(pProduct->records[0].param(0).text,QString("It's"));
            QCOMPARE(pProduct->records[0].param(1).text,QString::fromUtf8("\xC3\xA9t\\e"));
            QCOMPARE(pProduct->records[0].param(3).toIdList(),std::vector<unsigned int>({2}));

            const RStepEntity *pUnit = file.find(2);
            QVERIFY(pUnit && pUnit->isComplex());
            QVERIFY(pUnit->has("SI_UNIT"));
            QCOMPARE(pUnit->find("SI_UNIT")->param(0).text,QString("MILLI"));
            QVERIFY(pUnit->find("NAMED_UNIT")->param(0).type == RStepValue::Derived);

            QCOMPARE(file.find(3)->records[0].param(0).toDouble(),25.4);
            std::vector<double> xyz = file.find(4)->records[0].param(1).toDoubleList();
            QCOMPARE(xyz,std::vector<double>({-1.5e-3,2.0,3.0}));

            QCOMPARE(RStepFile::formatReal(1.0),QString("1."));
            QCOMPARE(RStepFile::formatReal(1e-5),QString("1.E-05"));
            QCOMPARE(RStepFile::formatString("It's"),QString("'It''s'"));
        }

        void parserRejectsInvalid()
        {
            RStepFile file;
            QVERIFY_THROWS_EXCEPTION(RError,file.parse("solid stl\nendsolid\n"));
            QVERIFY_THROWS_EXCEPTION(RError,file.parse("ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n#1=POINT((1.,2.);\nENDSEC;\nEND-ISO-10303-21;\n"));
        }

        void cylinderWithSeam()
        {
            QString data = frames() + circles(20,10.0) +
                "#50=ORIENTED_EDGE('',*,*,#27,.F.);\n#51=EDGE_LOOP('',(#50));\n#52=FACE_OUTER_BOUND('',#51,.T.);\n"
                "#53=PLANE('',#14);\n#54=ADVANCED_FACE('',(#52),#53,.F.);\n"
                "#60=ORIENTED_EDGE('',*,*,#28,.T.);\n#61=EDGE_LOOP('',(#60));\n#62=FACE_OUTER_BOUND('',#61,.T.);\n"
                "#63=PLANE('',#15);\n#64=ADVANCED_FACE('',(#62),#63,.T.);\n"
                "#70=ORIENTED_EDGE('',*,*,#27,.T.);\n#71=ORIENTED_EDGE('',*,*,#29,.T.);\n"
                "#72=ORIENTED_EDGE('',*,*,#28,.F.);\n#73=ORIENTED_EDGE('',*,*,#29,.F.);\n"
                "#74=EDGE_LOOP('',(#70,#71,#72,#73));\n#75=FACE_OUTER_BOUND('',#74,.T.);\n"
                "#76=CYLINDRICAL_SURFACE('',#14,10.);\n#77=ADVANCED_FACE('',(#75),#76,.T.);\n"
                "#80=CLOSED_SHELL('',(#54,#64,#77));\n#81=MANIFOLD_SOLID_BREP('Cylinder',#80);\n"
                "#82=ADVANCED_BREP_SHAPE_REPRESENTATION('',(#14,#81),#3);\n";

            RModelStep model;
            model.read(this->writeStep("cylinder.step",data));

            QCOMPARE(model.getNBodies(),1u);
            QCOMPARE(model.getBodyName(0),QString("Cylinder"));
            bool closed = false;
            double volume = checkClosed(model,closed);
            QVERIFY(closed);
            double exact = M_PI * 0.01 * 0.01 * 0.02;
            QVERIFY2(std::abs(volume - exact) < 0.02 * exact,qPrintable(QString("volume %1 expected %2").arg(volume).arg(exact)));
        }

        void cylinderWithoutSeam()
        {
            // Cylindrical face bounded by two circles only.
            QString data = frames() + circles(20,10.0) +
                "#50=ORIENTED_EDGE('',*,*,#27,.F.);\n#51=EDGE_LOOP('',(#50));\n#52=FACE_OUTER_BOUND('',#51,.T.);\n"
                "#53=PLANE('',#14);\n#54=ADVANCED_FACE('',(#52),#53,.F.);\n"
                "#60=ORIENTED_EDGE('',*,*,#28,.T.);\n#61=EDGE_LOOP('',(#60));\n#62=FACE_OUTER_BOUND('',#61,.T.);\n"
                "#63=PLANE('',#15);\n#64=ADVANCED_FACE('',(#62),#63,.T.);\n"
                "#70=ORIENTED_EDGE('',*,*,#27,.T.);\n#71=EDGE_LOOP('',(#70));\n#72=FACE_OUTER_BOUND('',#71,.T.);\n"
                "#73=ORIENTED_EDGE('',*,*,#28,.F.);\n#74=EDGE_LOOP('',(#73));\n#75=FACE_BOUND('',#74,.T.);\n"
                "#76=CYLINDRICAL_SURFACE('',#14,10.);\n#77=ADVANCED_FACE('',(#72,#75),#76,.T.);\n"
                "#80=CLOSED_SHELL('',(#54,#64,#77));\n#81=MANIFOLD_SOLID_BREP('',#80);\n"
                "#82=ADVANCED_BREP_SHAPE_REPRESENTATION('',(#14,#81),#3);\n";

            RModelStep model;
            model.read(this->writeStep("cylinder_noseam.step",data));

            bool closed = false;
            double volume = checkClosed(model,closed);
            QVERIFY(closed);
            double exact = M_PI * 0.01 * 0.01 * 0.02;
            QVERIFY2(std::abs(volume - exact) < 0.02 * exact,qPrintable(QString("volume %1 expected %2").arg(volume).arg(exact)));
        }

        void tube()
        {
            // Outer radius 10, inner radius 5 - annular planes with holes, reversed inner cylinder.
            QString data = frames() + circles(20,10.0) + circles(30,5.0) +
                "#50=ORIENTED_EDGE('',*,*,#27,.F.);\n#51=EDGE_LOOP('',(#50));\n#52=FACE_OUTER_BOUND('',#51,.T.);\n"
                "#55=ORIENTED_EDGE('',*,*,#37,.T.);\n#56=EDGE_LOOP('',(#55));\n#57=FACE_BOUND('',#56,.T.);\n"
                "#53=PLANE('',#14);\n#54=ADVANCED_FACE('',(#52,#57),#53,.F.);\n"
                "#60=ORIENTED_EDGE('',*,*,#28,.T.);\n#61=EDGE_LOOP('',(#60));\n#62=FACE_OUTER_BOUND('',#61,.T.);\n"
                "#65=ORIENTED_EDGE('',*,*,#38,.F.);\n#66=EDGE_LOOP('',(#65));\n#67=FACE_BOUND('',#66,.T.);\n"
                "#63=PLANE('',#15);\n#64=ADVANCED_FACE('',(#62,#67),#63,.T.);\n"
                "#70=ORIENTED_EDGE('',*,*,#27,.T.);\n#71=ORIENTED_EDGE('',*,*,#29,.T.);\n"
                "#72=ORIENTED_EDGE('',*,*,#28,.F.);\n#73=ORIENTED_EDGE('',*,*,#29,.F.);\n"
                "#74=EDGE_LOOP('',(#70,#71,#72,#73));\n#75=FACE_OUTER_BOUND('',#74,.T.);\n"
                "#76=CYLINDRICAL_SURFACE('',#14,10.);\n#77=ADVANCED_FACE('',(#75),#76,.T.);\n"
                "#90=ORIENTED_EDGE('',*,*,#37,.F.);\n#91=ORIENTED_EDGE('',*,*,#39,.T.);\n"
                "#92=ORIENTED_EDGE('',*,*,#38,.T.);\n#93=ORIENTED_EDGE('',*,*,#39,.F.);\n"
                "#94=EDGE_LOOP('',(#90,#91,#92,#93));\n#95=FACE_OUTER_BOUND('',#94,.T.);\n"
                "#96=CYLINDRICAL_SURFACE('',#14,5.);\n#97=ADVANCED_FACE('',(#95),#96,.F.);\n"
                "#80=CLOSED_SHELL('',(#54,#64,#77,#97));\n#81=MANIFOLD_SOLID_BREP('Tube',#80);\n"
                "#82=ADVANCED_BREP_SHAPE_REPRESENTATION('',(#14,#81),#3);\n";

            RModelStep model;
            model.read(this->writeStep("tube.step",data));

            bool closed = false;
            double volume = checkClosed(model,closed);
            QVERIFY(closed);
            double exact = M_PI * (0.01 * 0.01 - 0.005 * 0.005) * 0.02;
            QVERIFY2(std::abs(volume - exact) < 0.02 * exact,qPrintable(QString("volume %1 expected %2").arg(volume).arg(exact)));
        }

        void sphere()
        {
            // Single spherical face bounded by seam meridian traversed in both directions.
            QString data = frames() +
                "#20=CARTESIAN_POINT('',(0.,0.,-10.));\n#21=CARTESIAN_POINT('',(0.,0.,10.));\n"
                "#22=VERTEX_POINT('',#20);\n#23=VERTEX_POINT('',#21);\n"
                "#24=DIRECTION('',(0.,-1.,0.));\n#25=AXIS2_PLACEMENT_3D('',#10,#24,#13);\n"
                "#26=CIRCLE('',#25,10.);\n#27=EDGE_CURVE('',#22,#23,#26,.T.);\n"
                "#30=ORIENTED_EDGE('',*,*,#27,.T.);\n#31=ORIENTED_EDGE('',*,*,#27,.F.);\n"
                "#32=EDGE_LOOP('',(#30,#31));\n#33=FACE_OUTER_BOUND('',#32,.T.);\n"
                "#34=SPHERICAL_SURFACE('',#14,10.);\n#35=ADVANCED_FACE('',(#33),#34,.T.);\n"
                "#80=CLOSED_SHELL('',(#35));\n#81=MANIFOLD_SOLID_BREP('Sphere',#80);\n"
                "#82=ADVANCED_BREP_SHAPE_REPRESENTATION('',(#14,#81),#3);\n";

            RModelStep model;
            model.read(this->writeStep("sphere.step",data));

            bool closed = false;
            double volume = checkClosed(model,closed);
            QVERIFY(closed);
            double exact = 4.0 / 3.0 * M_PI * std::pow(0.01,3);
            QVERIFY2(std::abs(volume - exact) < 0.03 * exact,qPrintable(QString("volume %1 expected %2").arg(volume).arg(exact)));
        }

        void sphereCut()
        {
            // Sphere cut by plane x = 6 - spherical face lies outside of its only loop, which crosses the seam.
            QString data = frames() +
                "#20=CARTESIAN_POINT('',(6.,0.,0.));\n#21=DIRECTION('',(1.,0.,0.));\n#22=DIRECTION('',(0.,0.,1.));\n"
                "#23=AXIS2_PLACEMENT_3D('',#20,#21,#22);\n#24=CIRCLE('',#23,8.);\n"
                "#25=CARTESIAN_POINT('',(6.,0.,8.));\n#26=VERTEX_POINT('',#25);\n#27=EDGE_CURVE('',#26,#26,#24,.T.);\n"
                "#30=ORIENTED_EDGE('',*,*,#27,.F.);\n#31=EDGE_LOOP('',(#30));\n#32=FACE_OUTER_BOUND('',#31,.T.);\n"
                "#33=SPHERICAL_SURFACE('',#14,10.);\n#34=ADVANCED_FACE('',(#32),#33,.T.);\n"
                "#40=ORIENTED_EDGE('',*,*,#27,.T.);\n#41=EDGE_LOOP('',(#40));\n#42=FACE_OUTER_BOUND('',#41,.T.);\n"
                "#43=PLANE('',#23);\n#44=ADVANCED_FACE('',(#42),#43,.T.);\n"
                "#80=CLOSED_SHELL('',(#34,#44));\n#81=MANIFOLD_SOLID_BREP('Cut sphere',#80);\n"
                "#82=ADVANCED_BREP_SHAPE_REPRESENTATION('',(#14,#81),#3);\n";

            RModelStep model;
            model.read(this->writeStep("sphere_cut.step",data));

            bool closed = false;
            double volume = checkClosed(model,closed);
            QVERIFY(closed);
            double r = 0.01, h = 0.004;
            double exact = 4.0 / 3.0 * M_PI * r * r * r - M_PI * h * h * (3.0 * r - h) / 3.0;
            QVERIFY2(std::abs(volume - exact) < 0.03 * exact,qPrintable(QString("volume %1 expected %2").arg(volume).arg(exact)));
        }

        void tessellatedAssembly()
        {
            // Tessellated tetrahedron placed in assembly 100 mm along x.
            QString data =
                "#10=COORDINATES_LIST('',4,((0.,0.,0.),(10.,0.,0.),(0.,10.,0.),(0.,0.,10.)));\n"
                "#11=TRIANGULATED_FACE('',#10,0,(),$,(),((1,3,2),(1,2,4),(2,3,4),(1,4,3)));\n"
                "#12=TESSELLATED_SHELL('Tetra',(#11),$);\n"
                "#13=TESSELLATED_SHAPE_REPRESENTATION('part',(#12),#3);\n"
                "#20=CARTESIAN_POINT('',(0.,0.,0.));\n#21=DIRECTION('',(0.,0.,1.));\n#22=DIRECTION('',(1.,0.,0.));\n"
                "#23=AXIS2_PLACEMENT_3D('',#20,#21,#22);\n#24=CARTESIAN_POINT('',(100.,0.,0.));\n"
                "#25=AXIS2_PLACEMENT_3D('',#24,#21,#22);\n#26=SHAPE_REPRESENTATION('assembly',(#23,#25),#3);\n"
                "#27=ITEM_DEFINED_TRANSFORMATION('','',#23,#25);\n"
                "#28=(REPRESENTATION_RELATIONSHIP('','',#13,#26) REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#27) SHAPE_REPRESENTATION_RELATIONSHIP());\n";

            RModelStep model;
            model.read(this->writeStep("assembly.step",data));

            QCOMPARE(model.getNBodies(),1u);
            QCOMPARE(model.getBodyName(0),QString("Tetra"));
            QCOMPARE(model.getNElements(),4u);
            QCOMPARE(model.getNNodes(),4u);
            double xMin = 1e10;
            for (unsigned int i=0;i<model.getNNodes();i++)
            {
                xMin = std::min(xMin,model.getNode(i).getX());
            }
            QVERIFY(std::abs(xMin - 0.1) < 1e-12);
            bool closed = false;
            double volume = checkClosed(model,closed);
            QVERIFY(closed);
            QVERIFY(std::abs(volume - 1e-6 / 6.0) < 1e-15);
        }

        void writeAndReadBack()
        {
            // Unit cube (closed body) and a single triangle (open body).
            RModelStep model;
            model.setName("Cube");
            const double xyz[8][3] = {{0,0,0},{1,0,0},{1,1,0},{0,1,0},{0,0,1},{1,0,1},{1,1,1},{0,1,1}};
            for (const auto &p : xyz)
            {
                model.addNode(RNode(p[0],p[1],p[2]));
            }
            const unsigned int faces[12][3] = {{0,2,1},{0,3,2},{4,5,6},{4,6,7},{0,1,5},{0,5,4},
                                               {1,2,6},{1,6,5},{2,3,7},{2,7,6},{3,0,4},{3,4,7}};
            unsigned int cube = model.addBody("Box");
            for (const auto &f : faces)
            {
                model.addBodyTriangle(cube,f[0],f[1],f[2]);
            }
            unsigned int n1 = model.addNode(RNode(2,0,0));
            unsigned int n2 = model.addNode(RNode(3,0,0));
            unsigned int n3 = model.addNode(RNode(2,1,0));
            unsigned int sheet = model.addBody("Sheet");
            model.addBodyTriangle(sheet,n1,n2,n3);

            QString fileName = this->tempDir.filePath("cube.step");
            model.write(fileName);

            RStepFile file;
            file.read(fileName);
            int nSolids = 0, nShellModels = 0;
            for (const auto &iter : file.getEntities())
            {
                nSolids += (iter.second.keyword() == "MANIFOLD_SOLID_BREP") ? 1 : 0;
                nShellModels += (iter.second.keyword() == "SHELL_BASED_SURFACE_MODEL") ? 1 : 0;
            }
            QCOMPARE(nSolids,1);
            QCOMPARE(nShellModels,1);

            RModelStep readModel;
            readModel.read(fileName);
            QCOMPARE(readModel.getNBodies(),2u);
            QCOMPARE(readModel.getBodyName(0),QString("Box"));
            QCOMPARE(readModel.getBodyName(1),QString("Sheet"));
            QCOMPARE(readModel.getNElements(),13u);
            QCOMPARE(readModel.getNNodes(),11u);

            // Converted model has one surface per body.
            RModel rModel(readModel);
            QCOMPARE(rModel.getNSurfaces(),2u);
            QCOMPARE(rModel.getSurface(0).getName(),QString("Box"));
            QCOMPARE(rModel.getSurface(0).size(),12u);

            RModelStep exported;
            rModel.exportTo(exported);
            QCOMPARE(exported.getNBodies(),2u);
            QCOMPARE(exported.getNElements(),13u);

            // Cube volume survives the round trip.
            double volume = 0.0;
            for (unsigned int i=0;i<readModel.getNElements();i++)
            {
                if (readModel.getElementBody(i) != 0)
                {
                    continue;
                }
                const RElement &element = readModel.getElement(i);
                const RNode &a = readModel.getNode(element.getNodeId(0));
                const RNode &b = readModel.getNode(element.getNodeId(1));
                const RNode &c = readModel.getNode(element.getNodeId(2));
                volume += (a.getX() * (b.getY() * c.getZ() - b.getZ() * c.getY())
                         - a.getY() * (b.getX() * c.getZ() - b.getZ() * c.getX())
                         + a.getZ() * (b.getX() * c.getY() - b.getY() * c.getX())) / 6.0;
            }
            QVERIFY(std::abs(volume - 1.0) < 1e-12);
        }

        void missingGeometryFails()
        {
            RModelStep model;
            QVERIFY_THROWS_EXCEPTION(RError,model.read(this->writeStep("empty.step","#10=CARTESIAN_POINT('',(0.,0.,0.));\n")));
        }

        void samples()
        {
            // Optional check of real CAD files: RANGE_STEP_SAMPLES=<directory with *.step / *.stp files>.
            QString directory = qEnvironmentVariable("RANGE_STEP_SAMPLES");
            if (directory.isEmpty())
            {
                QSKIP("RANGE_STEP_SAMPLES is not set.");
            }
            QDir dir(directory);
            const QStringList fileNames = dir.entryList({"*.step","*.stp","*.STEP","*.STP"},QDir::Files);
            QVERIFY(!fileNames.isEmpty());
            for (const QString &fileName : fileNames)
            {
                RModelStep model;
                QElapsedTimer timer;
                timer.start();
                model.read(dir.filePath(fileName));
                qint64 elapsed = timer.elapsed();

                // Per body closure.
                unsigned int nClosed = 0;
                unsigned int nWatertight = 0;
                for (unsigned int b=0;b<model.getNBodies();b++)
                {
                    std::map<std::pair<unsigned int,unsigned int>,int> edges;
                    for (unsigned int i=0;i<model.getNElements();i++)
                    {
                        if (model.getElementBody(i) != b)
                        {
                            continue;
                        }
                        const RElement &element = model.getElement(i);
                        for (int k=0;k<3;k++)
                        {
                            edges[{element.getNodeId(k),element.getNodeId((k+1)%3)}]++;
                        }
                    }
                    bool closed = true;
                    for (const auto &edge : edges)
                    {
                        auto reverse = edges.find({edge.first.second,edge.first.first});
                        if (edge.second != 1 || reverse == edges.end() || reverse->second != 1)
                        {
                            closed = false;
                            break;
                        }
                    }
                    nClosed += closed ? 1 : 0;
                    std::map<std::pair<unsigned int,unsigned int>,int> undirected;
                    for (const auto &edge : edges)
                    {
                        undirected[{std::min(edge.first.first,edge.first.second),std::max(edge.first.first,edge.first.second)}] += edge.second;
                    }
                    bool watertight = true;
                    for (const auto &edge : undirected)
                    {
                        watertight = watertight && (edge.second == 2);
                    }
                    nWatertight += watertight ? 1 : 0;
                    if (!watertight)
                    {
                        int nOpen = 0;
                        for (const auto &edge : undirected)
                        {
                            nOpen += (edge.second != 2) ? 1 : 0;
                        }
                        qInfo("  open body %u \"%s\": %d open edges",b,qPrintable(model.getBodyName(b)),nOpen);
                    }
                }
                qInfo("%s: %u bodies (%u watertight, %u consistently oriented), %u triangles, %lld ms",qPrintable(fileName),model.getNBodies(),nWatertight,nClosed,model.getNElements(),elapsed);
                QVERIFY(model.getNElements() > 0);
            }
        }

};

QTEST_GUILESS_MAIN(TestStep)

#include "tst_step.moc"
