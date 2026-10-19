#ifndef RML_MODEL_STEP_H
#define RML_MODEL_STEP_H

#include <QString>
#include <QStringList>

#include "rml_model_raw.h"

//! Surface model exchanged through STEP (ISO 10303-21) files.
//! On import exact (B-rep) and tessellated geometry is triangulated, every
//! solid or shell becomes one body. On export every body is written as
//! planar faceted B-rep - closed bodies as solids, open bodies as shells.
class RModelStep : public RModelRaw
{

    protected:

        //! Model name.
        QString name;
        //! Model description.
        QString description;
        //! Body names.
        QStringList bodyNames;
        //! Body index for each element.
        std::vector<unsigned int> elementBodies;
        //! Maximal angle between adjacent facets on curved geometry (radians).
        double maxAngle;
        //! Maximal chord deviation relative to body size.
        double chordTolerance;

    private:

        //! Internal initialization function.
        void _init(const RModelStep *pModelStep = nullptr);

    public:

        //! Constructor.
        RModelStep();

        //! Copy constructor.
        RModelStep(const RModelStep &modelStep);

        //! Destructor.
        ~RModelStep();

        //! Assignment operator.
        RModelStep &operator =(const RModelStep &modelStep);

        //! Get model name.
        QString getName() const;

        //! Set model name.
        void setName(const QString &name);

        //! Get model description.
        QString getDescription() const;

        //! Set model description.
        void setDescription(const QString &description);

        //! Return number of bodies.
        unsigned int getNBodies() const;

        //! Return body name.
        const QString &getBodyName(unsigned int bodyID) const;

        //! Add new body and return its ID.
        unsigned int addBody(const QString &name);

        //! Return body ID of given element.
        unsigned int getElementBody(unsigned int elementID) const;

        //! Add node and return its ID.
        unsigned int addNode(const RNode &node);

        //! Add triangle to body.
        void addBodyTriangle(unsigned int bodyID,
                             unsigned int nodeID1,
                             unsigned int nodeID2,
                             unsigned int nodeID3);

        //! Set tessellation tolerances used on import.
        void setTessellationTolerances(double maxAngle, double relativeChordTolerance);

        //! Remove all nodes, elements and bodies.
        void clear();

        //! Read model from STEP file.
        //! Geometry is converted to meters.
        void read(const QString &fileName,
                  double         tolerance = 0.0);

        //! Write model to STEP file.
        void write(const QString &fileName) const;

};

#endif // RML_MODEL_STEP_H
