#ifndef RML_STEP_TESSELLATOR_H
#define RML_STEP_TESSELLATOR_H

#include <array>
#include <map>
#include <memory>
#include <vector>

#include "rml_step_file.h"
#include "rml_step_geometry.h"

//! Triangle soup produced by tessellation.
struct RStepMesh
{
    std::vector<RStepVector> nodes;
    std::vector<std::array<unsigned int,3>> triangles;

    //! Append other mesh transformed by given transformation.
    void append(const RStepMesh &mesh, const RStepTransform &transform, bool flip = false);
};

//! Tessellation of STEP topology (B-rep and tessellated geometry) into triangles.
class RStepTessellator
{

    public:

        enum ItemType
        {
            //! Not a shape item.
            ItemNone = 0,
            //! Boundary representation (exact geometry).
            ItemBRep,
            //! Tessellated (triangulated) geometry.
            ItemTessellated
        };

    protected:

        //! Loop point in parameter space.
        struct UVPoint
        {
            double uv[2];
            RStepVector p;
            bool hasPoint;
            bool singular;
        };

        //! Loop in parameter space.
        struct UVLoop
        {
            std::vector<UVPoint> points;
            double net[2];
            bool outer;
        };

    protected:

        //! STEP file.
        const RStepFile &file;
        //! Plane angle unit scale (to radians).
        double angleScale;
        //! Maximal angle between adjacent segments / facets.
        double maxAngle;
        //! Maximal chord deviation.
        double chordTolerance;
        //! Sampled edges.
        std::map<unsigned int,std::vector<RStepVector>> edgeCache;
        //! Built surfaces.
        std::map<unsigned int,std::shared_ptr<RStepSurface>> surfaceCache;
        //! IDs of faces which failed to tessellate.
        std::vector<unsigned int> failedFaces;
        //! Number of edges with unsupported curve geometry.
        unsigned int nUnsupportedEdges;

    public:

        //! Constructor.
        RStepTessellator(const RStepFile &file, double angleScale);

        //! Set tolerances.
        void setTolerances(double maxAngle, double chordTolerance);

        //! Return item type of given entity.
        static ItemType getItemType(const RStepEntity &entity);

        //! Return entity name.
        static QString getEntityName(const RStepEntity &entity);

        //! Estimate size (bounding box diagonal) of item from its vertices.
        //! Return 0 if size could not be determined.
        double estimateSize(unsigned int itemId) const;

        //! Tessellate shape item.
        //! Return false if item is not a shape item.
        bool tessellateItem(unsigned int itemId, RStepMesh &mesh);

        //! Return number of faces which failed to tessellate.
        unsigned int getNFailedFaces() const;

        //! Return IDs of faces which failed to tessellate.
        const std::vector<unsigned int> &getFailedFaces() const;

        //! Return number of edges with unsupported curve geometry.
        unsigned int getNUnsupportedEdges() const;

    protected:

        //! Tessellate set of faces (shell).
        void tessellateShell(unsigned int shellId, bool orientation, RStepMesh &mesh);

        //! Tessellate face.
        //! Return false on failure.
        bool tessellateFace(unsigned int faceId, bool orientation, RStepMesh &mesh);

        //! Tessellate triangulated face or surface set.
        bool tessellateTriangulated(const RStepEntity &entity, RStepMesh &mesh) const;

        //! Return sampled edge points from start to end vertex.
        const std::vector<RStepVector> &sampleEdge(unsigned int edgeId);

        //! Return loop points.
        bool readLoop(unsigned int loopId, std::vector<RStepVector> &points);

        //! Return surface for given entity (cached).
        std::shared_ptr<RStepSurface> getSurface(unsigned int surfaceId);

        //! Triangulate face loops on surface.
        bool triangulateFace(const RStepSurface &surface,
                             const std::vector<std::vector<RStepVector>> &loops,
                             const std::vector<bool> &outerFlags,
                             bool sameSense,
                             RStepMesh &mesh) const;

        //! Project and unwrap loop into parameter space.
        bool buildUVLoop(const RStepSurface &surface,
                         const std::vector<RStepVector> &points,
                         bool outer,
                         bool sameSense,
                         double size,
                         UVLoop &loop) const;

};

#endif // RML_STEP_TESSELLATOR_H
