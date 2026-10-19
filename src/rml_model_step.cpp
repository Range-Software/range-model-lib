#include <QDateTime>
#include <QFile>
#include <QFileInfo>
#include <QTextStream>

#include <cmath>
#include <functional>
#include <map>
#include <set>
#include <unordered_map>

#include <rbl_error.h>
#include <rbl_logger.h>
#include <rbl_progress.h>
#include <rbl_utils.h>

#include "rml_model_step.h"
#include "rml_step_file.h"
#include "rml_step_geometry.h"
#include "rml_step_tessellator.h"

RModelStep::RModelStep()
{
    this->_init();
}

RModelStep::RModelStep(const RModelStep &modelStep) : RModelRaw(modelStep)
{
    this->_init(&modelStep);
}

RModelStep::~RModelStep()
{
}

RModelStep &RModelStep::operator =(const RModelStep &modelStep)
{
    this->RModelRaw::operator =(modelStep);
    this->_init(&modelStep);
    return (*this);
}

void RModelStep::_init(const RModelStep *pModelStep)
{
    this->maxAngle = 3.14159265358979323846 / 12.0;
    this->chordTolerance = 1e-3;
    if (pModelStep)
    {
        this->name = pModelStep->name;
        this->description = pModelStep->description;
        this->bodyNames = pModelStep->bodyNames;
        this->elementBodies = pModelStep->elementBodies;
        this->maxAngle = pModelStep->maxAngle;
        this->chordTolerance = pModelStep->chordTolerance;
    }
}

QString RModelStep::getName() const
{
    return this->name;
}

void RModelStep::setName(const QString &name)
{
    this->name = name;
}

QString RModelStep::getDescription() const
{
    return this->description;
}

void RModelStep::setDescription(const QString &description)
{
    this->description = description;
}

unsigned int RModelStep::getNBodies() const
{
    return unsigned(this->bodyNames.size());
}

const QString &RModelStep::getBodyName(unsigned int bodyID) const
{
    return this->bodyNames.at(bodyID);
}

unsigned int RModelStep::addBody(const QString &name)
{
    this->bodyNames.append(name);
    return unsigned(this->bodyNames.size() - 1);
}

unsigned int RModelStep::getElementBody(unsigned int elementID) const
{
    return (elementID < this->elementBodies.size()) ? this->elementBodies[elementID] : 0;
}

unsigned int RModelStep::addNode(const RNode &node)
{
    this->nodes.push_back(node);
    return unsigned(this->nodes.size() - 1);
}

void RModelStep::addBodyTriangle(unsigned int bodyID, unsigned int nodeID1, unsigned int nodeID2, unsigned int nodeID3)
{
    RElement element;
    element.setType(R_ELEMENT_TRI1);
    element.setNodeId(0,nodeID1);
    element.setNodeId(1,nodeID2);
    element.setNodeId(2,nodeID3);
    this->elements.push_back(element);
    this->elementBodies.push_back(bodyID);
}

void RModelStep::setTessellationTolerances(double maxAngle, double relativeChordTolerance)
{
    this->maxAngle = maxAngle;
    this->chordTolerance = relativeChordTolerance;
}

void RModelStep::clear()
{
    this->RModelRaw::clear();
    this->name.clear();
    this->description.clear();
    this->bodyNames.clear();
    this->elementBodies.clear();
}

#ifndef NAME
#define NAME "Range"
#endif
#ifndef VERSION
#define VERSION ""
#endif

namespace
{

//! Return SI prefix factor.
double siPrefixFactor(const QString &prefix)
{
    static const std::map<QString,double> prefixes = {
        {"EXA",1e18}, {"PETA",1e15}, {"TERA",1e12}, {"GIGA",1e9}, {"MEGA",1e6}, {"KILO",1e3},
        {"HECTO",1e2}, {"DECA",1e1}, {"DECI",1e-1}, {"CENTI",1e-2}, {"MILLI",1e-3}, {"MICRO",1e-6},
        {"NANO",1e-9}, {"PICO",1e-12}, {"FEMTO",1e-15}, {"ATTO",1e-18}
    };
    auto iter = prefixes.find(prefix);
    return (iter == prefixes.end()) ? 1.0 : iter->second;
}

//! Return conversion factor of unit to SI base unit (metre, radian).
double unitFactor(const RStepFile &file, unsigned int unitId, int depth = 0)
{
    const RStepEntity *pUnit = file.find(unitId);
    if (!pUnit || depth > 8)
    {
        return 1.0;
    }
    if (const RStepRecord *pSi = pUnit->find("SI_UNIT"))
    {
        // Complex instance: SI_UNIT(prefix,name), simple instance: SI_UNIT(dimensions,prefix,name).
        size_t prefixPosition = pUnit->isComplex() ? 0 : 1;
        const RStepValue &prefix = pSi->param(prefixPosition);
        return (prefix.type == RStepValue::Enum) ? siPrefixFactor(prefix.text) : 1.0;
    }
    if (const RStepRecord *pConversion = pUnit->find("CONVERSION_BASED_UNIT"))
    {
        const RStepEntity *pMeasure = file.find(pConversion->param(1).toId());
        if (!pMeasure || pMeasure->records.empty())
        {
            return 1.0;
        }
        // LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE(25.4),#unit) or complex MEASURE_WITH_UNIT.
        const RStepRecord *pRecord = pMeasure->find("MEASURE_WITH_UNIT");
        if (!pRecord)
        {
            pRecord = &pMeasure->records[0];
        }
        double value = pRecord->param(0).toDouble(1.0);
        return value * unitFactor(file,pRecord->param(1).toId(),depth + 1);
    }
    return 1.0;
}

//! Find length and plane angle unit factors of representation context.
bool contextUnits(const RStepFile &file, unsigned int contextId, double &lengthScale, double &angleScale)
{
    const RStepEntity *pContext = file.find(contextId);
    if (!pContext)
    {
        return false;
    }
    const RStepRecord *pUnits = pContext->find("GLOBAL_UNIT_ASSIGNED_CONTEXT");
    if (!pUnits)
    {
        return false;
    }
    bool found = false;
    for (unsigned int unitId : pUnits->param(0).toIdList())
    {
        const RStepEntity *pUnit = file.find(unitId);
        if (!pUnit)
        {
            continue;
        }
        if (pUnit->has("LENGTH_UNIT"))
        {
            lengthScale = unitFactor(file,unitId);
            found = true;
        }
        else if (pUnit->has("PLANE_ANGLE_UNIT"))
        {
            angleScale = unitFactor(file,unitId);
        }
    }
    return found;
}

//! Representation record (name, items, context) of entity or nullptr.
const RStepRecord *findRepresentation(const RStepEntity &entity)
{
    for (const RStepRecord &record : entity.records)
    {
        if (record.keyword.endsWith("REPRESENTATION") && record.params.size() >= 3 && record.param(1).isList())
        {
            return &record;
        }
    }
    return nullptr;
}

//! Union find.
unsigned int findRoot(std::map<unsigned int,unsigned int> &parents, unsigned int id)
{
    auto iter = parents.find(id);
    if (iter == parents.end())
    {
        parents[id] = id;
        return id;
    }
    while (parents[id] != id)
    {
        parents[id] = parents[parents[id]];
        id = parents[id];
    }
    return id;
}

//! Shape item instance.
struct ItemInstance
{
    unsigned int itemId;
    RStepTransform transform;
    unsigned int frame;
};

//! Spatial hash for merging nodes.
class NodeMerger
{
    public:

        NodeMerger(double tolerance) : tolerance(tolerance) {}

        unsigned int find(const RStepVector &p, const std::vector<RNode> &nodes, unsigned int newId)
        {
            if (this->tolerance <= 0.0)
            {
                Key key{p.x,p.y,p.z};
                auto result = this->exact.insert({key,newId});
                return result.first->second;
            }
            long long ix = (long long)std::floor(p.x / this->tolerance);
            long long iy = (long long)std::floor(p.y / this->tolerance);
            long long iz = (long long)std::floor(p.z / this->tolerance);
            for (long long dx=-1;dx<=1;dx++)
            {
                for (long long dy=-1;dy<=1;dy++)
                {
                    for (long long dz=-1;dz<=1;dz++)
                    {
                        auto iter = this->cells.find(Cell{ix+dx,iy+dy,iz+dz});
                        if (iter == this->cells.end())
                        {
                            continue;
                        }
                        for (unsigned int id : iter->second)
                        {
                            const RNode &node = nodes[id];
                            double ex = node.getX() - p.x, ey = node.getY() - p.y, ez = node.getZ() - p.z;
                            if (ex * ex + ey * ey + ez * ez <= this->tolerance * this->tolerance)
                            {
                                return id;
                            }
                        }
                    }
                }
            }
            this->cells[Cell{ix,iy,iz}].push_back(newId);
            return newId;
        }

    protected:

        struct Key
        {
            double x, y, z;
            bool operator==(const Key &k) const { return x == k.x && y == k.y && z == k.z; }
        };
        struct KeyHash
        {
            size_t operator()(const Key &k) const
            {
                return std::hash<double>()(k.x) ^ (std::hash<double>()(k.y) * 31) ^ (std::hash<double>()(k.z) * 131);
            }
        };
        struct Cell
        {
            long long x, y, z;
            bool operator==(const Cell &c) const { return x == c.x && y == c.y && z == c.z; }
        };
        struct CellHash
        {
            size_t operator()(const Cell &c) const
            {
                return std::hash<long long>()(c.x * 73856093LL ^ c.y * 19349663LL ^ c.z * 83492791LL);
            }
        };

        double tolerance;
        std::unordered_map<Key,unsigned int,KeyHash> exact;
        std::unordered_map<Cell,std::vector<unsigned int>,CellHash> cells;
};

} // namespace

void RModelStep::read(const QString &fileName, double tolerance)
{
    RLogger::info("Reading STEP file \'%s\'\n",fileName.toUtf8().constData());
    RLogger::indent();

    RStepFile file;
    try
    {
        file.read(fileName);
    }
    catch (const RError &error)
    {
        RLogger::unindent();
        throw error;
    }

    this->clear();
    this->name = QFileInfo(fileName).baseName();
    if (const RStepRecord *pDescription = file.findHeader("FILE_DESCRIPTION"))
    {
        QStringList lines;
        for (const RStepValue &line : pDescription->param(0).items)
        {
            if (!line.text.isEmpty())
            {
                lines.append(line.text);
            }
        }
        this->description = lines.join(" ");
    }

    const std::unordered_map<unsigned int,RStepEntity> &entities = file.getEntities();

    // Representations, their contexts and relationships.
    std::map<unsigned int,const RStepRecord*> representations;
    std::map<unsigned int,unsigned int> frameParents;
    struct Placement
    {
        unsigned int child;
        unsigned int parent;
        RStepTransform transform;
    };
    std::vector<Placement> placements;
    std::map<unsigned int,QString> productNames;

    for (const auto &iter : entities)
    {
        const RStepEntity &entity = iter.second;
        if (const RStepRecord *pRepresentation = findRepresentation(entity))
        {
            representations[entity.id] = pRepresentation;
            findRoot(frameParents,entity.id);
        }
    }

    for (const auto &iter : entities)
    {
        const RStepEntity &entity = iter.second;
        const RStepRecord *pRelationship = entity.find("REPRESENTATION_RELATIONSHIP");
        if (!pRelationship)
        {
            pRelationship = entity.find("SHAPE_REPRESENTATION_RELATIONSHIP");
        }
        if (pRelationship && pRelationship->params.size() >= 4)
        {
            unsigned int rep1 = pRelationship->param(2).toId();
            unsigned int rep2 = pRelationship->param(3).toId();
            if (!representations.count(rep1) || !representations.count(rep2))
            {
                continue;
            }
            const RStepRecord *pWithTransformation = entity.find("REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION");
            if (!pWithTransformation)
            {
                // Same coordinate frame.
                unsigned int root1 = findRoot(frameParents,rep1);
                unsigned int root2 = findRoot(frameParents,rep2);
                if (root1 != root2)
                {
                    frameParents[root1] = root2;
                }
                continue;
            }
            const RStepEntity *pTransformation = file.find(pWithTransformation->param(0).toId());
            RStepTransform transform;
            if (pTransformation && pTransformation->keyword() == "ITEM_DEFINED_TRANSFORMATION")
            {
                RStepTransform t1, t2;
                if (RStepGeometry::readTransform(file,pTransformation->records[0].param(2).toId(),t1) &&
                    RStepGeometry::readTransform(file,pTransformation->records[0].param(3).toId(),t2))
                {
                    transform = t2 * t1.inverse();
                }
            }
            else if (pTransformation)
            {
                RStepGeometry::readTransform(file,pTransformation->id,transform);
            }
            placements.push_back({rep1,rep2,transform});
        }
        else if (entity.keyword() == "SHAPE_DEFINITION_REPRESENTATION")
        {
            const RStepEntity *pShape = file.find(entity.records[0].param(0).toId());
            const RStepEntity *pDefinition = pShape ? file.find(pShape->records[0].param(2).toId()) : nullptr;
            const RStepEntity *pFormation = pDefinition ? file.find(pDefinition->records[0].param(2).toId()) : nullptr;
            const RStepEntity *pProduct = pFormation ? file.find(pFormation->records[0].param(2).toId()) : nullptr;
            if (pProduct && pProduct->keyword() == "PRODUCT")
            {
                QString productName = pProduct->records[0].param(1).text;
                if (productName.isEmpty())
                {
                    productName = pProduct->records[0].param(0).text;
                }
                productNames[entity.records[0].param(1).toId()] = productName;
            }
        }
    }

    // Frame instances (assembly placements).
    std::map<unsigned int,std::vector<std::pair<unsigned int,RStepTransform>>> frameLinks;
    for (const Placement &placement : placements)
    {
        unsigned int child = findRoot(frameParents,placement.child);
        unsigned int parent = findRoot(frameParents,placement.parent);
        if (child != parent)
        {
            frameLinks[child].push_back({parent,placement.transform});
        }
    }
    std::map<unsigned int,std::vector<RStepTransform>> frameInstances;
    std::set<unsigned int> visiting;
    std::function<const std::vector<RStepTransform>&(unsigned int)> getInstances = [&](unsigned int frame) -> const std::vector<RStepTransform>&
    {
        auto iter = frameInstances.find(frame);
        if (iter != frameInstances.end())
        {
            return iter->second;
        }
        std::vector<RStepTransform> instances;
        auto links = frameLinks.find(frame);
        if (links == frameLinks.end() || visiting.count(frame))
        {
            instances.push_back(RStepTransform());
        }
        else
        {
            visiting.insert(frame);
            for (const auto &link : links->second)
            {
                for (const RStepTransform &parentTransform : getInstances(link.first))
                {
                    instances.push_back(parentTransform * link.second);
                    if (instances.size() > 100000)
                    {
                        break;
                    }
                }
            }
            visiting.erase(frame);
        }
        return frameInstances[frame] = instances;
    };

    // Units - taken from the context of the first representation carrying shape items.
    double lengthScale = 0.0;
    double angleScale = 1.0;
    bool mixedUnits = false;

    // Shape items of representations (mapped items are expanded).
    std::vector<ItemInstance> items;
    bool hasBRep = false;
    std::function<void(unsigned int,const RStepTransform&,unsigned int,int)> collectItems =
        [&](unsigned int representationId, const RStepTransform &transform, unsigned int frame, int depth)
    {
        auto iter = representations.find(representationId);
        if (iter == representations.end() || depth > 16)
        {
            return;
        }
        for (unsigned int itemId : iter->second->param(1).toIdList())
        {
            const RStepEntity *pItem = file.find(itemId);
            if (!pItem)
            {
                continue;
            }
            if (pItem->keyword() == "MAPPED_ITEM")
            {
                const RStepEntity *pMap = file.find(pItem->records[0].param(1).toId());
                if (!pMap || pMap->keyword() != "REPRESENTATION_MAP")
                {
                    continue;
                }
                RStepTransform origin, target;
                RStepGeometry::readTransform(file,pMap->records[0].param(0).toId(),origin);
                RStepGeometry::readTransform(file,pItem->records[0].param(2).toId(),target);
                collectItems(pMap->records[0].param(1).toId(),transform * target * origin.inverse(),frame,depth + 1);
                continue;
            }
            RStepTessellator::ItemType type = RStepTessellator::getItemType(*pItem);
            if (type == RStepTessellator::ItemNone)
            {
                continue;
            }
            if (type == RStepTessellator::ItemBRep)
            {
                hasBRep = true;
            }
            double repLengthScale = 1.0;
            double repAngleScale = 1.0;
            if (contextUnits(file,iter->second->param(2).toId(),repLengthScale,repAngleScale))
            {
                if (lengthScale == 0.0)
                {
                    lengthScale = repLengthScale;
                    angleScale = repAngleScale;
                }
                else if (std::abs(repLengthScale - lengthScale) > 1e-12 * lengthScale)
                {
                    mixedUnits = true;
                }
            }
            items.push_back({itemId,transform,frame});
        }
    };

    std::map<unsigned int,std::vector<unsigned int>> frameRepresentations;
    for (const auto &iter : representations)
    {
        frameRepresentations[findRoot(frameParents,iter.first)].push_back(iter.first);
    }

    std::vector<ItemInstance> instances;
    for (const auto &frame : frameRepresentations)
    {
        items.clear();
        for (unsigned int representationId : frame.second)
        {
            collectItems(representationId,RStepTransform(),frame.first,0);
        }
        if (items.empty())
        {
            continue;
        }
        // Same item listed in several representations of one frame is taken once.
        std::set<std::pair<unsigned int,std::vector<double>>> unique;
        for (const ItemInstance &item : items)
        {
            const RStepTransform &t = item.transform;
            std::vector<double> key = {t.r[0].x,t.r[0].y,t.r[0].z,t.r[1].x,t.r[1].y,t.r[1].z,t.r[2].x,t.r[2].y,t.r[2].z,t.t.x,t.t.y,t.t.z,t.s};
            if (!unique.insert({item.itemId,key}).second)
            {
                continue;
            }
            for (const RStepTransform &frameTransform : getInstances(frame.first))
            {
                instances.push_back({item.itemId,frameTransform * item.transform,frame.first});
            }
        }
    }

    if (hasBRep)
    {
        // Prefer exact geometry over tessellated copies.
        std::vector<ItemInstance> brepInstances;
        for (const ItemInstance &instance : instances)
        {
            if (RStepTessellator::getItemType(*file.find(instance.itemId)) == RStepTessellator::ItemBRep)
            {
                brepInstances.push_back(instance);
            }
        }
        instances.swap(brepInstances);
    }

    if (instances.empty())
    {
        RLogger::unindent();
        throw RError(RError::Type::InvalidInput,R_ERROR_REF,"STEP file \'%s\' does not contain any solid, shell or tessellated geometry.",fileName.toUtf8().constData());
    }

    if (lengthScale == 0.0)
    {
        lengthScale = 1e-3;
        RLogger::warning("No length unit found, millimeters are assumed.\n");
    }
    if (mixedUnits)
    {
        RLogger::warning("Representations use different length units, geometry may be scaled incorrectly.\n");
    }
    RLogger::info("Length unit: %g m\n",lengthScale);

    RStepTessellator tessellator(file,angleScale);

    // Global size used if item size cannot be determined.
    double globalSize = 0.0;
    {
        RStepVector lo(1e300,1e300,1e300), hi(-1e300,-1e300,-1e300);
        for (const auto &iter : entities)
        {
            if (iter.second.keyword() == "VERTEX_POINT")
            {
                RStepVector p;
                if (RStepGeometry::readVertex(file,iter.first,p))
                {
                    lo = RStepVector(std::min(lo.x,p.x),std::min(lo.y,p.y),std::min(lo.z,p.z));
                    hi = RStepVector(std::max(hi.x,p.x),std::max(hi.y,p.y),std::max(hi.z,p.z));
                }
            }
        }
        globalSize = (hi.x >= lo.x) ? (hi - lo).length() : 1.0 / lengthScale;
    }

    std::map<unsigned int,RStepMesh> meshCache;
    std::map<QString,unsigned int> nameCounts;

    RProgressInitialize("Tessellating STEP geometry");

    for (size_t i=0;i<instances.size();i++)
    {
        RProgressPrint(i+1,instances.size());

        const ItemInstance &instance = instances[i];
        auto iter = meshCache.find(instance.itemId);
        if (iter == meshCache.end())
        {
            double size = tessellator.estimateSize(instance.itemId);
            if (size <= 0.0)
            {
                size = globalSize;
            }
            tessellator.setTolerances(this->maxAngle,this->chordTolerance * size);
            RStepMesh mesh;
            tessellator.tessellateItem(instance.itemId,mesh);
            iter = meshCache.insert({instance.itemId,mesh}).first;
        }
        const RStepMesh &localMesh = iter->second;
        if (localMesh.triangles.empty())
        {
            continue;
        }

        RStepMesh mesh;
        mesh.append(localMesh,instance.transform);

        // Body name.
        QString bodyName = RStepTessellator::getEntityName(*file.find(instance.itemId)).trimmed();
        if (bodyName.isEmpty() || bodyName.compare("NONE",Qt::CaseInsensitive) == 0)
        {
            for (const auto &product : productNames)
            {
                if (findRoot(frameParents,product.first) == instance.frame)
                {
                    bodyName = product.second.trimmed();
                    break;
                }
            }
        }
        if (bodyName.isEmpty())
        {
            bodyName = "Body";
        }
        unsigned int count = ++nameCounts[bodyName];
        if (count > 1)
        {
            bodyName += QString(" (%1)").arg(count);
        }

        // Merge nodes within the body.
        double size = 0.0;
        {
            RStepVector lo(1e300,1e300,1e300), hi(-1e300,-1e300,-1e300);
            for (const RStepVector &p : mesh.nodes)
            {
                lo = RStepVector(std::min(lo.x,p.x),std::min(lo.y,p.y),std::min(lo.z,p.z));
                hi = RStepVector(std::max(hi.x,p.x),std::max(hi.y,p.y),std::max(hi.z,p.z));
            }
            size = (hi - lo).length() * lengthScale;
        }
        NodeMerger merger(std::max(tolerance,1e-9 * size));
        std::vector<unsigned int> nodeMap(mesh.nodes.size());
        for (size_t j=0;j<mesh.nodes.size();j++)
        {
            RStepVector p = mesh.nodes[j] * lengthScale;
            unsigned int newId = unsigned(this->nodes.size());
            unsigned int id = merger.find(p,this->nodes,newId);
            if (id == newId)
            {
                this->nodes.push_back(RNode(p.x,p.y,p.z));
            }
            nodeMap[j] = id;
        }

        unsigned int bodyID = RConstants::eod;
        for (const auto &triangle : mesh.triangles)
        {
            unsigned int n1 = nodeMap[triangle[0]];
            unsigned int n2 = nodeMap[triangle[1]];
            unsigned int n3 = nodeMap[triangle[2]];
            if (n1 == n2 || n2 == n3 || n1 == n3)
            {
                continue;
            }
            if (bodyID == RConstants::eod)
            {
                bodyID = this->addBody(bodyName);
            }
            this->addBodyTriangle(bodyID,n1,n2,n3);
        }
    }

    RProgressFinalize();

    if (tessellator.getNFailedFaces() > 0)
    {
        QStringList faceIds;
        for (unsigned int faceId : tessellator.getFailedFaces())
        {
            if (faceIds.size() >= 10)
            {
                faceIds.append("...");
                break;
            }
            faceIds.append(QString("#%1").arg(faceId));
        }
        RLogger::warning("%u face(s) could not be tessellated - surface may contain holes: %s\n",tessellator.getNFailedFaces(),faceIds.join(", ").toUtf8().constData());
    }
    if (tessellator.getNUnsupportedEdges() > 0)
    {
        RLogger::warning("%u edge(s) with unsupported curve geometry were replaced by straight lines.\n",tessellator.getNUnsupportedEdges());
    }
    RLogger::info("Bodies: %u, nodes: %u, triangles: %u\n",this->getNBodies(),this->getNNodes(),this->getNElements());
    RLogger::unindent();

    if (this->getNElements() == 0)
    {
        throw RError(RError::Type::InvalidInput,R_ERROR_REF,"No geometry could be tessellated from STEP file \'%s\'.",fileName.toUtf8().constData());
    }
}

void RModelStep::write(const QString &fileName) const
{
    RLogger::info("Writing STEP file \'%s\'\n",fileName.toUtf8().constData());

    if (fileName.isEmpty())
    {
        throw RError(RError::Type::InvalidFileName,R_ERROR_REF,"No file name was provided.");
    }

    QFile stepFile(fileName);

    if (!stepFile.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        throw RError(RError::Type::OpenFile,R_ERROR_REF,"Failed to open the file \'%s\'.",fileName.toUtf8().constData());
    }

    QTextStream out(&stepFile);

    QString modelName = this->name.isEmpty() ? QFileInfo(fileName).baseName() : this->name;

    out << "ISO-10303-21;\n";
    out << "HEADER;\n";
    out << "FILE_DESCRIPTION((" << RStepFile::formatString(this->description.isEmpty() ? QString("Range FEA model") : this->description) << "),'2;1');\n";
    out << "FILE_NAME(" << RStepFile::formatString(QFileInfo(fileName).fileName()) << ","
        << RStepFile::formatString(QDateTime::currentDateTime().toString(Qt::ISODate)) << ",(''),(''),"
        << RStepFile::formatString(QString(NAME) + " " + QString(VERSION)) << ","
        << RStepFile::formatString(QString(NAME)) << ",'');\n";
    out << "FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));\n";
    out << "ENDSEC;\n";
    out << "DATA;\n";

    unsigned int nextId = 1;
    auto add = [&](const QString &text) -> unsigned int
    {
        unsigned int id = nextId++;
        out << '#' << id << '=' << text << ";\n";
        return id;
    };
    auto ref = [](unsigned int id) { return QString("#%1").arg(id); };
    auto real = [](double value) { return RStepFile::formatReal(value); };
    auto vector = [&](double x, double y, double z) { return QString("(%1,%2,%3)").arg(real(x),real(y),real(z)); };

    // Context.
    unsigned int applicationContext = add("APPLICATION_CONTEXT('automotive design')");
    add(QString("APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,%1)").arg(ref(applicationContext)));
    unsigned int productContext = add(QString("PRODUCT_CONTEXT('',%1,'mechanical')").arg(ref(applicationContext)));
    unsigned int definitionContext = add(QString("PRODUCT_DEFINITION_CONTEXT('part definition',%1,'design')").arg(ref(applicationContext)));
    unsigned int lengthUnit = add("(LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT($,.METRE.))");
    unsigned int angleUnit = add("(NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.))");
    unsigned int solidAngleUnit = add("(NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT())");
    unsigned int uncertainty = add(QString("UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-09),%1,'distance_accuracy_value','confusion accuracy')").arg(ref(lengthUnit)));
    unsigned int context = add(QString("(GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((%1)) GLOBAL_UNIT_ASSIGNED_CONTEXT((%2,%3,%4)) REPRESENTATION_CONTEXT('Context #1','3D Context with UNIT and UNCERTAINTY'))")
                               .arg(ref(uncertainty),ref(lengthUnit),ref(angleUnit),ref(solidAngleUnit)));
    unsigned int originPoint = add("CARTESIAN_POINT('',(0.,0.,0.))");
    unsigned int axisZ = add("DIRECTION('',(0.,0.,1.))");
    unsigned int axisX = add("DIRECTION('',(1.,0.,0.))");
    unsigned int origin = add(QString("AXIS2_PLACEMENT_3D('',%1,%2,%3)").arg(ref(originPoint),ref(axisZ),ref(axisX)));

    std::vector<unsigned int> solids;
    std::vector<unsigned int> shellModels;

    RProgressInitialize("Writing STEP file");

    unsigned int nBodies = std::max(this->getNBodies(),1u);
    std::vector<std::vector<unsigned int>> bodyElements(nBodies);
    for (unsigned int i=0;i<this->getNElements();i++)
    {
        if (this->elements[i].getType() == R_ELEMENT_TRI1)
        {
            bodyElements[std::min(this->getElementBody(i),nBodies-1)].push_back(i);
        }
    }

    for (unsigned int b=0;b<nBodies;b++)
    {
        RProgressPrint(b+1,nBodies);

        // Non-degenerate triangles.
        std::vector<unsigned int> triangles;
        std::vector<RStepVector> normals;
        for (unsigned int elementID : bodyElements[b])
        {
            const RElement &element = this->elements[elementID];
            const RNode &n1 = this->nodes[element.getNodeId(0)];
            const RNode &n2 = this->nodes[element.getNodeId(1)];
            const RNode &n3 = this->nodes[element.getNodeId(2)];
            RStepVector a(n2.getX() - n1.getX(),n2.getY() - n1.getY(),n2.getZ() - n1.getZ());
            RStepVector c(n3.getX() - n1.getX(),n3.getY() - n1.getY(),n3.getZ() - n1.getZ());
            RStepVector normal = a.cross(c);
            if (normal.length() <= 1e-14 * std::max(a.dot(a),c.dot(c)) || element.getNodeId(0) == element.getNodeId(1) ||
                element.getNodeId(1) == element.getNodeId(2) || element.getNodeId(0) == element.getNodeId(2))
            {
                continue;
            }
            triangles.push_back(elementID);
            normals.push_back(normal.normalized());
        }
        if (triangles.empty())
        {
            continue;
        }

        // Edges and closure check - closed shell has every edge used once in each direction.
        std::map<std::pair<unsigned int,unsigned int>,int> edgeUse;
        for (unsigned int elementID : triangles)
        {
            const RElement &element = this->elements[elementID];
            for (int k=0;k<3;k++)
            {
                unsigned int a = element.getNodeId(k);
                unsigned int c = element.getNodeId((k+1)%3);
                edgeUse[{a,c}]++;
            }
        }
        bool closed = true;
        for (const auto &edge : edgeUse)
        {
            auto reverse = edgeUse.find({edge.first.second,edge.first.first});
            if (edge.second != 1 || reverse == edgeUse.end() || reverse->second != 1)
            {
                closed = false;
                break;
            }
        }

        // Vertices.
        std::map<unsigned int,std::pair<unsigned int,unsigned int>> vertices;
        for (unsigned int elementID : triangles)
        {
            const RElement &element = this->elements[elementID];
            for (int k=0;k<3;k++)
            {
                unsigned int nodeID = element.getNodeId(k);
                if (vertices.count(nodeID))
                {
                    continue;
                }
                const RNode &node = this->nodes[nodeID];
                unsigned int point = add(QString("CARTESIAN_POINT('',%1)").arg(vector(node.getX(),node.getY(),node.getZ())));
                unsigned int vertex = add(QString("VERTEX_POINT('',%1)").arg(ref(point)));
                vertices[nodeID] = {point,vertex};
            }
        }

        // Edges - oriented from lower to higher node ID.
        std::map<std::pair<unsigned int,unsigned int>,unsigned int> edges;
        for (const auto &use : edgeUse)
        {
            unsigned int a = std::min(use.first.first,use.first.second);
            unsigned int c = std::max(use.first.first,use.first.second);
            if (edges.count({a,c}))
            {
                continue;
            }
            const RNode &na = this->nodes[a];
            const RNode &nc = this->nodes[c];
            RStepVector d(nc.getX() - na.getX(),nc.getY() - na.getY(),nc.getZ() - na.getZ());
            double length = d.length();
            d = d.normalized();
            unsigned int direction = add(QString("DIRECTION('',%1)").arg(vector(d.x,d.y,d.z)));
            unsigned int lineVector = add(QString("VECTOR('',%1,%2)").arg(ref(direction),real(length)));
            unsigned int line = add(QString("LINE('',%1,%2)").arg(ref(vertices[a].first),ref(lineVector)));
            edges[{a,c}] = add(QString("EDGE_CURVE('',%1,%2,%3,.T.)").arg(ref(vertices[a].second),ref(vertices[c].second),ref(line)));
        }

        // Planar faces.
        QStringList faces;
        for (size_t i=0;i<triangles.size();i++)
        {
            const RElement &element = this->elements[triangles[i]];
            QStringList orientedEdges;
            for (int k=0;k<3;k++)
            {
                unsigned int a = element.getNodeId(k);
                unsigned int c = element.getNodeId((k+1)%3);
                unsigned int edge = edges[{std::min(a,c),std::max(a,c)}];
                orientedEdges.append(ref(add(QString("ORIENTED_EDGE('',*,*,%1,%2)").arg(ref(edge),(a < c) ? ".T." : ".F."))));
            }
            unsigned int loop = add(QString("EDGE_LOOP('',(%1))").arg(orientedEdges.join(',')));
            unsigned int bound = add(QString("FACE_OUTER_BOUND('',%1,.T.)").arg(ref(loop)));
            const RNode &n1 = this->nodes[element.getNodeId(0)];
            const RNode &n2 = this->nodes[element.getNodeId(1)];
            RStepVector reference = RStepVector(n2.getX() - n1.getX(),n2.getY() - n1.getY(),n2.getZ() - n1.getZ());
            reference = (reference - normals[i] * reference.dot(normals[i])).normalized();
            unsigned int normalDirection = add(QString("DIRECTION('',%1)").arg(vector(normals[i].x,normals[i].y,normals[i].z)));
            unsigned int referenceDirection = add(QString("DIRECTION('',%1)").arg(vector(reference.x,reference.y,reference.z)));
            unsigned int placement = add(QString("AXIS2_PLACEMENT_3D('',%1,%2,%3)").arg(ref(vertices[element.getNodeId(0)].first),ref(normalDirection),ref(referenceDirection)));
            unsigned int plane = add(QString("PLANE('',%1)").arg(ref(placement)));
            faces.append(ref(add(QString("ADVANCED_FACE('',(%1),%2,.T.)").arg(ref(bound),ref(plane)))));
        }

        QString bodyName = (b < this->getNBodies()) ? this->getBodyName(b) : modelName;
        if (closed)
        {
            unsigned int shell = add(QString("CLOSED_SHELL('',(%1))").arg(faces.join(',')));
            solids.push_back(add(QString("MANIFOLD_SOLID_BREP(%1,%2)").arg(RStepFile::formatString(bodyName),ref(shell))));
        }
        else
        {
            unsigned int shell = add(QString("OPEN_SHELL('',(%1))").arg(faces.join(',')));
            shellModels.push_back(add(QString("SHELL_BASED_SURFACE_MODEL(%1,(%2))").arg(RStepFile::formatString(bodyName),ref(shell))));
        }
    }

    // Product structure.
    unsigned int product = add(QString("PRODUCT(%1,%1,'',(%2))").arg(RStepFile::formatString(modelName),ref(productContext)));
    add(QString("PRODUCT_RELATED_PRODUCT_CATEGORY('part',$,(%1))").arg(ref(product)));
    unsigned int formation = add(QString("PRODUCT_DEFINITION_FORMATION('','',%1)").arg(ref(product)));
    unsigned int definition = add(QString("PRODUCT_DEFINITION('design','',%1,%2)").arg(ref(formation),ref(definitionContext)));
    unsigned int definitionShape = add(QString("PRODUCT_DEFINITION_SHAPE('','',%1)").arg(ref(definition)));
    unsigned int shapeRepresentation = add(QString("SHAPE_REPRESENTATION(%1,(%2),%3)").arg(RStepFile::formatString(modelName),ref(origin),ref(context)));
    add(QString("SHAPE_DEFINITION_REPRESENTATION(%1,%2)").arg(ref(definitionShape),ref(shapeRepresentation)));

    auto itemList = [&](const std::vector<unsigned int> &ids)
    {
        QStringList list(ref(origin));
        for (unsigned int id : ids)
        {
            list.append(ref(id));
        }
        return list.join(',');
    };
    if (!solids.empty())
    {
        unsigned int representation = add(QString("ADVANCED_BREP_SHAPE_REPRESENTATION('',(%1),%2)").arg(itemList(solids),ref(context)));
        add(QString("SHAPE_REPRESENTATION_RELATIONSHIP('','',%1,%2)").arg(ref(shapeRepresentation),ref(representation)));
    }
    if (!shellModels.empty())
    {
        unsigned int representation = add(QString("MANIFOLD_SURFACE_SHAPE_REPRESENTATION('',(%1),%2)").arg(itemList(shellModels),ref(context)));
        add(QString("SHAPE_REPRESENTATION_RELATIONSHIP('','',%1,%2)").arg(ref(shapeRepresentation),ref(representation)));
    }

    out << "ENDSEC;\n";
    out << "END-ISO-10303-21;\n";

    out.flush();
    if (stepFile.error() != QFile::NoError)
    {
        throw RError(RError::Type::WriteFile,R_ERROR_REF,"Failed to write to file \'%s\'.",fileName.toUtf8().constData());
    }
    stepFile.close();

    RProgressFinalize("Done");
}
