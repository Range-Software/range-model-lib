#include <rbl_error.h>

#include "rml_problem.h"

typedef struct _RProblemDesc
{
    QString             id;
    QString             name;
    QString             desc;
    RProblemType        type;
    RProblemTypeMask    requiredProblemTypeMask;
    RProblemTypeMask    excludedProblemTypeMask;
    bool                timeSolverEnabled;
} RProblemDesc;

const RProblemDesc problemDesc [] =
{
    {
        "prb-acoustics",
        "Acoustics",
        "Customized wave equation for solving acoustic problems.",
        R_PROBLEM_ACOUSTICS,
        R_PROBLEM_NONE,
        R_PROBLEM_WAVE | R_PROBLEM_STRESS_MODAL,
        true
    },
    {
        "prb-contaminant",
        "Contaminant dispersion",
        "Calculate distribution of contaminant in the flow field.",
        R_PROBLEM_FLUID_PARTICLE,
        R_PROBLEM_FLUID,
        R_PROBLEM_STRESS_MODAL,
        true
    },
    {
        "prb-electrostatics",
        "Electro-statics",
        "Calculate electric field, current density and generated Joule\'s heat.",
        R_PROBLEM_ELECTROSTATICS,
        R_PROBLEM_NONE,
        R_PROBLEM_STRESS_MODAL,
        false
    },
    {
        "prb-magnetostatics",
        "Magneto-statics",
        "Calculate magnetic field.",
        R_PROBLEM_MAGNETOSTATICS,
        R_PROBLEM_ELECTROSTATICS,
        R_PROBLEM_STRESS_MODAL,
        false
    },
    {
        "prb-fluid",
        "Incompressible viscous flow",
        "Steady-state and transient flow of newtonian fluids.",
        R_PROBLEM_FLUID,
        R_PROBLEM_NONE,
        R_PROBLEM_POTENTIAL | R_PROBLEM_STRESS_MODAL,
        true
    },
    {
        "prb-fluid-heat",
        "Heat transfer in fluids",
        "Heat transfer in fluids.",
        R_PROBLEM_FLUID_HEAT,
        R_PROBLEM_FLUID,
        R_PROBLEM_STRESS_MODAL,
        true
    },
    {
        "prb-heat",
        "Heat transfer",
        "Steady-state and transient Heat-transfer.",
        R_PROBLEM_HEAT,
        R_PROBLEM_NONE,
        R_PROBLEM_STRESS_MODAL,
        true
    },
    {
        "prb-radiative_heat",
        "Radiative heat transfer",
        "Steady-state and transient radiative heat-transfer.",
        R_PROBLEM_RADIATIVE_HEAT,
        R_PROBLEM_HEAT,
        R_PROBLEM_STRESS_MODAL,
        true
    },
    {
        "prb-stress",
        "Stress analysis",
        "Stress and linear elasto-dynamics analysis including modal analysis.",
        R_PROBLEM_STRESS,
        R_PROBLEM_NONE,
        R_PROBLEM_NONE,
        true
    },
    {
        "prb-stress-modal",
        "Modal analysis",
        "Stress modal analysis.",
        R_PROBLEM_STRESS_MODAL,
        R_PROBLEM_NONE,
        R_PROBLEM_ACOUSTICS | R_PROBLEM_FLUID_PARTICLE | R_PROBLEM_ELECTROSTATICS | R_PROBLEM_MAGNETOSTATICS | R_PROBLEM_FLUID | R_PROBLEM_FLUID_HEAT | R_PROBLEM_HEAT | R_PROBLEM_RADIATIVE_HEAT | R_PROBLEM_POTENTIAL | R_PROBLEM_WAVE,
        false
    },
    {
        "prb-potential",
        "Flow through porous media",
        "Flow through porous media.",
        R_PROBLEM_POTENTIAL,
        R_PROBLEM_NONE,
        R_PROBLEM_FLUID | R_PROBLEM_STRESS_MODAL,
        true
    },
    {
        "prb-wave",
        "Wave - general",
        "Solve general wave equation for wave displacements.",
        R_PROBLEM_WAVE,
        R_PROBLEM_NONE,
        R_PROBLEM_ACOUSTICS | R_PROBLEM_STRESS_MODAL,
        true
    },
    {
        "prb-mesh",
        "Mesh reconstruction",
        "Reconstruct mesh based on provided criteria.",
        R_PROBLEM_MESH,
        R_PROBLEM_NONE,
        R_PROBLEM_NONE,
        false
    },
    {
        "prb-none",
        "",
        "",
        R_PROBLEM_NONE,
        R_PROBLEM_NONE,
        R_PROBLEM_NONE,
        false
    }
};

RProblem::RProblem()
{
    this->_init();
} /* RProblem::RProblem */


RProblem::RProblem(const RProblem &problem)
{
    this->_init(&problem);
} /* RProblem::RProblem (copy) */


RProblem::~RProblem()
{
} /* RProblem::~RProblem */


void RProblem::_init(const RProblem *pProblem)
{
    this->matrixSolverConfs.insert(RMatrixSolverConf::CG,RMatrixSolverConf(RMatrixSolverConf::CG));
    this->matrixSolverConfs.insert(RMatrixSolverConf::GMRES,RMatrixSolverConf(RMatrixSolverConf::GMRES));
    if (pProblem)
    {
        this->taskTree = pProblem->taskTree;
        this->timeSolver = pProblem->timeSolver;
        this->matrixSolverConfs = pProblem->matrixSolverConfs;
        this->monitoringPointManager = pProblem->monitoringPointManager;
        this->problemSetup = pProblem->problemSetup;
    }
} /* RProblem::_init */


const RProblemTaskItem &RProblem::getProblemTaskTree() const
{
    return this->taskTree;
} /* RProblem::getProblemTaskTree */


RProblemTaskItem &RProblem::getProblemTaskTree()
{
    return this->taskTree;
} /* RProblem::getProblemTaskTree */


void RProblem::setProblemTaskTree(const RProblemTaskItem &taskTree)
{
    this->taskTree = taskTree;
} /* RProblem::setProblemTaskTree */


const RTimeSolver &RProblem::getTimeSolver() const
{
    return this->timeSolver;
} /* RProblem::getTimeSolver */


RTimeSolver &RProblem::getTimeSolver()
{
    return this->timeSolver;
} /* RProblem::getTimeSolver */


void RProblem::setTimeSolver(const RTimeSolver &timeSolver)
{
    this->timeSolver = timeSolver;
} /* RProblem::setTimeSolver */


RMatrixSolverConf RProblem::getMatrixSolverConf(RMatrixSolverType matrixSolverType) const
{
    if (!this->matrixSolverConfs.contains(matrixSolverType))
    {
        throw RError(RError::Type::Application,R_ERROR_REF,"Invalid matrix solver type \'%d\'",matrixSolverType);
    }
    return this->matrixSolverConfs[matrixSolverType];
} /* RProblem::getMatrixSolver */


RMatrixSolverConf &RProblem::getMatrixSolverConf(RMatrixSolverType matrixSolverType)
{
    if (!this->matrixSolverConfs.contains(matrixSolverType))
    {
        throw RError(RError::Type::Application,R_ERROR_REF,"Invalid matrix solver type \'%d\'",matrixSolverType);
    }
    this->matrixSolverConfs[matrixSolverType].setType(matrixSolverType);
    return this->matrixSolverConfs[matrixSolverType];
} /* RProblem::getMatrixSolver */


const RMonitoringPointManager &RProblem::getMonitoringPointManager() const
{
    return this->monitoringPointManager;
} /* RProblem::getMonitoringPointManager */


RMonitoringPointManager &RProblem::getMonitoringPointManager()
{
    return this->monitoringPointManager;
} /* RProblem::getMonitoringPointManager */


void RProblem::setMonitoringPointManager(const RMonitoringPointManager &monitoringPointManager)
{
    this->monitoringPointManager = monitoringPointManager;
} /* RProblem::setMonitoringPointManager */


const RProblemSetup &RProblem::getProblemSetup() const
{
    return this->problemSetup;
} /* RProblem::getProblemSetup */


RProblemSetup &RProblem::getProblemSetup()
{
    return this->problemSetup;
} /* RProblem::getProblemSetup */


void RProblem::setProblemSetup(const RProblemSetup &problemSetup)
{
    this->problemSetup = problemSetup;
} /* RProblem::setProblemSetup */


RProblem &RProblem::operator = (const RProblem &problem)
{
    this->_init(&problem);
    return (*this);
} /* RProblem::operator */


RProblemType RProblem::getTypeFromId(const QString &problemId)
{
    RProblemType problemType(R_PROBLEM_NONE);


    unsigned int i = 0;
    while (problemDesc[i].type != R_PROBLEM_NONE)
    {
        if (problemDesc[i].id == problemId)
        {
            problemType = problemDesc[i].type;
            break;
        }
        i++;
    }

    return problemType;
} /* RProblem::getTypeFromId */


QString RProblem::getId(RProblemType type)
{
    R_ERROR_ASSERT (R_PROBLEM_TYPE_IS_VALID (type));
    QString id;
    unsigned int i = 0;
    while (problemDesc[i].type != R_PROBLEM_NONE)
    {
        if (problemDesc[i].type == type)
        {
            id = problemDesc[i].id;
            break;
        }
        i++;
    }
    return id;
} /* RProblem::getId */


QString RProblem::getName(RProblemType type)
{
    R_ERROR_ASSERT (R_PROBLEM_TYPE_IS_VALID (type));
    QString name;
    unsigned int i = 0;
    while (problemDesc[i].type != R_PROBLEM_NONE)
    {
        if (problemDesc[i].type == type)
        {
            name = problemDesc[i].name;
            break;
        }
        i++;
    }
    return name;
} /* RProblem::getName */


QString RProblem::getDesc(RProblemType type)
{
    R_ERROR_ASSERT (R_PROBLEM_TYPE_IS_VALID (type));

    QString desc;
    unsigned int i = 0;
    while (problemDesc[i].type != R_PROBLEM_NONE)
    {
        if (problemDesc[i].type == type)
        {
            desc = problemDesc[i].desc;
            break;
        }
        i++;
    }
    return desc;
} /* RProblem::getDesc */


bool RProblem::getTimeSolverEnabled(RProblemTypeMask typeMask)
{
    unsigned int i = 0;

    if (typeMask == R_PROBLEM_MESH)
    {
        return false;
    }

    while (problemDesc[i].type != R_PROBLEM_NONE)
    {
        if (problemDesc[i].type &typeMask &&problemDesc[i].type != R_PROBLEM_MESH)
        {
            if (!problemDesc[i].timeSolverEnabled)
            {
                return false;
            }
        }
        i++;
    }
    return true;
} /* RProblemMask::getTimeSolverTypeMask */


RProblemTypeMask RProblem::getRequiredProblemTypeMask(RProblemTypeMask typeMask)
{
    RProblemTypeMask reqTypeMask = R_PROBLEM_NONE;
    unsigned int i = 0;
    while (problemDesc[i].type != R_PROBLEM_NONE)
    {
        if (problemDesc[i].type &typeMask)
        {
            reqTypeMask |= problemDesc[i].requiredProblemTypeMask;
        }
        i++;
    }
    return reqTypeMask;
} /* RProblemMask::getRequiredProblemTypeMask */


RProblemTypeMask RProblem::getExcludedProblemTypeMask(RProblemTypeMask typeMask)
{
    RProblemTypeMask excTypeMask = R_PROBLEM_NONE;
    unsigned int i = 0;
    while (problemDesc[i].type != R_PROBLEM_NONE)
    {
        if (problemDesc[i].type &typeMask)
        {
            excTypeMask |= problemDesc[i].excludedProblemTypeMask;
        }
        i++;
    }
    return excTypeMask;
} /* RProblemMask::getTimeSolverTypeMask */


std::vector<RProblemType> RProblem::getTypes(RProblemTypeMask typeMask)
{
    R_ERROR_ASSERT (R_PROBLEM_TYPE_MASK_IS_VALID (typeMask));

    std::vector<RProblemType> types;
    unsigned int i = 0;
    while (problemDesc[i].type != R_PROBLEM_NONE)
    {
        if (problemDesc[i].type &typeMask)
        {
            types.push_back (problemDesc[i].type);
        }
        i++;
    }

    return types;
} /* RProblemMask::getTypes */


std::vector<RVariableType> RProblem::getVariableTypes(RProblemTypeMask typeMask)
{
    std::vector<RVariableType> variableTypes;
    std::vector<RProblemType> problemTypes = RProblem::getTypes(typeMask);

    for (int i=R_VARIABLE_NONE;i<R_VARIABLE_N_TYPES;i++)
    {
        RProblemTypeMask variableProblemTypeMask = RVariable::getResultsProblemTypeMask(RVariableType(i));
        for (unsigned int j=0;j<problemTypes.size();j++)
        {
            if (variableProblemTypeMask &problemTypes[j])
            {
                variableTypes.push_back(RVariableType(i));
                break;
            }
        }
    }

    return variableTypes;
} /* RProblemMask::getVariableTypes */

void RProblem::sortTypesByDependency(std::vector<RProblemType> &problemTypes)
{
    if (problemTypes.size() == 0)
    {
        return;
    }

    bool orderChanged = false;

    do
    {
        orderChanged = false;
        for (uint i=0;i<problemTypes.size()-1;i++)
        {
            RProblemTypeMask requiredProblemTypes = RProblem::getRequiredProblemTypeMask(problemTypes[i]);
            for (uint j=i+1;j<problemTypes.size();j++)
            {
                if (requiredProblemTypes &problemTypes[j])
                {
                    problemTypes.insert(problemTypes.begin()+i,problemTypes.at(j));
                    problemTypes.erase(problemTypes.begin()+j+1);
                    orderChanged = true;
                    break;
                }
            }
            if (orderChanged)
            {
                break;
            }
        }
    } while (orderChanged);
} /* RProblemMask::sortTypesByDependency */
