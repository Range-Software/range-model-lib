#include <rbl_logger.h>

#include "rml_problem.h"

const double RProblemTaskItem::defaultCvgValue = 1.0e-5;

void RProblemTaskItem::_init(const RProblemTaskItem *pSolverTaskItem)
{
    if (pSolverTaskItem)
    {
        this->problemType = pSolverTaskItem->problemType;
        this->nIterations = pSolverTaskItem->nIterations;
        this->cvgValue = pSolverTaskItem->cvgValue;
        this->children = pSolverTaskItem->children;
    }
}

RProblemTaskItem::RProblemTaskItem(RProblemType problemType)
    : problemType(problemType)
    , nIterations(1)
    , cvgValue(RProblemTaskItem::defaultCvgValue)
{
    this->_init();
}

RProblemTaskItem::RProblemTaskItem(const RProblemTaskItem &solverTaskItem)
{
    this->_init(&solverTaskItem);
}

RProblemTaskItem::~RProblemTaskItem()
{

}

RProblemTaskItem &RProblemTaskItem::operator =(const RProblemTaskItem &solverTaskItem)
{
    this->_init(&solverTaskItem);
    return (*this);
}

RProblemTypeMask RProblemTaskItem::getProblemTypeMask(void) const
{
    RProblemTypeMask problemTypeMask = this->getProblemType();

    for (uint i=0;i<this->getNChildren();i++)
    {
        problemTypeMask |= this->getChild(i).getProblemTypeMask();
    }

    return problemTypeMask;
}

RProblemType RProblemTaskItem::getProblemType(void) const
{
    return this->problemType;
}

void RProblemTaskItem::setProblemType(RProblemType problemType)
{
    this->problemType = problemType;
    if (this->problemType != R_PROBLEM_NONE)
    {
        this->nIterations = 0;
        this->cvgValue = 0.0;
        this->children.resize(0);
    }
}

unsigned int RProblemTaskItem::getNIterations(void) const
{
    return this->nIterations;
}

void RProblemTaskItem::setNIterations(unsigned int nIterations)
{
    this->nIterations = nIterations;
}

double RProblemTaskItem::getCvgValue(void) const
{
    return this->cvgValue;
}

void RProblemTaskItem::setCvgValue(double cvgValue)
{
    this->cvgValue = cvgValue;
}

unsigned int RProblemTaskItem::getNChildren(void) const
{
    return (unsigned int)this->children.size();
}

const RProblemTaskItem &RProblemTaskItem::getChild(unsigned int position) const
{
    return this->children.at(position);
}

RProblemTaskItem &RProblemTaskItem::getChild(unsigned int position)
{
    return this->children.at(position);
}

void RProblemTaskItem::addChild(const RProblemTaskItem &solverTaskItem)
{
    this->children.push_back(solverTaskItem);
}

void RProblemTaskItem::insertChild(unsigned int position, const RProblemTaskItem &solverTaskItem)
{
    this->children.insert(this->children.begin()+position,1,solverTaskItem);
}

void RProblemTaskItem::removeChild(unsigned int position)
{
    this->children.erase(this->children.begin()+position);
}

void RProblemTaskItem::print(bool printTitle) const
{
    if (printTitle)
    {
        RLogger::info("Task-tree to be executed:\n");
        RLogger::indent();
    }
    if (this->getProblemType() == R_PROBLEM_NONE)
    {
        RLogger::info("Number of iterations: %u\n",this->getNIterations());
        if (this->getCvgValue() > 0.0)
        {
            RLogger::info("Convergence value: %g\n",this->getCvgValue());
        }
        RLogger::indent();
        for (unsigned int i=0;i<this->getNChildren();i++)
        {
            this->getChild(i).print(false);
        }
        RLogger::unindent(false);
    }
    else
    {
        RLogger::info("Problem type: %s\n",RProblem::getName(this->getProblemType()).toUtf8().constData());
    }
    if (printTitle)
    {
        RLogger::unindent(false);
    }
}

bool RProblemTaskItem::hasDefaultLayout(void) const
{
    if (this->problemType != R_PROBLEM_NONE)
    {
        return false;
    }

    const RProblemTypeMask problemTypeMask = this->getProblemTypeMask();

    // Problem types which can not be selected would be lost.
    if (problemTypeMask & ~R_PROBLEM_ALL)
    {
        return false;
    }

    if (!RProblemTaskItem::getNeedsIterations(problemTypeMask) && this->nIterations != 1)
    {
        return false;
    }

    std::vector<RProblemType> problemTypes = RProblem::getTypes(problemTypeMask);
    RProblem::sortTypesByDependency(problemTypes);

    if (this->children.size() != problemTypes.size())
    {
        return false;
    }

    for (std::size_t i=0;i<problemTypes.size();i++)
    {
        if (this->children[i].problemType != problemTypes[i])
        {
            return false;
        }
    }

    return true;
}

bool RProblemTaskItem::getNeedsIterations(RProblemTypeMask problemTypeMask)
{
    return (problemTypeMask & (R_PROBLEM_FLUID | R_PROBLEM_RADIATIVE_HEAT)) != R_PROBLEM_NONE;
}

unsigned int RProblemTaskItem::getDefaultNIterations(RProblemTypeMask problemTypeMask, bool transient)
{
    if (problemTypeMask & R_PROBLEM_FLUID)
    {
        return transient ? 20 : 2000;
    }
    if (problemTypeMask & R_PROBLEM_RADIATIVE_HEAT)
    {
        return 5;
    }
    return 1;
}

RProblemTaskItem RProblemTaskItem::generateDefaultTree(RProblemTypeMask problemTypeMask, bool transient)
{
    RProblemTaskItem root;

    std::vector<RProblemType> problemTypes = RProblem::getTypes(problemTypeMask);
    RProblem::sortTypesByDependency(problemTypes);

    for (RProblemType problemType : problemTypes)
    {
        root.addChild(RProblemTaskItem(problemType));
    }

    root.setNIterations(RProblemTaskItem::getDefaultNIterations(problemTypeMask,transient));

    return root;
}
