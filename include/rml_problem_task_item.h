#ifndef RML_PROBLEM_TASK_ITEM_H
#define RML_PROBLEM_TASK_ITEM_H

#include <vector>

#include "rml_problem_type.h"

class RProblemTaskItem
{

    public:

        //! Default convergence value of a newly created task group.
        static const double defaultCvgValue;

    protected:

        //! Problem type.
        RProblemType problemType;
        //! Number of children iterations.
        //! If problem type is not R_PROBLEM_NONE nIterations is ignored.
        unsigned int nIterations;
        //! Convergence value ending the children iterations early.
        //! The iterations stop as soon as every child reports a convergence
        //! below this value. Zero or less runs all of them.
        //! If problem type is not R_PROBLEM_NONE cvgValue is ignored.
        double cvgValue;
        //! Children items.
        //! If problem type is not R_PROBLEM_NONE children is ignored.
        std::vector<RProblemTaskItem> children;

    private:

        //! Internal initialization function.
        void _init(const RProblemTaskItem *pSolverTaskItem = nullptr);

    public:

        //! Constructor.
        RProblemTaskItem(RProblemType problemType = R_PROBLEM_NONE);

        //! Copy constructor.
        RProblemTaskItem(const RProblemTaskItem &solverTaskItem);

        //! Destructor.
        ~RProblemTaskItem();

        //! Assignment operator.
        RProblemTaskItem & operator =(const RProblemTaskItem &solverTaskItem);

        //! Return problem type mask.
        RProblemTypeMask getProblemTypeMask(void) const;

        //! Return problem type.
        RProblemType getProblemType(void) const;

        //! Return problem type.
        void setProblemType(RProblemType problemType);

        //! Return number of iterations.
        unsigned int getNIterations(void) const;

        //! Return number of iterations.
        void setNIterations(unsigned int nIterations);

        //! Return convergence value.
        double getCvgValue(void) const;

        //! Set convergence value.
        void setCvgValue(double cvgValue);

        //! Return number of children.
        unsigned int getNChildren(void) const;

        //! Return const reference to child at given position.
        const RProblemTaskItem &getChild(unsigned int position) const;

        //! Return reference to child at given position.
        RProblemTaskItem &getChild(unsigned int position);

        //! Add child.
        void addChild(const RProblemTaskItem &solverTaskItem);

        //! Insert child.
        void insertChild(unsigned int position, const RProblemTaskItem &solverTaskItem);

        //! Remove child.
        void removeChild(unsigned int position);

        //! Print task tree.
        void print(bool printTitle = true) const;

        //! Return true if this is a default layout of a task tree: a single group of tasks
        //! in solve order, repeated only when given problem types need iterations.
        bool hasDefaultLayout(void) const;

        //! Return true if given problem types have to be solved iteratively.
        static bool getNeedsIterations(RProblemTypeMask problemTypeMask);

        //! Return default number of iterations for given problem types.
        //! If transient is true the number is per time step.
        static unsigned int getDefaultNIterations(RProblemTypeMask problemTypeMask, bool transient);

        //! Generate default problem tree from given problem type mask.
        static RProblemTaskItem generateDefaultTree(RProblemTypeMask problemTypeMask, bool transient = false);

        //! Allow RFileIO to access private members.
        friend class RFileIO;

};

#endif // RML_PROBLEM_TASK_ITEM_H
