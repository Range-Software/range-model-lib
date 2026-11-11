#ifndef RML_SUPPORT_REACTION_H
#define RML_SUPPORT_REACTION_H

#include <QString>

#include <rbl_r3vector.h>

#include "rml_entity_group.h"

//! Total reaction force of one support - an entity which holds the structure
//! with a displacement boundary condition.
class RSupportReaction
{

    protected:

        //! Type of the supporting entity.
        REntityGroupType entityType;
        //! Name of the supporting entity.
        QString name;
        //! Total reaction force in global coordinates [N].
        RR3Vector force;

    private:

        //! Internal initialization function.
        void _init(const RSupportReaction *pSupportReaction = nullptr);

    public:

        //! Constructor.
        RSupportReaction();

        //! Constructor.
        RSupportReaction(REntityGroupType entityType, const QString &name, const RR3Vector &force);

        //! Copy constructor.
        RSupportReaction(const RSupportReaction &supportReaction);

        //! Destructor.
        ~RSupportReaction();

        //! Assignment operator.
        RSupportReaction &operator =(const RSupportReaction &supportReaction);

        //! Return type of the supporting entity.
        REntityGroupType getEntityType() const;

        //! Set type of the supporting entity.
        void setEntityType(REntityGroupType entityType);

        //! Return name of the supporting entity.
        const QString &getName() const;

        //! Set name of the supporting entity.
        void setName(const QString &name);

        //! Return total reaction force.
        const RR3Vector &getForce() const;

        //! Set total reaction force.
        void setForce(const RR3Vector &force);

};

#endif // RML_SUPPORT_REACTION_H
