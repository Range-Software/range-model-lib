#ifndef RML_MODEL_DATA_H
#define RML_MODEL_DATA_H

class RModelData
{

    private:

        //! Internal initialization function.
        void _init ( const RModelData *pModelData = nullptr );

    protected:

        //! Model is selected.
        bool selected;

    public:

        //! Constructor.
        RModelData();

        //! Copy constructor.
        RModelData(const RModelData &modelData);

        //! Destructor.
        ~RModelData();

        //! Assignment operator.
        RModelData & operator = (const RModelData &modelData);

        //! Return whether the model is selected or not.
        bool getSelected(void) const;

        //! Set whether the model is selected.
        void setSelected(bool selected);

        //! Allow RFileIO to access private members.
        friend class RFileIO;
};

#endif /* RML_MODEL_DATA_H */
