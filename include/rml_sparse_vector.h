#ifndef RML_SPARSE_VECTOR_H
#define RML_SPARSE_VECTOR_H

#include <QtGlobal>
#include <algorithm>
#include <vector>

/*
 * Following vector:
 *
 * [ 0.0 , 1.0 , 0.0 , 2.0 , 0.0 , 3.0 ]
 *
 * will be stored as:
 *
 *            | RSparseVectorItem |
 * +----------++-------+---------++
 * | Position || Index | Value   ||
 * +----------++-------+---------++
 * | 0        || 1     | 1.0     ||
 * | 1        || 3     | 2.0     ||
 * | 2        || 5     | 3.0     ||
 * +----------++-------+---------++
 */

template <class T>
class RSparseVectorItem
{

    public:

        //! Value index position.
        uint index;
        //! Value.
        T value;

        //! Constructor.
        RSparseVectorItem(uint index, T value)
            : index(index)
            , value(value)
        {
        }

        //! Less operator.
        bool operator <(const RSparseVectorItem<T> &sparseVectorItem) const
        {
            return (this->index < sparseVectorItem.index);
        }

        //! Equals operator.
        bool operator ==(const RSparseVectorItem<T> &sparseVectorItem) const
        {
            return (this->index == sparseVectorItem.index);
        }
};

template <class T>
class RSparseVector
{

    protected:

        //! Vector of matrix rows.
        std::vector< RSparseVectorItem<T> > data;

    private:

        //! Internal initialization function.
        void _init(const RSparseVector<T> *pVector = nullptr)
        {
            if (pVector)
            {
                this->data = pVector->data;
            }
        }

    public:

        //! Constructor.
        RSparseVector()
        {
            this->_init();
        }

        //! Copy constructor.
        RSparseVector(const RSparseVector<T> &vector)
        {
            this->_init(&vector);
        }

        //! Destructor.
        ~RSparseVector()
        {
        }

        //! Assignment operator.
        RSparseVector<T> & operator =(const RSparseVector<T> &vector)
        {
            this->_init(&vector);
            return (*this);
        }

        //! Return size of the vector.
        uint size(void) const
        {
            return uint(this->data.size());
        }

        //! Return value at given position.
        const T &getValue(uint position) const
        {
            return this->data[position].value;
        }

        //! Add value at given sparse-vector storage position.
        void addValueAtPosition(uint position, T value)
        {
            this->data[position].value += value;
        }

        //! Return index at given position.
        uint getIndex(uint position) const
        {
            return this->data[position].index;
        }

        //! Return vector of position indexes.
        std::vector<uint> getIndexes(void) const
        {
            std::vector<uint> idxList;

            idxList.resize(this->data.size());

            for (uint i=0;i<this->data.size();i++)
            {
                idxList[i] = this->data[i].index;
            }

            return idxList;
        }

        //! Add value.
        //! If value with given index already exist value will be added to its current value.
        void addValue(uint index, T value)
        {
            RSparseVectorItem<T> key(index, T{});
            auto it = std::lower_bound(this->data.begin(), this->data.end(), key);
            if (it != this->data.end() && it->index == index)
            {
                it->value += value;
            }
            else
            {
                this->data.insert(it, RSparseVectorItem<T>(index, value));
            }
        }

        //! Return real sized vector of values.
        //! nElements difines minimum size of the vector.
        std::vector<T> getValues(uint nElements) const
        {
            std::vector<T> values;
            values.resize(nElements,0.0);

            for (uint i=0;i<this->data.size();i++)
            {
                if (this->data[i].index >= nElements)
                {
                    nElements = this->data[i].index + 1;
                    values.resize(nElements,0.0);
                }
                values[this->data[i].index] = this->data[i].value;
            }
            return values;
        }

        //! Vector add operation.
        void addVector(const RSparseVector<T> &v)
        {
            if (v.data.empty()) return;
            if (this->data.empty()) { this->data = v.data; return; }

            std::vector< RSparseVectorItem<T> > merged;
            merged.reserve(this->data.size() + v.data.size());
            auto it1 = this->data.cbegin();
            auto it2 = v.data.cbegin();
            while (it1 != this->data.cend() && it2 != v.data.cend())
            {
                if (it1->index < it2->index)      { merged.push_back(*it1++); }
                else if (it2->index < it1->index) { merged.push_back(*it2++); }
                else { merged.push_back(*it1++); merged.back().value += it2->value; ++it2; }
            }
            merged.insert(merged.end(), it1, this->data.cend());
            merged.insert(merged.end(), it2, v.data.cend());
            this->data = std::move(merged);
        }

        //! Add vector values assuming both sparse vectors have the same index pattern.
        void addVectorValues(const RSparseVector<T> &v)
        {
            for (uint i=0;i<this->data.size();i++)
            {
                this->data[i].value += v.data[i].value;
            }
        }

        //! Fill values while preserving sparse indexes.
        void fillValues(T value)
        {
            for (uint i=0;i<this->data.size();i++)
            {
                this->data[i].value = value;
            }
        }

        //! Find position for given index.
        bool findPosition(uint index, uint &position) const
        {
            RSparseVectorItem<T> key(index, T{});
            auto it = std::lower_bound(this->data.begin(), this->data.end(), key);
            if (it != this->data.end() && it->index == index)
            {
                position = uint(it - this->data.begin());
                return true;
            }
            return false;
        }

        //! Reserve vector size.
        void reserve(uint nElements)
        {
            this->data.reserve(nElements);
        }

        //! Clear vector.
        void clear(void)
        {
            this->data.clear();
        }

};

#endif // RML_SPARSE_VECTOR_H
