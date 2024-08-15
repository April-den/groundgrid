#pragma once
#include "Eigen/Eigen"

// for std::unordered_map()
#include <unordered_map>

// for std::vector
#include <vector>

#include <memory>

// for std::move()
#include <utility>
#include <cassert>

namespace grid_map
{
    using Matrix = Eigen::MatrixXf;
    using DataType = Matrix::Scalar;
    using Position = Eigen::Vector2d;
    using Vector = Eigen::Vector2d;
    using Position3 = Eigen::Vector3d;
    using Vector3 = Eigen::Vector3d;
    using Index = Eigen::Array2i;
    using Size = Eigen::Array2i;
    using Length = Eigen::Array2d;
    using Time = uint64_t;

    bool checkIfIndexInRange(const Index &index, const Size &bufferSize)
    {
        return index[0] >= 0 && index[1] >= 0 && index[0] < bufferSize[0] && index[1] < bufferSize[1];
    }

    inline bool checkIfStartIndexAtDefaultPosition(const Index &bufferStartIndex)
    {
        return ((bufferStartIndex == 0).all());
    }

    void wrapIndexToRange(Index &index, const Size &bufferSize)
    {
        for (int i = 0; i < index.size(); i++)
        {
            // Try shortcuts before resorting to the expensive modulo operation.
            if (index[i] < bufferSize[i])
            {
                if (index[i] >= 0)
                { // within the wanted range
                    return;
                }
                else if (index[i] >= -bufferSize[i])
                { // Index is below range, but not more than one span of the range.
                    index[i] += bufferSize[i];
                    return;
                }
                else
                { // Index is largely below range.
                    index[i] = index[i] % bufferSize[i];
                    index[i] += bufferSize[i];
                }
            }
            else if (index[i] < bufferSize[i] * 2)
            { // Index is above range, but not more than one span of the range.
                index[i] -= bufferSize[i];
                return;
            }
            else
            { // Index is largely above range.
                index[i] = index[i] % bufferSize[i];
            }
        }
    }

    void wrapIndexToRange(int &index, int bufferSize)
    {
        // Try shortcuts before resorting to the expensive modulo operation.
        if (index < bufferSize)
        {
            if (index >= 0)
            { // within the wanted range
                return;
            }
            else if (index >= -bufferSize)
            { // Index is below range, but not more than one span of the range.
                index += bufferSize;
                return;
            }
            else
            { // Index is largely below range.
                index = index % bufferSize;
                index += bufferSize;
            }
        }
        else if (index < bufferSize * 2)
        { // Index is above range, but not more than one span of the range.
            index -= bufferSize;
            return;
        }
        else
        { // Index is largely above range.
            index = index % bufferSize;
        }
    }

    Index getIndexFromBufferIndex(const Index &bufferIndex, const Size &bufferSize, const Index &bufferStartIndex)
    {
        if (checkIfStartIndexAtDefaultPosition(bufferStartIndex))
        {
            return bufferIndex;
        }

        Index index = bufferIndex - bufferStartIndex;
        wrapIndexToRange(index, bufferSize);
        return index;
    }

    Index getBufferIndexFromIndex(const Index &index, const Size &bufferSize, const Index &bufferStartIndex)
    {
        if (checkIfStartIndexAtDefaultPosition(bufferStartIndex))
        {
            return index;
        }

        Index bufferIndex = index + bufferStartIndex;
        wrapIndexToRange(bufferIndex, bufferSize);
        return bufferIndex;
    }

    bool incrementIndexForSubmap(Index &submapIndex, Index &index, const Index &submapTopLeftIndex,
                                 const Size &submapBufferSize, const Size &bufferSize,
                                 const Index &bufferStartIndex)
    {
        // Copy the data first, only copy it back if everything is within range.
        Index tempIndex = index;
        Index tempSubmapIndex = submapIndex;

        // Increment submap index.
        if (tempSubmapIndex[1] + 1 < submapBufferSize[1])
        {
            // Same row.
            tempSubmapIndex[1]++;
        }
        else
        {
            // Next row.
            tempSubmapIndex[0]++;
            tempSubmapIndex[1] = 0;
        }

        // End of iterations reached.
        if (!checkIfIndexInRange(tempSubmapIndex, submapBufferSize))
        {
            return false;
        }

        // Get corresponding index in map.
        Index unwrappedSubmapTopLeftIndex = getIndexFromBufferIndex(submapTopLeftIndex, bufferSize, bufferStartIndex);
        tempIndex = getBufferIndexFromIndex(unwrappedSubmapTopLeftIndex + tempSubmapIndex, bufferSize, bufferStartIndex);

        // Copy data back.
        index = tempIndex;
        submapIndex = tempSubmapIndex;
        return true;
    }

    bool checkIfPositionWithinMap(const Position &position,
                                  const Length &mapLength,
                                  const Position &mapPosition)
    {
        Vector offset;
        getVectorToOrigin(offset, mapLength);
        Position positionTransformed = getMapFrameToBufferOrderTransformation().cast<double>() * (position - mapPosition - offset);

        return positionTransformed.x() >= 0.0 && positionTransformed.y() >= 0.0 && positionTransformed.x() < mapLength(0) && positionTransformed.y() < mapLength(1);
    }

    inline bool getVectorToOrigin(Vector &vectorToOrigin, const Length &mapLength)
    {
        vectorToOrigin = (0.5 * mapLength).matrix();
        return true;
    }

    inline Eigen::Matrix2i getMapFrameToBufferOrderTransformation()
    {
        return getBufferOrderToMapFrameTransformation().transpose();
    }

    inline Eigen::Matrix2i getBufferOrderToMapFrameTransformation()
    {
        return -Eigen::Matrix2i::Identity();
    }

    class BufferRegion
    {
    public:
        /*!
         * The definition of the buffer region positions.
         */
        enum class Quadrant
        {
            Undefined,
            TopLeft,
            TopRight,
            BottomLeft,
            BottomRight
        };

        constexpr static unsigned int nQuadrants = 4;

        BufferRegion() : startIndex_(Index::Zero()), size_(Size::Zero()),
                         quadrant_(BufferRegion::Quadrant::Undefined) {};

        BufferRegion(Index startIndex, Size size, BufferRegion::Quadrant quadrant) : startIndex_(std::move(startIndex)),
                                                                                     size_(std::move(size)), quadrant_(std::move(quadrant)) {}

        virtual ~BufferRegion() = default;

        const Index &getStartIndex() const
        {
            return startIndex_;
        };

        void setStartIndex(const Index &startIndex)
        {
            startIndex_ = startIndex;
        };

        const Size &getSize() const
        {
            return size_;
        };

        void setSize(const Size &size)
        {
            size_ = size;
        }

        Quadrant getQuadrant() const
        {
            return quadrant_;
        };

        inline Quadrant getQuadrant(const Index &index, const Index &bufferStartIndex)
        {
            if (index[0] >= bufferStartIndex[0] && index[1] >= bufferStartIndex[1])
            {
                return Quadrant::TopLeft;
            }
            if (index[0] >= bufferStartIndex[0] && index[1] < bufferStartIndex[1])
            {
                return Quadrant::TopRight;
            }
            if (index[0] < bufferStartIndex[0] && index[1] >= bufferStartIndex[1])
            {
                return Quadrant::BottomLeft;
            }
            if (index[0] < bufferStartIndex[0] && index[1] < bufferStartIndex[1])
            {
                return Quadrant::BottomRight;
            }
            return Quadrant::Undefined;
        }

        void setQuadrant(Quadrant type)
        {
            quadrant_ = type;
        };

        bool getBufferRegionsForSubmap(std::vector<BufferRegion> &submapBufferRegions,
                                       const Index &submapIndex,
                                       const Size &submapBufferSize,
                                       const Size &bufferSize,
                                       const Index &bufferStartIndex)
        {
            if ((getIndexFromBufferIndex(submapIndex, bufferSize, bufferStartIndex) + submapBufferSize > bufferSize).any())
            {
                return false;
            }

            submapBufferRegions.clear();

            Index bottomRightIndex = submapIndex + submapBufferSize - Index::Ones();
            wrapIndexToRange(bottomRightIndex, bufferSize);

            Quadrant quadrantOfTopLeft = getQuadrant(submapIndex, bufferStartIndex);
            Quadrant quadrantOfBottomRight = getQuadrant(bottomRightIndex, bufferStartIndex);

            if (quadrantOfTopLeft == Quadrant::TopLeft)
            {

                if (quadrantOfBottomRight == Quadrant::TopLeft)
                {
                    submapBufferRegions.emplace_back(submapIndex, submapBufferSize, Quadrant::TopLeft);
                    return true;
                }

                if (quadrantOfBottomRight == Quadrant::TopRight)
                {
                    Size topLeftSize(submapBufferSize(0), bufferSize(1) - submapIndex(1));
                    submapBufferRegions.emplace_back(submapIndex, topLeftSize, Quadrant::TopLeft);

                    Index topRightIndex(submapIndex(0), 0);
                    Size topRightSize(submapBufferSize(0), submapBufferSize(1) - topLeftSize(1));
                    submapBufferRegions.emplace_back(topRightIndex, topRightSize, Quadrant::TopRight);
                    return true;
                }

                if (quadrantOfBottomRight == Quadrant::BottomLeft)
                {
                    Size topLeftSize(bufferSize(0) - submapIndex(0), submapBufferSize(1));
                    submapBufferRegions.emplace_back(submapIndex, topLeftSize, Quadrant::TopLeft);

                    Index bottomLeftIndex(0, submapIndex(1));
                    Size bottomLeftSize(submapBufferSize(0) - topLeftSize(0), submapBufferSize(1));
                    submapBufferRegions.emplace_back(bottomLeftIndex, bottomLeftSize, Quadrant::BottomLeft);
                    return true;
                }

                if (quadrantOfBottomRight == Quadrant::BottomRight)
                {
                    Size topLeftSize(bufferSize(0) - submapIndex(0), bufferSize(1) - submapIndex(1));
                    submapBufferRegions.emplace_back(submapIndex, topLeftSize, Quadrant::TopLeft);

                    Index topRightIndex(submapIndex(0), 0);
                    Size topRightSize(bufferSize(0) - submapIndex(0), submapBufferSize(1) - topLeftSize(1));
                    submapBufferRegions.emplace_back(topRightIndex, topRightSize, Quadrant::TopRight);

                    Index bottomLeftIndex(0, submapIndex(1));
                    Size bottomLeftSize(submapBufferSize(0) - topLeftSize(0), bufferSize(1) - submapIndex(1));
                    submapBufferRegions.emplace_back(bottomLeftIndex, bottomLeftSize, Quadrant::BottomLeft);

                    Index bottomRightIndex = Index::Zero();
                    Size bottomRightSize(bottomLeftSize(0), topRightSize(1));
                    submapBufferRegions.emplace_back(bottomRightIndex, bottomRightSize, Quadrant::BottomRight);
                    return true;
                }
            }
            else if (quadrantOfTopLeft == Quadrant::TopRight)
            {

                if (quadrantOfBottomRight == Quadrant::TopRight)
                {
                    submapBufferRegions.emplace_back(submapIndex, submapBufferSize, Quadrant::TopRight);
                    return true;
                }

                if (quadrantOfBottomRight == Quadrant::BottomRight)
                {

                    Size topRightSize(bufferSize(0) - submapIndex(0), submapBufferSize(1));
                    submapBufferRegions.emplace_back(submapIndex, topRightSize, Quadrant::TopRight);

                    Index bottomRightIndex(0, submapIndex(1));
                    Size bottomRightSize(submapBufferSize(0) - topRightSize(0), submapBufferSize(1));
                    submapBufferRegions.emplace_back(bottomRightIndex, bottomRightSize, Quadrant::BottomRight);
                    return true;
                }
            }
            else if (quadrantOfTopLeft == Quadrant::BottomLeft)
            {

                if (quadrantOfBottomRight == Quadrant::BottomLeft)
                {
                    submapBufferRegions.emplace_back(submapIndex, submapBufferSize, Quadrant::BottomLeft);
                    return true;
                }

                if (quadrantOfBottomRight == Quadrant::BottomRight)
                {
                    Size bottomLeftSize(submapBufferSize(0), bufferSize(1) - submapIndex(1));
                    submapBufferRegions.emplace_back(submapIndex, bottomLeftSize, Quadrant::BottomLeft);

                    Index bottomRightIndex(submapIndex(0), 0);
                    Size bottomRightSize(submapBufferSize(0), submapBufferSize(1) - bottomLeftSize(1));
                    submapBufferRegions.emplace_back(bottomRightIndex, bottomRightSize, Quadrant::BottomRight);
                    return true;
                }
            }
            else if (quadrantOfTopLeft == Quadrant::BottomRight)
            {

                if (quadrantOfBottomRight == Quadrant::BottomRight)
                {
                    submapBufferRegions.emplace_back(submapIndex, submapBufferSize, Quadrant::BottomRight);
                    return true;
                }
            }

            return false;
        }

    private:
        //! Start index (typically top-left) of the buffer region.
        Index startIndex_;

        //! Size of the buffer region.
        Size size_;

        //! Quadrant type of the buffer region.
        Quadrant quadrant_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    class GridMap
    {
    public:
        typedef grid_map::DataType DataType;
        typedef grid_map::Matrix Matrix;

        GridMap(const std::vector<std::string> &layers)
        {
            position_.setZero();
            length_.setZero();
            resolution_ = 0.0;
            size_.setZero();
            startIndex_.setZero();
            timestamp_ = 0;
            layers_ = layers;

            for (auto &layer : layers_)
            {
                data_.insert(std::pair<std::string, Matrix>(layer, Matrix()));
            }
        };
        GridMap();
        GridMap(const GridMap &) = default;
        GridMap &operator=(const GridMap &) = default;
        GridMap(GridMap &&) = default;
        GridMap &operator=(GridMap &&) = default;

        virtual ~GridMap() = default;

        const Matrix &get(const std::string &layer) const
        {
            try
            {
                return data_.at(layer);
            }
            catch (const std::out_of_range &exception)
            {
                throw std::out_of_range("GridMap::get(...) : No map layer '" + layer + "' available.");
            }
        }

        Matrix &get(const std::string &layer)
        {
            try
            {
                return data_.at(layer);
            }
            catch (const std::out_of_range &exception)
            {
                throw std::out_of_range("GridMap::get(...) : No map layer of type '" + layer + "' available.");
            }
        }

        const Matrix &operator[](const std::string &layer) const
        {
            return get(layer);
        }

        Matrix &operator[](const std::string &layer)
        {
            return get(layer);
        }

        void setFrameId(const std::string &frameId)
        {
            frameId_ = frameId;
        }

        const Index &getStartIndex() const
        {
            return startIndex_;
        }

        void resize(const Index &size)
        {
            size_ = size;
            for (auto &data : data_)
            {
                data.second.resize(size_(0), size_(1));
            }
        }

        void clearAll()
        {
            for (auto &data : data_)
            {
                data.second.setConstant(NAN);
            }
        }
        void setGeometry(const Length &length, const double resolution, const Position &position)
        {
            assert(length(0) > 0.0);
            assert(length(1) > 0.0);
            assert(resolution > 0.0);
            Size size;
            size(0) = static_cast<int>(round(length(0) / resolution)); // There is no round() function in Eigen.
            size(1) = static_cast<int>(round(length(1) / resolution));
            resize(size);
            clearAll();

            resolution_ = resolution;
            length_ = (size_.cast<double>() * resolution_).matrix();
            position_ = position;
            startIndex_.setZero();
        }

        void GridMap::add(const std::string &layer, const double value)
        {
            add(layer, Matrix::Constant(size_(0), size_(1), value));
        }

        void GridMap::add(const std::string &layer, const Matrix &data)
        {
            assert(size_(0) == data.rows());
            assert(size_(1) == data.cols());

            if (exists(layer))
            {
                // Type exists already, overwrite its data.
                data_.at(layer) = data;
            }
            else
            {
                // Type does not exist yet, add type and data.
                data_.insert(std::pair<std::string, Matrix>(layer, data));
                layers_.push_back(layer);
            }
        }

        bool exists(const std::string &layer) const
        {
            return !(data_.find(layer) == data_.end());
        }

        bool isInside(const Position &position) const
        {
            return checkIfPositionWithinMap(position, length_, position_);
        }

        inline bool checkIfStartIndexAtDefaultPosition(const Index &bufferStartIndex)
        {
            return ((bufferStartIndex == 0).all());
        }

        inline Eigen::Matrix2i getBufferOrderToMapFrameTransformation()
        {
            return -Eigen::Matrix2i::Identity();
        }

        bool checkIfIndexInRange(const Index &index, const Size &bufferSize)
        {
            return index[0] >= 0 && index[1] >= 0 && index[0] < bufferSize[0] && index[1] < bufferSize[1];
        }

        bool getIndexFromPosition(Index &index,
                                  const Position &position,
                                  const Length &mapLength,
                                  const Position &mapPosition,
                                  const double &resolution,
                                  const Size &bufferSize,
                                  const Index &bufferStartIndex)
        {
            Vector offset;
            getVectorToOrigin(offset, mapLength);
            Vector indexVector = ((position - offset - mapPosition).array() / resolution).matrix();
            Index indexTem = {-indexVector[0], -indexVector[1]};
            index = getBufferIndexFromIndex(indexTem, size_, startIndex_);
            return checkIfPositionWithinMap(position, mapLength, mapPosition) && checkIfIndexInRange(index, bufferSize);
        }

        double GridMap::getResolution() const
        {
            return resolution_;
        }

        bool getIndex(const Position &position, Index &index)
        {
            return getIndexFromPosition(index, position, length_, position_, resolution_, size_, startIndex_);
        }

        inline bool getVectorToFirstCell(Vector &vectorToFirstCell,
                                         const Length &mapLength, const double &resolution)
        {
            Vector vectorToOrigin;
            getVectorToOrigin(vectorToOrigin, mapLength);

            // Vector to center of cell.
            vectorToFirstCell = (vectorToOrigin.array() - 0.5 * resolution).matrix();
            return true;
        }

        Index getIndexFromBufferIndex(const Index &bufferIndex, const Size &bufferSize, const Index &bufferStartIndex)
        {
            if (checkIfStartIndexAtDefaultPosition(bufferStartIndex))
            {
                return bufferIndex;
            }

            Index index = bufferIndex - bufferStartIndex;
            wrapIndexToRange(index, bufferSize);
            return index;
        }

        inline Vector getIndexVectorFromIndex(
            const Index &index,
            const Size &bufferSize,
            const Index &bufferStartIndex)
        {
            Index unwrappedIndex;
            unwrappedIndex = getIndexFromBufferIndex(index, bufferSize, bufferStartIndex);
            return transformBufferOrderToMapFrame(unwrappedIndex);
        }

        bool getPositionFromIndex(Position &position,
                                  const Index &index,
                                  const Length &mapLength,
                                  const Position &mapPosition,
                                  const double &resolution,
                                  const Size &bufferSize,
                                  const Index &bufferStartIndex)
        {
            if (!checkIfIndexInRange(index, bufferSize))
            {
                return false;
            }
            Vector offset;
            getVectorToFirstCell(offset, mapLength, resolution);
            position = mapPosition + offset + resolution * getIndexVectorFromIndex(index, bufferSize, bufferStartIndex);
            return true;
        }

        bool getPosition(const Index &index, Position &position)
        {
            return getPositionFromIndex(position, index, length_, position_, resolution_, size_, startIndex_);
        }

        const Size &getSize() const
        {
            return size_;
        }

        inline Index transformMapFrameToBufferOrder(const Vector &vector)
        {
            return {-vector[0], -vector[1]};
        }

        inline Index transformMapFrameToBufferOrder(const Eigen::Vector2i &vector)
        {
            return {-vector[0], -vector[1]};
        }

        inline Vector transformBufferOrderToMapFrame(const Index &index)
        {
            return {-index[0], -index[1]};
        }

        bool getIndexShiftFromPositionShift(Index &indexShift,
                                            const Vector &positionShift,
                                            const double &resolution)
        {
            Vector indexShiftVectorTemp = (positionShift.array() / resolution).matrix();
            Eigen::Vector2i indexShiftVector;
            for (int i = 0; i < indexShiftVector.size(); i++)
            {
                indexShiftVector[i] = static_cast<int>(indexShiftVectorTemp[i] + 0.5 * (indexShiftVectorTemp[i] > 0 ? 1 : -1));
            }

            indexShift = transformMapFrameToBufferOrder(indexShiftVector);
            return true;
        }

        bool getPositionShiftFromIndexShift(Vector &positionShift,
                                            const Index &indexShift,
                                            const double &resolution)
        {
            positionShift = transformBufferOrderToMapFrame(indexShift) * resolution;
            return true;
        }

        void clearRows(unsigned int index, unsigned int nRows)
        {
            for (auto &layer : layers_)
            {
                data_.at(layer).block(index, 0, nRows, getSize()(1)).setConstant(NAN);
            }
        }

        void clearCols(unsigned int index, unsigned int nCols)
        {
            for (auto &layer : layers_)
            {
                data_.at(layer).block(0, index, getSize()(0), nCols).setConstant(NAN);
            }
        }

        bool move(const Position &position, std::vector<BufferRegion> &newRegions)
        {
            Index indexShift;
            Position positionShift = position - position_;
            getIndexShiftFromPositionShift(indexShift, positionShift, resolution_);
            Position alignedPositionShift;
            getPositionShiftFromIndexShift(alignedPositionShift, indexShift, resolution_);

            // Delete fields that fall out of map (and become empty cells).
            for (int i = 0; i < indexShift.size(); i++)
            {
                if (indexShift(i) != 0)
                {
                    if (abs(indexShift(i)) >= getSize()(i))
                    {
                        // Entire map is dropped.
                        clearAll();
                        newRegions.emplace_back(Index(0, 0), getSize(), BufferRegion::Quadrant::Undefined);
                    }
                    else
                    {
                        // Drop cells out of map.
                        int sign = (indexShift(i) > 0 ? 1 : -1);
                        int startIndex = startIndex_(i) - (sign < 0 ? 1 : 0);
                        int endIndex = startIndex - sign + indexShift(i);
                        int nCells = abs(indexShift(i));
                        int index = (sign > 0 ? startIndex : endIndex);
                        wrapIndexToRange(index, getSize()(i));

                        if (index + nCells <= getSize()(i))
                        {
                            // One region to drop.
                            if (i == 0)
                            {
                                clearRows(index, nCells);
                                newRegions.emplace_back(Index(index, 0), Size(nCells, getSize()(1)), BufferRegion::Quadrant::Undefined);
                            }
                            else if (i == 1)
                            {
                                clearCols(index, nCells);
                                newRegions.emplace_back(Index(0, index), Size(getSize()(0), nCells), BufferRegion::Quadrant::Undefined);
                            }
                        }
                        else
                        {
                            // Two regions to drop.
                            int firstIndex = index;
                            int firstNCells = getSize()(i) - firstIndex;
                            if (i == 0)
                            {
                                clearRows(firstIndex, firstNCells);
                                newRegions.emplace_back(Index(firstIndex, 0), Size(firstNCells, getSize()(1)), BufferRegion::Quadrant::Undefined);
                            }
                            else if (i == 1)
                            {
                                clearCols(firstIndex, firstNCells);
                                newRegions.emplace_back(Index(0, firstIndex), Size(getSize()(0), firstNCells), BufferRegion::Quadrant::Undefined);
                            }

                            int secondIndex = 0;
                            int secondNCells = nCells - firstNCells;
                            if (i == 0)
                            {
                                clearRows(secondIndex, secondNCells);
                                newRegions.emplace_back(Index(secondIndex, 0), Size(secondNCells, getSize()(1)), BufferRegion::Quadrant::Undefined);
                            }
                            else if (i == 1)
                            {
                                clearCols(secondIndex, secondNCells);
                                newRegions.emplace_back(Index(0, secondIndex), Size(getSize()(0), secondNCells), BufferRegion::Quadrant::Undefined);
                            }
                        }
                    }
                }
            }
            // Update information.
            startIndex_ += indexShift;
            wrapIndexToRange(startIndex_, getSize());
            position_ += alignedPositionShift;

            // Check if map has been moved at all.
            return indexShift.any();
        }

        float &at(const std::string &layer, const Index &index)
        {
            try
            {
                return data_.at(layer)(index(0), index(1));
            }
            catch (const std::out_of_range &exception)
            {
                throw std::out_of_range("GridMap::at(...) : No map layer '" + layer + "' available.");
            }
        }

        bool isDefaultStartIndex() const
        {
            return (startIndex_ == 0).all();
        }

        void convertToDefaultStartIndex()
        {
            if (isDefaultStartIndex())
            {
                return;
            }

            std::vector<BufferRegion> bufferRegions;
            BufferRegion tem;
            if (!tem.getBufferRegionsForSubmap(bufferRegions, startIndex_, size_, size_, startIndex_))
            {
                throw std::out_of_range("Cannot access submap of this size.");
            }

            for (auto &data : data_)
            {
                auto tempData(data.second);
                for (const auto &bufferRegion : bufferRegions)
                {
                    Index index = bufferRegion.getStartIndex();
                    Size size = bufferRegion.getSize();

                    if (bufferRegion.getQuadrant() == BufferRegion::Quadrant::TopLeft)
                    {
                        tempData.topLeftCorner(size(0), size(1)) = data.second.block(index(0), index(1), size(0), size(1));
                    }
                    else if (bufferRegion.getQuadrant() == BufferRegion::Quadrant::TopRight)
                    {
                        tempData.topRightCorner(size(0), size(1)) = data.second.block(index(0), index(1), size(0), size(1));
                    }
                    else if (bufferRegion.getQuadrant() == BufferRegion::Quadrant::BottomLeft)
                    {
                        tempData.bottomLeftCorner(size(0), size(1)) = data.second.block(index(0), index(1), size(0), size(1));
                    }
                    else if (bufferRegion.getQuadrant() == BufferRegion::Quadrant::BottomRight)
                    {
                        tempData.bottomRightCorner(size(0), size(1)) = data.second.block(index(0), index(1), size(0), size(1));
                    }
                }
                data.second = tempData;
            }

            startIndex_.setZero();
        }

    private:
        std::string frameId_;
        Time timestamp_;
        std::unordered_map<std::string, Matrix> data_;
        std::vector<std::string> layers_;
        std::vector<std::string> basicLayers_;
        Length length_;
        double resolution_;
        Position position_;
        Size size_;
        Index startIndex_;
    };

}