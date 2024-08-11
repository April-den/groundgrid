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

        void setQuadrant(Quadrant type)
        {
            quadrant_ = type;
        };

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

        inline bool getVectorToOrigin(Vector &vectorToOrigin, const Length &mapLength)
        {
            vectorToOrigin = (0.5 * mapLength).matrix();
            return true;
        }

        inline bool checkIfStartIndexAtDefaultPosition(const Index &bufferStartIndex)
        {
            return ((bufferStartIndex == 0).all());
        }

        void wrapIndexToRange(Index &index, const Size &bufferSize)
        {
            for (int i = 0; i < index.size(); i++)
            {
                wrapIndexToRange(index[i], bufferSize[i]);
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

        inline Eigen::Matrix2i getBufferOrderToMapFrameTransformation()
        {
            return -Eigen::Matrix2i::Identity();
        }

        bool checkIfPositionWithinMap(const Position &position,
                                      const Length &mapLength,
                                      const Position &mapPosition)
        {
            Vector offset;
            getVectorToOrigin(offset, mapLength);
            Position positionTransformed = getBufferOrderToMapFrameTransformation().transpose().cast<double>() * (position - mapPosition - offset);

            return positionTransformed.x() >= 0.0 && positionTransformed.y() >= 0.0 && positionTransformed.x() < mapLength(0) && positionTransformed.y() < mapLength(1);
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

        bool getIndex(const Position &position, Index &index)
        {
            return getIndexFromPosition(index, position, length_, position_, resolution_, size_, startIndex_);
        }

        const Size &getSize() const
        {
            return size_;
        }

        inline Index transformMapFrameToBufferOrder(const Vector &vector)
        {
            return {-vector[0], -vector[1]};
        }

        inline Index transformMapFrameToBufferOrder(const Eigen::Vector2i& vector) {
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