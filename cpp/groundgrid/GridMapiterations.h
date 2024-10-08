#pragma once
#include "GridMap.h"

namespace grid_map
{

  /*!
   * Iterator class to iterate through a rectangular part of the map (submap).
   * Before using this iterator, make sure that the requested submap is
   * actually contained in the grid map.
   */
  class SubmapIterator
  {
  public:
    /*!
     * Constructor.
     * @param submap the buffer region of a grid map to iterate over.
     */
    SubmapIterator(const grid_map::GridMap &gridMap,
                   const grid_map::BufferRegion &bufferRegion)
        : SubmapIterator(gridMap, bufferRegion.getStartIndex(), bufferRegion.getSize())
    {
    }
    /*!
     * Constructor.
     * @param gridMap the grid map to iterate on.
     * @param submapStartIndex the start index of the submap, typically top-left index.
     * @param submapSize the size of the submap to iterate on.
     */
    SubmapIterator(const grid_map::GridMap &gridMap, const Index &submapStartIndex,
                   const Size &submapSize)
    {
      size_ = gridMap.getSize();
      startIndex_ = gridMap.getStartIndex();
      index_ = submapStartIndex;
      submapSize_ = submapSize;
      submapStartIndex_ = submapStartIndex;
      submapIndex_.setZero();
      isPastEnd_ = false;
    }

    /*!
     * Compare to another iterator.
     * @return whether the current iterator points to a different address than the other one.
     */
    bool operator!=(const SubmapIterator &other) const
    {
      return (index_ != other.index_).any();
    }
    /*!
     * Dereference the iterator with const.
     * @return the value to which the iterator is pointing.
     */
    const Index &operator*() const
    {
      return index_;
    }

    /*!
     * Get the current index in the submap.
     * @return the current index in the submap.
     */
    const Index &getSubmapIndex() const
    {
      return submapIndex_;
    }

    /*!
     * Increase the iterator to the next element.
     * @return a reference to the updated iterator.
     */
    SubmapIterator &operator++()
    {
      isPastEnd_ = !incrementIndexForSubmap(submapIndex_, index_, submapStartIndex_,
                                            submapSize_, size_, startIndex_);
      return *this;
    }

    /*!
     * Indicates if iterator is past end.
     * @return true if iterator is out of scope, false if end has not been reached.
     */
    bool isPastEnd() const
    {
      return isPastEnd_;
    }

    /*!
     * Returns the size of the submap covered by the iterator.
     * @return the size of the submap covered by the iterator.
     */
    const Size &getSubmapSize() const
    {
      return submapSize_;
    }

  private:
    //! Size of the buffer.
    Size size_;

    //! Start index of the circular buffer.
    Index startIndex_;

    //! Current index.
    Index index_;

    //! Submap buffer size.
    Size submapSize_;

    //! Top left index of the submap.
    Index submapStartIndex_;

    //! Current index in the submap.
    Index submapIndex_;

    //! Is iterator out of scope.
    bool isPastEnd_;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace grid_map