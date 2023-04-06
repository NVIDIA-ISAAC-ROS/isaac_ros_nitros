/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#ifndef NVIDIA_GXF_STD_GEMS_SUBALLOCATORS_FIRST_FIT_ALLOCATOR_BASE_HPP
#define NVIDIA_GXF_STD_GEMS_SUBALLOCATORS_FIRST_FIT_ALLOCATOR_BASE_HPP

#include <cstdint>
#include <memory>
#include <new>
#include <utility>

#include "common/expected.hpp"

namespace nvidia {
namespace gxf {

// Memory management helper class, it keeps track of what part of a big chunk of memory has been
// allocated and can efficently find the first available chunk of memory that fits a given size.
// This class works only with indexes and rely on an external class to hold the memory.
// Internally it uses binary segement tree to keep track the biggest available block in a given area
// of the the memory. All the operation should take a logarithmic time.
class FirstFitAllocatorBase {
 public:
  // Error codes used by the classes of this file.
  enum class Error {
    // Returned when the suballocators is already in use and some memory has not been released yet.
    kAlreadyInUse,
    // Returned if the size is invalid (negative or too big).
    kInvalidSize,
    // Returned if the class can't allocate enough memory.
    kOutOfMemory,
    // Returned if we attempt to release a block of memory not allocated yet.
    kBlockNotAllocated,
    // This error happens when there is logically issue during the execution. This should never
    // happen.
    kLogicError,
  };
  // Expected type used by this class.
  template <typename T>
  using expected_t = nvidia::Expected<T, Error>;
  // Unexpected type used by this class.
  using unexpected_t = nvidia::Unexpected<Error>;

  FirstFitAllocatorBase();
  // Allocates the require memory to handle the query for a chunk of memory of a give size .
  // It requires 32 * 2^ceil(log2(size)) Bytes of memory.
  // It fails if size is invalid (too big or negative).
  expected_t<void> allocate(int32_t size);
  // Attempts to acquire a block of memory of a given size.
  // If such a contiguous block exists, it will return the lowest index where such a block exist.
  // This block will be blocked until a call to release(index, size) is made.
  // If no block exists, it will return Error::kOutOfMemory.
  expected_t<int32_t> acquire(int32_t size);
  // Releases a block of memory that has been acquired with the function above.
  // If there was no `acquire(size)` that returned `index`, this query will fail, otherwise it will
  // free the block of memory for further acquisition.
  // Note once a block of memory has been released, it can't be released again.
  expected_t<void> release(int32_t index);

 private:
  // Helper data structure to efficiently find a block of memory of a given size.
  // It represents a node in a binary segement tree.
  struct Memory {
    // Updates the node using both children.
    // size = l.size + r.size
    // left = l.left or l.size + r.left iff l.max == l.size
    // right = r.right or r.right + l.right iff r.max == r.size
    // max = max(l.max, r.max, l.right + r.left)
    void update(const Memory& left_child, const Memory& right_child);
    // Marks a node as beeing free (if == 1) or acquired (if == 0).
    void set(int32_t free);

    // Left and right store the size of available memory which start respectively from the left or
    // right side of the subtree.
    int32_t left, right;
    // Max is the size of biggest available block in the subtree.
    // When a block is starting at the given index, the leaf will contain -1.
    int32_t max;
    // Size is constant overtime and represent the size of the subtree.
    // When a block has been acquired, size will contain the size that has been acquired from the
    // given position.
    int32_t size;
  };

  // Updates a path from a given node to the root.
  void propagateToRoot(int32_t idx);
  // Updates a segment of the tree [left, right[ and marks it as free or acquired.
  void update(int32_t left, int32_t right, int32_t free);

  // Hold the segment tree memory.
  std::unique_ptr<Memory[]> tree_;
  // The total size available at the beginning.
  int32_t size_;
  // Helper index that helps pointing toward the leaves of the tree.
  int32_t last_layer_first_index_;
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_STD_GEMS_SUBALLOCATORS_FIRST_FIT_ALLOCATOR_BASE_HPP
