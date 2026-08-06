//
// libmavconn
// Copyright 2026 Vladimir Ermakov, All rights reserved.
//
// This file is part of the mavros package and subject to the license terms
// in the top-level LICENSE file of the mavros repository.
// https://github.com/mavlink/mavros/tree/master/LICENSE.md
//
/**
 * @brief Fixed-size block pool allocator for the tx queue (internal)
 * @file pool_allocator.hpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup mavconn
 * @{
 */

#pragma once
#ifndef MAVCONN__POOL_ALLOCATOR_HPP_
#define MAVCONN__POOL_ALLOCATOR_HPP_

#include <algorithm>
#include <array>
#include <cstddef>
#include <memory>
#include <new>
#include <utility>
#include <vector>

#include <mavconn/msgbuffer.hpp>

namespace mavconn
{

/**
 * @brief Chunked block pool for the tx queue.
 *
 * Serves fixed @ref kBlockSize blocks. The first @ref kChunkBlocks blocks come
 * from one contiguous inline array (zero heap allocation); further blocks are
 * served from contiguous heap chunks of @ref kChunkBlocks each. Freed blocks are
 * recycled, so steady-state sends perform no malloc/free. When no heap slot is
 * in use, the heap chunks are freed and the pool returns to the inline-only
 * state (capacity back to @ref kChunkBlocks).
 *
 * The pool has no upper capacity limit of its own: the owning std::deque bounds
 * the number of live blocks via MAVConnInterface::MAX_TXQ_SIZE, so the pool is
 * never asked to hold more than that.
 *
 * @note NOT thread-safe by itself. All access is serialized by the owning
 *       channel's tx mutex (both emplace_back and pop_front hold it), and each
 *       connection owns its own pool, so no pool-internal locking is needed.
 */
class MsgBufferBlockPool
{
public:
  static constexpr size_t kBlockSize = sizeof(MsgBuffer);
  static constexpr size_t kChunkBlocks = 32;    // slots per contiguous chunk

  MsgBufferBlockPool() = default;
  ~MsgBufferBlockPool() = default;
  MsgBufferBlockPool(const MsgBufferBlockPool &) = delete;
  MsgBufferBlockPool & operator=(const MsgBufferBlockPool &) = delete;

  void * acquire()
  {
    if (!free_list_.empty()) {
      void * p = free_list_.back();
      free_list_.pop_back();
      chunk_of(p)->in_use++;
      in_use_++;
      return p;
    }
    if (inline_handed_ < kChunkBlocks) {
      void * p = inline_store_ + inline_handed_++ * kBlockSize;
      inline_chunk_.in_use++;
      in_use_++;
      return p;
    }
    return alloc_chunk_slot();
  }

  void release(void * p)
  {
    chunk_of(p)->in_use--;
    in_use_--;
    free_list_.push_back(p);
    shrink_if_idle();
  }

private:
  struct Chunk
  {
    std::unique_ptr<std::byte[]> data;   // null for the inline chunk
    size_t in_use{0};
  };

  alignas(std::max_align_t) std::byte inline_store_[kChunkBlocks * kBlockSize];
  Chunk inline_chunk_;                   // data == nullptr, points at inline_store_
  size_t inline_handed_{0};              // inline slots handed out via cursor
  std::vector<Chunk> chunks_;
  std::vector<void *> free_list_;
  size_t in_use_{0};

  [[nodiscard]] bool is_inline(void * p) const
  {
    auto base = reinterpret_cast<const std::byte *>(inline_store_);
    auto q = static_cast<const std::byte *>(p);
    return q >= base && q < base + kChunkBlocks * kBlockSize;
  }

  Chunk * chunk_of(void * p)
  {
    if (is_inline(p)) {
      return &inline_chunk_;
    }
    for (auto & c : chunks_) {
      auto cb = c.data.get();
      if (p >= cb && p < cb + kChunkBlocks * kBlockSize) {
        return &c;
      }
    }
    return nullptr;   // unreachable for slots handed out by this pool
  }

  void * alloc_chunk_slot()
  {
    auto chunk = std::make_unique<std::byte[]>(kChunkBlocks * kBlockSize);
    std::byte * base = chunk.get();
    chunks_.emplace_back(Chunk{std::move(chunk), 1});
    in_use_++;
    // remaining slots of the new chunk become available
    for (size_t i = 1; i < kChunkBlocks; i++) {
      free_list_.push_back(base + i * kBlockSize);
    }
    return base;
  }

  // Once no heap slot is in use, drop the heap chunks and return to the
  // inline-only state (capacity back to kChunkBlocks).
  void shrink_if_idle()
  {
    for (auto & c : chunks_) {
      if (c.in_use > 0) {
        return;
      }
    }
    free_list_.erase(
      std::remove_if(free_list_.begin(), free_list_.end(),
      [this](void * p) {return !is_inline(p);}),
      free_list_.end());
    chunks_.clear();
  }
};

/**
 * @brief Allocator pairing std::deque<MsgBuffer> with a MsgBufferBlockPool.
 *
 * Serves the deque's 1-element MsgBuffer nodes from the pool (std::deque nodes
 * are a single MsgBuffer because sizeof(MsgBuffer) > half the default node
 * size); all other allocations (the deque's pointer map) fall back to
 * ::operator new. Allocation of the pool is shared across rebinds, so every
 * copy of the allocator draws from the same pool.
 */
template<typename T>
class MsgBufferAllocator
{
public:
  using value_type = T;
  using propagate_on_container_copy_assignment = std::true_type;
  using propagate_on_container_move_assignment = std::true_type;
  using propagate_on_container_swap = std::true_type;

  MsgBufferAllocator() noexcept
  : pool_(std::make_shared<MsgBufferBlockPool>()) {}

  template<typename U>
  MsgBufferAllocator(const MsgBufferAllocator<U> & other) noexcept
  : pool_(other.pool_) {}

  [[nodiscard]] T * allocate(std::size_t n)
  {
    // The deque's MsgBuffer node blocks are the hot path; its pointer map
    // (other sizes) falls through to ::operator new.
    if (n == 1 && sizeof(T) == MsgBufferBlockPool::kBlockSize) [[likely]] {
      return static_cast<T *>(pool_->acquire());
    }
    return static_cast<T *>(::operator new(n * sizeof(T)));
  }

  void deallocate(T * p, std::size_t n) noexcept
  {
    if (n == 1 && sizeof(T) == MsgBufferBlockPool::kBlockSize) [[likely]] {
      pool_->release(p);
    } else {
      ::operator delete(p);
    }
  }

  template<typename U>
  [[nodiscard]] bool operator==(const MsgBufferAllocator<U> & other) const noexcept
  {
    return pool_ == other.pool_;
  }

  template<typename U>
  [[nodiscard]] bool operator!=(const MsgBufferAllocator<U> & other) const noexcept
  {
    return !(*this == other);
  }

private:
  std::shared_ptr<MsgBufferBlockPool> pool_;

  template<typename>
  friend class MsgBufferAllocator;
};

}  // namespace mavconn

#endif  // MAVCONN__POOL_ALLOCATOR_HPP_
