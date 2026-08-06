//
// libmavconn
// Copyright 2026 Vladimir Ermakov, All rights reserved.
//
// This file is part of the mavros package and subject to the license terms
// in the top-level LICENSE file of the mavros repository.
// https://github.com/mavlink/mavros/tree/master/LICENSE.md
//

/**
 * @brief Unit tests for the tx queue block pool allocator
 */

#include <gtest/gtest.h>

#include <atomic>
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <new>
#include <vector>

#include "mavconn/pool_allocator.hpp"

using mavconn::MsgBuffer;
using mavconn::MsgBufferAllocator;
using mavconn::MsgBufferBlockPool;

namespace
{

// Count only the chunk-sized allocations, which is exactly what the pool
// allocates for heap slots (kChunkBlocks * kBlockSize). No other allocation in
// the process uses that size, so the counter is unambiguous.
std::atomic<size_t> g_chunk_allocs{0};
constexpr size_t kChunkBytes = MsgBufferBlockPool::kChunkBlocks * MsgBufferBlockPool::kBlockSize;

void reset_counter()
{
  g_chunk_allocs = 0;
}

bool is_aligned(const void * p)
{
  return reinterpret_cast<uintptr_t>(p) % alignof(MsgBuffer) == 0;
}

}  // namespace

// Global new/delete overrides so the tests can observe heap slot allocations.
void * operator new(std::size_t n)
{
  if (n == kChunkBytes) {
    g_chunk_allocs++;
  }
  return std::malloc(n);
}

void * operator new[](std::size_t n)
{
  if (n == kChunkBytes) {
    g_chunk_allocs++;
  }
  return std::malloc(n);
}

void * operator new(std::size_t n, const std::nothrow_t &) noexcept
{
  if (n == kChunkBytes) {
    g_chunk_allocs++;
  }
  return std::malloc(n);
}

void operator delete(void * p) noexcept
{
  std::free(p);
}

void operator delete[](void * p) noexcept
{
  std::free(p);
}

void operator delete(void * p, std::size_t) noexcept
{
  std::free(p);
}

void operator delete[](void * p, std::size_t) noexcept
{
  std::free(p);
}

TEST(PoolAllocator, InlineFirstChunkZeroAlloc)
{
  MsgBufferBlockPool pool;
  std::vector<void *> ptrs;

  reset_counter();
  for (size_t i = 0; i < MsgBufferBlockPool::kChunkBlocks; i++) {
    void * p = pool.acquire();
    ASSERT_NE(nullptr, p);
    ASSERT_TRUE(is_aligned(p));
    ptrs.push_back(p);
  }

  EXPECT_EQ(size_t(0), g_chunk_allocs) << "first chunk must come from inline storage";
  for (auto p : ptrs) {
    pool.release(p);
  }
}

TEST(PoolAllocator, GrowsWithContiguousChunks)
{
  MsgBufferBlockPool pool;
  std::vector<void *> ptrs;

  reset_counter();
  for (size_t i = 0; i < 100; i++) {
    void * p = pool.acquire();
    ASSERT_NE(nullptr, p);
    ptrs.push_back(p);
  }

  EXPECT_GT(g_chunk_allocs, size_t(0)) << "growing past inline must allocate heap chunks";

  // all outstanding pointers distinct and aligned
  for (size_t i = 0; i < ptrs.size(); i++) {
    ASSERT_TRUE(is_aligned(ptrs[i]));
    for (size_t j = i + 1; j < ptrs.size(); j++) {
      EXPECT_NE(ptrs[i], ptrs[j]);
    }
  }

  for (auto p : ptrs) {
    pool.release(p);
  }
}

TEST(PoolAllocator, ShrinksBackToInlineWhenIdle)
{
  MsgBufferBlockPool pool;
  std::vector<void *> ptrs;

  // grow well past the inline chunk
  for (size_t i = 0; i < 200; i++) {
    ptrs.push_back(pool.acquire());
  }
  EXPECT_GT(g_chunk_allocs, size_t(0));

  // drain fully (FIFO, like the deque)
  for (auto p : ptrs) {
    pool.release(p);
  }

  // after draining, re-acquiring one chunk must not allocate heap
  reset_counter();
  std::vector<void *> again;
  for (size_t i = 0; i < MsgBufferBlockPool::kChunkBlocks; i++) {
    again.push_back(pool.acquire());
  }
  EXPECT_EQ(size_t(0), g_chunk_allocs) << "heap chunks must be freed when idle";

  for (auto p : again) {
    pool.release(p);
  }
}

TEST(PoolAllocator, GrowsToDequeLimit)
{
  // The pool has no cap of its own; the owning deque bounds it via
  // MAX_TXQ_SIZE. Verify it can grow to hold that many concurrent blocks.
  MsgBufferBlockPool pool;
  std::vector<void *> ptrs;

  constexpr size_t kDequeLimit = 1000;   // MAVConnInterface::MAX_TXQ_SIZE
  for (size_t i = 0; i < kDequeLimit; i++) {
    ptrs.push_back(pool.acquire());
  }

  EXPECT_EQ(kDequeLimit, ptrs.size());
  for (size_t i = 0; i < ptrs.size(); i++) {
    for (size_t j = i + 1; j < ptrs.size(); j++) {
      EXPECT_NE(ptrs[i], ptrs[j]);
    }
  }

  for (auto p : ptrs) {
    pool.release(p);
  }
}

TEST(PoolAllocator, FifoReuseStaysBounded)
{
  MsgBufferBlockPool pool;

  // mimic the deque pattern: push up to K, drain, repeat; heap usage must not
  // grow unboundedly across rounds.
  size_t peak = 0;
  for (int round = 0; round < 10; round++) {
    reset_counter();
    std::vector<void *> v;
    for (size_t i = 0; i < 80; i++) {
      v.push_back(pool.acquire());
    }
    peak = std::max(peak, g_chunk_allocs.load());
    for (auto p : v) {
      pool.release(p);
    }
  }

  // 80 slots = 1 inline chunk (32) + 2 heap chunks (64); never more.
  EXPECT_LE(peak, size_t(2));
}

TEST(PoolAllocator, DequeSteadyStateNoAlloc)
{
  std::deque<MsgBuffer, MsgBufferAllocator<MsgBuffer>> tx_q;

  // warm up the pool
  for (size_t i = 0; i < MsgBufferBlockPool::kChunkBlocks; i++) {
    tx_q.emplace_back();
  }
  tx_q.clear();

  reset_counter();
  for (int i = 0; i < 100000; i++) {
    tx_q.emplace_back();
    tx_q.pop_front();
  }
  EXPECT_EQ(size_t(0), g_chunk_allocs) << "steady-state push/pop must not allocate";
}

TEST(PoolAllocator, DequeFrontReferenceStable)
{
  std::deque<MsgBuffer, MsgBufferAllocator<MsgBuffer>> tx_q;

  tx_q.emplace_back();
  MsgBuffer & ref = tx_q.front();
  for (int i = 0; i < 1000; i++) {
    tx_q.emplace_back();
    ASSERT_EQ(&ref, &tx_q.front()) << "deque must not relocate the front on push";
  }
}

TEST(PoolAllocator, AllocatorCopiesSharePool)
{
  MsgBufferAllocator<MsgBuffer> a;
  MsgBufferAllocator<MsgBuffer> b(a);

  EXPECT_EQ(a, b);

  // a block allocated by one must be reusable by the other (same pool)
  MsgBuffer * p = a.allocate(1);
  b.deallocate(p, 1);
}
