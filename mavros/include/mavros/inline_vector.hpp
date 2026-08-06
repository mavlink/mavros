/*
 * Copyright 2026 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief Portable inline buffer (small vector)
 * @file inline_vector.hpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup mavutils
 * @{
 *  @brief A small inline sequence container
 */

#pragma once

#ifndef MAVROS__INLINE_VECTOR_HPP_
#define MAVROS__INLINE_VECTOR_HPP_

#include <array>
#include <cstddef>
#include <vector>

namespace mavros
{
namespace utils
{

/**
 * A portable implementation of a small inline sequence container, filling the
 * role of C++23 std::inplace_vector (and boost::container::small_vector) for
 * the C++20 baseline used by this project.
 *
 * Elements are stored in an inline array of capacity N; when that capacity is
 * exceeded (e.g. a large swarm) the contents spill into a heap-allocated
 * std::vector. Unlike std::inplace_vector, which throws std::bad_alloc when
 * full, this container grows without a hard limit.
 */
template<typename T, size_t N>
class InlineVector
{
public:
  InlineVector() = default;

  void push_back(const T & value)
  {
    if (spilled_) {
      spill_.push_back(value);
    } else if (size_ < N) {
      inline_[size_++] = value;
    } else {
      spilled_ = true;
      spill_.reserve(N + 8);
      spill_.insert(spill_.end(), inline_.begin(), inline_.begin() + size_);
      spill_.push_back(value);
    }
  }

  [[nodiscard]] bool empty() const
  {
    return spilled_ ? spill_.empty() : size_ == 0;
  }

  [[nodiscard]] size_t size() const
  {
    return spilled_ ? spill_.size() : size_;
  }

  const T * begin() const
  {
    return spilled_ ? spill_.data() : inline_.data();
  }

  const T * end() const
  {
    return begin() + size();
  }

private:
  std::array<T, N> inline_{};
  std::vector<T> spill_;
  size_t size_{0};
  bool spilled_{false};
};

}   // namespace utils
}   // namespace mavros

#endif  // MAVROS__INLINE_VECTOR_HPP_
