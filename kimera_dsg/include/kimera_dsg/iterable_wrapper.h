#pragma once
#include <cstdint>
#include <type_traits>

namespace kimera {

/**
 * @brief class that handles constant iterator access
 *
 * This is intended to be a thin wrapper around begin and end
 * for various standard library containers to expose a more natural
 * style of iteration around members of the scene graph and scene graph
 * layers while making it harder to accidentally mess up the internal
 * book-keeping of the graph.
 */
template <typename Iter>
class IterableWrapper {
 public:
  /**
   * @brief value type of underlying container
   */
  using value_type = typename Iter::value_type;
  /**
   * @brief reference type of underlying container
   */
  using reference = typename Iter::reference;
  /**
   * @brief const reference type of underlying container
   */
  using const_reference = typename Iter::const_reference;
  /**
   * @brief const iterator type of underlying container
   */
  using const_iterator = typename Iter::const_iterator;
  /**
   * @brief size type of underlying container
   */
  using size_type = typename Iter::size_type;

  /**
   * @brief make a wrapper around a reference iterator
   * @warning this makes this class non-copyable and non-movable
   */
  explicit IterableWrapper(const Iter& iterable) : iterable_(iterable) {}

  // these are all unsafe for this class
  IterableWrapper(const IterableWrapper& other) = delete;
  IterableWrapper& operator=(const IterableWrapper& other) = delete;
  IterableWrapper(IterableWrapper&& other) = delete;
  IterableWrapper& operator=(IterableWrapper&& other) = delete;

  /**
   * @brief get the begin iterator of the underlying container
   * @returns begin of the underlying container
   */
  inline const_iterator begin() const { return iterable_.begin(); }

  /**
   * @brief get the end iterator of the underlying container
   * @returns end of the underlying container
   */
  inline const_iterator end() const { return iterable_.end(); }

  /**
   * @brief get the constant begin iterator of the underlying container
   * @returns cbegin of the underlying container
   */
  inline const_iterator cbegin() const { return iterable_.cbegin(); }

  /**
   * @brief get the constant end iterator of the underlying container
   * @returns cend of the underlying container
   */
  inline const_iterator cend() const { return iterable_.cend(); }

  /**
   * @brief get the size of the underlying container
   * @returns the size of the underlying container
   */
  inline size_type size() const { return iterable_.size(); }

  /**
   * @brief get whether or not the underlying container is empty
   * @returns whether the underlying container is empty
   */
  inline bool empty() const { return iterable_.empty(); }

  // TODO(nathan) consider (but probably don't do) random access via at
  // TODO(nathan) consider eventually taking a filter

 private:
  const Iter& iterable_;
};

}  // namespace kimera
