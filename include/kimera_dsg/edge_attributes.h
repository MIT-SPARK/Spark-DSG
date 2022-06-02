#pragma once
#include <memory>

namespace kimera {

/**
 * @brief Collection of information for an edge
 * @note most of the fields are unused but will hopefully make using
 *       the DSG classes more flexible later
 */
struct EdgeAttributes {
  //! desired pointer type for the edge attributes
  using Ptr = std::unique_ptr<EdgeAttributes>;

  EdgeAttributes();

  explicit EdgeAttributes(double weight);

  virtual ~EdgeAttributes();

  /**
   * @brief output attribute information
   * @param out output stream
   * @param attrs attributes to print
   * @returns original output stream
   */
  friend std::ostream& operator<<(std::ostream& out, const EdgeAttributes& attrs);

  virtual EdgeAttributes::Ptr clone() const;

  //! whether or not the edge weight is valid
  bool weighted;
  //! the weight of the edge
  double weight;

 protected:
  //! actually output information to the std::ostream
  virtual void fill_ostream(std::ostream& out) const;
};

}  // namespace kimera
