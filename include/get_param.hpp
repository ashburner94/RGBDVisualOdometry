#ifndef GET_PARAM_HPP
#define GET_PARAM_HPP

#include <stdexcept>

/**
 * @brief Try to load a parameter and throw a runtime error if it doesn't
 * succeed.
 */
template <typename T>
inline static void getParam(ros::NodeHandle& nh, std::string param_name,
                            T& param) {
  if (!nh.getParam(param_name, param)) {
    throw std::runtime_error(nh.getNamespace() + ": failed to load parameter " +
                             param_name);
  }
}

#endif /*GET_PARAM_HPP*/
