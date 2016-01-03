# Author: Eric Huang - Jake
# Find Eigen
# This sets the following variables:
# Eigen_FOUND
# Eigen_INCLUDE_DIRS

find_path(EIGEN_INCLUDE_DIR
    NAMES Eigen/Core
    PATHS "${CMAKE_INSTALL_PREFIX}/include"
    PATH_SUFFIXES eigen3 eigen)

set(EIGEN_INCLUDE_DIRS ${EIGEN_INCLUDE_DIR})

# FIXME Find a consistent name for Eigen!
set(Eigen_INCLUDE_DIR ${EIGEN_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Eigen DEFAULT_MSG EIGEN_INCLUDE_DIR)

mark_as_advanced(EIGEN_INCLUDE_DIR)