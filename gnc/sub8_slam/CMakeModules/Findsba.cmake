# Partially lifted from some crappy repository - Jake
# - Try to find libSBA
# Once done, this will define
#  SBA_FOUND - system has libsba
#  SBA_INCLUDE_DIRS - the libsba include directories
#  SBA_LIBRARIES - link these to use libsba


# Include dir
find_path(SBA_INCLUDE_DIR
  # sba.h
  # NAMES sba/sba.h sparse_bundle_adjustment/sba.h
  NAMES sba sparse_bundle_adjustment
  PATHS
  /opt/local/include
  /opt/ros/indigo/include
  /usr/include
  /usr/local/include
)

# Finally the library itself
find_library(SBA_LIB
  NAMES sba
  PATHS
  /opt/local/lib
  /usr/local/lib
  /opt/ros/indigo/lib
  /usr/lib
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(sba DEFAULT_MSG SBA_LIB SBA_INCLUDE_DIR)

