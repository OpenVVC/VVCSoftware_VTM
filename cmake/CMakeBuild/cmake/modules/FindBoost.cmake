#
# This module adds find support for Boost up to 1.66.0.
# Depending on the CMake version it will pull in a patched module or the original module.
#
# Wrapper to pull in a patched FindBoost.cmake depending on the CMake version.


#if( CMAKE_VERSION VERSION_LESS 3.11 )
  message( STATUS "Loading patched FindBoost.cmake to get Boost 1.66.0 support." )
  include( ${CMAKE_CURRENT_LIST_DIR}/patched/FindBoost.cmake )
#else()
#  include( ${CMAKE_ROOT}/Modules/FindBoost.cmake )
#endif()
