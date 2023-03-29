# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_prototype_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED prototype_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(prototype_FOUND FALSE)
  elseif(NOT prototype_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(prototype_FOUND FALSE)
  endif()
  return()
endif()
set(_prototype_CONFIG_INCLUDED TRUE)

# output package information
if(NOT prototype_FIND_QUIETLY)
  message(STATUS "Found prototype: 0.0.0 (${prototype_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'prototype' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${prototype_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(prototype_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${prototype_DIR}/${_extra}")
endforeach()
