# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_myactuator_rmd_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED myactuator_rmd_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(myactuator_rmd_FOUND FALSE)
  elseif(NOT myactuator_rmd_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(myactuator_rmd_FOUND FALSE)
  endif()
  return()
endif()
set(_myactuator_rmd_CONFIG_INCLUDED TRUE)

# output package information
if(NOT myactuator_rmd_FIND_QUIETLY)
  message(STATUS "Found myactuator_rmd: 0.0.1 (${myactuator_rmd_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'myactuator_rmd' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT myactuator_rmd_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(myactuator_rmd_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_include_directories-extras.cmake;ament_cmake_export_targets-extras.cmake")
foreach(_extra ${_extras})
  include("${myactuator_rmd_DIR}/${_extra}")
endforeach()
