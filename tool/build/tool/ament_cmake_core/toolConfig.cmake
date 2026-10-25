# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_tool_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED tool_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(tool_FOUND FALSE)
  elseif(NOT tool_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(tool_FOUND FALSE)
  endif()
  return()
endif()
set(_tool_CONFIG_INCLUDED TRUE)

# output package information
if(NOT tool_FIND_QUIETLY)
  message(STATUS "Found tool: 0.1.0 (${tool_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'tool' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${tool_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(tool_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${tool_DIR}/${_extra}")
endforeach()
