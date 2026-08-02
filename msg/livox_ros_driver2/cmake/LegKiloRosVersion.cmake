set(LEGKILO_ROS_VERSION "auto" CACHE STRING "ROS version to build against: auto, 1, or 2")
set_property(CACHE LEGKILO_ROS_VERSION PROPERTY STRINGS auto 1 2)

function(legkilo_resolve_ros_version OUT_VAR)
  set(_resolved "${LEGKILO_ROS_VERSION}")
  string(TOLOWER "${_resolved}" _resolved_lower)

  if(_resolved_lower STREQUAL "auto")
    if(DEFINED ENV{ROS_VERSION} AND NOT "$ENV{ROS_VERSION}" STREQUAL "")
      set(_resolved "$ENV{ROS_VERSION}")
    elseif(DEFINED ENV{ROS_DISTRO} AND NOT "$ENV{ROS_DISTRO}" STREQUAL "")
      if("$ENV{ROS_DISTRO}" MATCHES "^(foxy|galactic|humble|iron|jazzy|rolling)$")
        set(_resolved "2")
      else()
        set(_resolved "1")
      endif()
    else()
      set(_resolved "1")
    endif()
  endif()

  if(NOT _resolved STREQUAL "1" AND NOT _resolved STREQUAL "2")
    message(FATAL_ERROR "LEGKILO_ROS_VERSION must be one of: auto, 1, 2. Current value: ${LEGKILO_ROS_VERSION}")
  endif()

  set(${OUT_VAR} "${_resolved}" PARENT_SCOPE)
endfunction()

