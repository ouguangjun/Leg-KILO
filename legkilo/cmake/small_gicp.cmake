set(SMALL_GICP_PATH "${PROJECT_SOURCE_DIR}/thirdparty/smallgicp")

if(SMALL_GICP_ONLY_HEADER)
  include_directories(${SMALL_GICP_PATH})
else()
  add_library(small_gicp STATIC
    ${SMALL_GICP_PATH}/small_gicp/registration/registration.cpp
    ${SMALL_GICP_PATH}/small_gicp/registration/registration_helper.cpp
  )

  target_include_directories(small_gicp PUBLIC
    ${SMALL_GICP_PATH}
  )
endif()