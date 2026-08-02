set(KISS_MATCHER_PATH "${PROJECT_SOURCE_DIR}/thirdparty/kiss_matcher")
set(ROBIN_PATH "${PROJECT_SOURCE_DIR}/thirdparty/robin")
set(PMC_PATH "${PROJECT_SOURCE_DIR}/thirdparty/pmc")
set(FLANN_PATH "${PROJECT_SOURCE_DIR}/thirdparty/flann/include")
set(XENIUM_PATH "${PROJECT_SOURCE_DIR}/thirdparty/xenium")

find_package(OpenMP REQUIRED)
find_library(LZ4_LIBRARY lz4 REQUIRED)

function(legkilo_enable_openmp target_name)
  if(TARGET OpenMP::OpenMP_CXX)
    target_link_libraries(${target_name} PRIVATE OpenMP::OpenMP_CXX)
  else()
    if(OpenMP_CXX_FLAGS)
      separate_arguments(OPENMP_CXX_FLAG_LIST NATIVE_COMMAND "${OpenMP_CXX_FLAGS}")
      target_compile_options(${target_name} PRIVATE ${OPENMP_CXX_FLAG_LIST})
    endif()
    if(OpenMP_CXX_LIBRARIES)
      target_link_libraries(${target_name} PRIVATE ${OpenMP_CXX_LIBRARIES})
    endif()
  endif()
endfunction()

add_library(pmc STATIC
  ${PMC_PATH}/pmc_heu.cpp
  ${PMC_PATH}/pmc_maxclique.cpp
  ${PMC_PATH}/pmcx_maxclique.cpp
  ${PMC_PATH}/pmcx_maxclique_basic.cpp
  ${PMC_PATH}/pmc_cores.cpp
  ${PMC_PATH}/pmc_utils.cpp
  ${PMC_PATH}/pmc_graph.cpp
  ${PMC_PATH}/pmc_clique_utils.cpp
)
target_include_directories(pmc PUBLIC
  ${PMC_PATH}/include
)
set_target_properties(pmc PROPERTIES POSITION_INDEPENDENT_CODE ON)
legkilo_enable_openmp(pmc)

add_library(robin STATIC
  ${ROBIN_PATH}/src/core.cpp
  ${ROBIN_PATH}/src/graph_core.cpp
  ${ROBIN_PATH}/src/graph_solvers.cpp
  ${ROBIN_PATH}/src/pkc.cpp
  ${ROBIN_PATH}/src/problems.cpp
  ${ROBIN_PATH}/src/robin.cpp
  ${ROBIN_PATH}/src/utils.cpp
)
target_include_directories(robin PUBLIC
  ${ROBIN_PATH}/include
  ${PMC_PATH}/include
  ${XENIUM_PATH}
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(robin PUBLIC
  pmc
)
set_target_properties(robin PROPERTIES POSITION_INDEPENDENT_CODE ON)
legkilo_enable_openmp(robin)

add_library(kiss_matcher STATIC
  ${KISS_MATCHER_PATH}/core/kiss_matcher/ROBINMatching.cpp
  ${KISS_MATCHER_PATH}/core/kiss_matcher/FasterPFH.cpp
  ${KISS_MATCHER_PATH}/core/kiss_matcher/KISSMatcher.cpp
  ${KISS_MATCHER_PATH}/core/kiss_matcher/GncSolver.cpp
)
target_include_directories(kiss_matcher PUBLIC
  ${KISS_MATCHER_PATH}/core
  ${ROBIN_PATH}/include
  ${PMC_PATH}/include
  ${FLANN_PATH}
  ${XENIUM_PATH}
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(kiss_matcher PUBLIC
  robin
  tbb
  ${LZ4_LIBRARY}
)
set_target_properties(kiss_matcher PROPERTIES POSITION_INDEPENDENT_CODE ON)
legkilo_enable_openmp(kiss_matcher)
