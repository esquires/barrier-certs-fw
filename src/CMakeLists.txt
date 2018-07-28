SET (LIBRARY_NAME barrier-certs)
SET (LIB_MAJOR 0)
SET (LIB_MINOR 0)
SET (LIB_RELEASE 1)

#set(SRCS BarrierCert.cpp)
set(SRCS
  Utilities.cpp
  BarrierCert.cpp
  LimitedSensing.cpp
  ControlsIO.cpp)

ADD_LIBRARY(${LIBRARY_NAME} SHARED ${SRCS})

# https://stackoverflow.com/a/52136398
get_target_property(lib_include_dirs osqp::osqp INTERFACE_INCLUDE_DIRECTORIES)
target_include_directories(${LIBRARY_NAME} SYSTEM PRIVATE ${lib_include_dirs})

include_directories(/home/esquires/repos/temp/NumericalIntegration)

target_include_directories(${LIBRARY_NAME}
    PUBLIC
    ${GMP_INCLUDES}
    ${MPFR_INCLUDES}
    ${MPFRCPP_INCLUDES}
   "$<INSTALL_INTERFACE:include>"
   "$<BUILD_INTERFACE:${PROJECT_INC_DIR}>")

TARGET_LINK_LIBRARIES(${LIBRARY_NAME}
  PUBLIC
  ${MPFR_LIBRARIES}
  ${GMP_LIBRARIES}
  scrimmage-core
  ${Boost_LIBRARIES}
  PRIVATE
  osqp::osqp)

SET (_soversion ${LIB_MAJOR}.${LIB_MINOR}.${LIB_RELEASE})

set_target_properties(${LIBRARY_NAME} PROPERTIES 
  SOVERSION ${LIB_MAJOR} 
  VERSION ${_soversion}
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
  )

install(TARGETS ${LIBRARY_NAME}
  # IMPORTANT: Add the library to the "export-set"
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)

SubDirList(SUBDIRS ${CMAKE_CURRENT_SOURCE_DIR})
FOREACH(subdir ${SUBDIRS})
  ADD_SUBDIRECTORY(${subdir})
ENDFOREACH()

# Push up the SCRIMMAGE_LIBS variable
set(PROJECT_LIBS ${PROJECT_LIBS} ${LIBRARY_NAME} PARENT_SCOPE)
set(PROJECT_PLUGINS ${PROJECT_PLUGINS} PARENT_SCOPE)
