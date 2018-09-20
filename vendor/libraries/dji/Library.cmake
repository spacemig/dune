#add_subdirectory(vendor/libraries/dji/osdk-3.6.1)

set(DJI_OSDK_PATH "vendor/libraries/dji/osdk-3.6.1")

include_directories(${DJI_OSDK_PATH}/osdk-core/api/inc
                    ${DJI_OSDK_PATH}/osdk-core/protocol/inc
                    ${DJI_OSDK_PATH}/osdk-core/hal/inc
                    ${DJI_OSDK_PATH}/osdk-core/utility/inc
                    ${DJI_OSDK_PATH}/osdk-core/platform/linux/inc
                    ${DJI_OSDK_PATH}/osdk-core/platform/default/inc
                    ${DJI_OSDK_PATH}/sample/linux/common
                    ${DJI_OSDK_PATH}/sample/linux/flight-control
                    ${DJI_OSDK_PATH}/sample/linux/telemetry)

file(GLOB DUNE_DJI_FILES
  ${DJI_OSDK_PATH}/osdk-core/api/src/*.cpp
  ${DJI_OSDK_PATH}/osdk-core/protocol/src/*.cpp
  ${DJI_OSDK_PATH}/osdk-core/hal/src/*.cpp
  ${DJI_OSDK_PATH}/osdk-core/utility/src/*.cpp
  ${DJI_OSDK_PATH}/osdk-core/platform/linux/src/*.cpp
  ${DJI_OSDK_PATH}/osdk-core/platform/default/src/*.cpp
  ${DJI_OSDK_PATH}/sample/linux/common/*.cpp
  ${DJI_OSDK_PATH}/sample/linux/flight-control/flight_control_sample.cpp
  ${DJI_OSDK_PATH}/sample/linux/telemetry/telemetry_sample.cpp
  )

add_library(djiosdk-core
        STATIC
        ${DUNE_DJI_FILES})

list(APPEND DUNE_VENDOR_LIBS libdjiosdk-core.a)

set_source_files_properties(${DUNE_DJI_FILES}
  PROPERTIES COMPILE_FLAGS "${DUNE_CXX_FLAGS} ${DUNE_CXX_FLAGS_STRICT}")

list(APPEND DUNE_VENDOR_FILES ${DUNE_DJI_FILES})

#set(DUNE_VENDOR_INCS_DIR ${DUNE_VENDOR_INCS_DIR}
#  ${PROJECT_SOURCE_DIR}/vendor/libraries)


