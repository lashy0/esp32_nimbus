set(BME68x_API_PATH ${CMAKE_CURRENT_LIST_DIR}/bme68x_api)

set(SRCS_FILES
    "bme680.c"
    "${BME68x_API_PATH}/bme68x.c"
)

set(INCLUDES_DIRS
    "include"
    "${BME68x_API_PATH}"
)

set(PUBLISH_REQUIRES
    "driver"
)

idf_component_register(
    SRCS "${SRCS_FILES}"
    INCLUDE_DIRS "${INCLUDES_DIRS}"
    REQUIRES "${PUBLISH_REQUIRES}"
)