cmake_minimum_required(VERSION 3.25)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Features
option(FEATURE_SPI "SPI bus" ON)
option(FEATURE_I2C "I2C bus" ON)
option(FEATURE_FIFO "FIFO feature" ON)

project(icm-42688 CXX)

set(CMAKE_DEBUG_POSTFIX d)

add_library(${PROJECT_NAME} "")
add_library(immortals::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

set(HEADER_FILES
        source/device.h
        source/device_inl.h
        source/registers.h)

set(SOURCE_FILES
        source/device.cpp)

if (${FEATURE_FIFO})
    target_compile_definitions(${PROJECT_NAME} PUBLIC ICM42688_FEATURE_FIFO=1)
    list(APPEND HEADER_FILES
            source/fifo.h)
    list(APPEND SOURCE_FILES
            source/fifo.cpp)
endif ()

target_sources(${PROJECT_NAME} PRIVATE ${HEADER_FILES} ${SOURCE_FILES})

if (${FEATURE_SPI})
    target_compile_definitions(${PROJECT_NAME} PUBLIC ICM42688_FEATURE_SPI=1)
endif ()
if (${FEATURE_I2C})
    target_compile_definitions(${PROJECT_NAME} PUBLIC ICM42688_FEATURE_I2C=1)
endif ()

install(DIRECTORY source/
        DESTINATION include/icm-42688
        FILES_MATCHING
        PATTERN "*.h")

install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}_target
        LIBRARY DESTINATION lib)

include(CMakePackageConfigHelpers)
configure_package_config_file(${PROJECT_NAME}-config.cmake.in ${PROJECT_NAME}-config.cmake INSTALL_DESTINATION share/${PROJECT_NAME})

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
        DESTINATION share/${PROJECT_NAME})

install(EXPORT ${PROJECT_NAME}_target
        NAMESPACE immortals::
        FILE ${PROJECT_NAME}-targets.cmake
        DESTINATION share/${PROJECT_NAME})
