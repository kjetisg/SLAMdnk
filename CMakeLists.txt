cmake_minimum_required(VERSION 3.13)

project( SLAMdnk LANGUAGES CXX)

get_filename_component(SLAMdnk_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/SLAMdnk DIRECTORY)
message("Project root is:  ${SLAMdnk_ROOT}")
message("Project name: ${PROJECT_NAME}")
message("CMake binary dir: ${CMAKE_BINARY_DIR}")

set(FEATURE_EXTRACTOR_DIR ${SLAMdnk_ROOT}/featureExtractor) #ToDo: Change this directory

set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE 1) #No source extension in object file

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup(TARGETS)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_COMPILER /usr/bin/g++)
set(CMAKE_STATIC_LIBRARY_PREFIX_CXX "")

set(PROJECT_LIB ${PROJECT_NAME}_lib)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CONAN_INCLUDE_DIRS}
    ${PROJECT_BINARY_DIR}
)

if (UNIX)
    # UNIX-specific options:
    add_compile_options(-std=c++17 -pthread)
endif()


set(SLAMdnk_MAIN_OBJS
    ${SLAMdnk_ROOT}/main.cpp
    ${SLAMdnk_ROOT}/CMakeLists.txt
)

add_library(${PROJECT_NAME}_lib STATIC ${SLAMdnk_MAIN_OBJS} ${SLAMdnk_SRC})
add_subdirectory(${FEATURE_EXTRACTOR_DIR})
message("FEATURE_EXTRACTOR_DIR : ${FEATURE_EXTRACTOR_DIR}")


add_executable(${PROJECT_NAME} ${SLAMdnk_MAIN_OBJS})

target_link_libraries(${PROJECT_NAME} PUBLIC ${PROJECT_LIB} FeatureExtractorLib)


target_compile_definitions(${PROJECT_NAME} PUBLIC ${CONAN_DEFINES})

target_link_libraries( ${PROJECT_NAME}_lib 
            CONAN_PKG::eigen
            CONAN_PKG::opencv
            CONAN_PKG::gtsam
            # ${CONAN_LIBS} 
)
# target_link_libraries(${PROJECT_NAME} ${PROJECT_LIB})