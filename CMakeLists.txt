cmake_minimum_required(VERSION 3.13)

project( SLAMdnk LANGUAGES CXX)
#ToDo: Separate project name from executable name - we probably want several executables!

get_filename_component(SLAMdnk_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/SLAMdnk DIRECTORY)
message(STATUS "Project root is:  ${SLAMdnk_ROOT}")
message("Project name: ${PROJECT_NAME}")
message("CMake binary dir: ${CMAKE_CURRENT_BINARY_DIR}")

set(FEATURE_EXTRACTOR_DIR ${SLAMdnk_ROOT}/featureExtractor) #ToDo: Change this directory

set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE 1) #No source extension in object file

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup(TARGETS)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_COMPILER /usr/bin/g++)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_STATIC_LIBRARY_PREFIX_CXX "")

set(PROJECT_LIB ${PROJECT_NAME}_lib)


include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CONAN_INCLUDE_DIRS}
    ${PROJECT_BINARY_DIR}
)

message("CONAN_INCLUDE_DIRS : ${CONAN_INCLUDE_DIRS}")

if (UNIX)
    # UNIX-specific options:
    add_compile_options(-std=c++17 -pthread)
    add_compile_options(-march=native -m64)# -flto)
    # add_compile_options(-l=3)
    # add_link_options(--allow-multiple-definition)
    add_compile_options( -DSPDLOG_BUILD_TESTS=OFF -DSPDLOG_BUILD_TESTS_HO=OFF -DSPDLOG_FMT_EXTERNAL=ON)
endif()


set(SLAMdnk_MAIN_OBJS
    ${SLAMdnk_ROOT}/main.cpp
    ${SLAMdnk_ROOT}/CMakeLists.txt
)

# add_library(${PROJECT_NAME}_lib MODULE "")#STATIC ${SLAMdnk_MAIN_OBJS} ${SLAMdnk_SRC})
# set_target_properties(${PROJECT_NAME}_lib PROPERTIES LINKER_LANGUAGE CXX)
add_subdirectory(${FEATURE_EXTRACTOR_DIR})

#
# target_link_libraries(myApp PRIVATE
# $<IF:$<CONFIG:Debug>,checkedAlgo,fastAlgo>
find_package(OpenCV)
if (OpenCV_LIBS MATCHES "^opencv")
    message(STATUS "Local OpenCV installation found, linking this over Conan version.")
    include_directories( ${OpenCV_INCLUDE_DIRS} )
else()
    message(STATUS "Using Conan version of OpenCV")
endif()
#

set(EXTERNAL_LIBRARIES 
    CONAN_PKG::spdlog
    CONAN_PKG::eigen
    # CONAN_PKG::opencv
    CONAN_PKG::gtsam
    CONAN_PKG::gstreamer
    ${OpenCV_LIBS}
    # ${CONAN_LIBS}
)

add_executable(${PROJECT_NAME} ${SLAMdnk_MAIN_OBJS})

# target_include_directories( ${PROJECT_NAME} PRIVATE
#     ${CMAKE_CURRENT_SOURCE_DIR}
#     ${CONAN_INCLUDE_DIRS}
#     ${PROJECT_BINARY_DIR}
# )

# target_include_directories( FeatureExtractorLib PRIVATE
#     # ${CMAKE_CURRENT_SOURCE_DIR}
#     ${CONAN_INCLUDE_DIRS}
#     # ${PROJECT_BINARY_DIR}
# )

target_link_libraries(${PROJECT_NAME} 
    PUBLIC 
        # ${PROJECT_LIB} 
        FeatureExtractorLib 
        ${EXTERNAL_LIBRARIES}
)

target_compile_definitions(${PROJECT_NAME} PUBLIC ${CONAN_DEFINES})
if (CMAKE_COMPILER_IS_GNUCXX)
    target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -g)
endif()

# target_link_libraries( ${PROJECT_NAME}_lib 
#     CONAN_PKG::spdlog
#     CONAN_PKG::eigen
#     # CONAN_PKG::opencv
#     CONAN_PKG::gtsam
#     CONAN_PKG::gstreamer
#     ${OpenCV_LIBS}
#     # ${CONAN_LIBS} 
# )
# target_link_libraries(${PROJECT_NAME} ${PROJECT_LIB})

# add_custom_target(graphviz ALL
#                   "${CMAKE_COMMAND}" "--graphviz=dependency_graph.dot" .
#                   WORKING_DIRECTORY "${CMAKE_BINARY_DIR}")