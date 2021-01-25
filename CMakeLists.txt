cmake_minimum_required(VERSION 3.13)

project( SLAMdnk LANGUAGES CXX)

get_filename_component(SLAMdnk_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/SLAMdnk DIRECTORY)
message("Project root is:  ${SLAMdnk_ROOT}")
message("PROJECT_NAME: ${PROJECT_NAME}")
set(OPENCV_TEST_DIR ${SLAMdnk_ROOT}/src) #ToDo: Change this directory

set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE 1) #No source extension in object file

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup(TARGETS)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_COMPILER /usr/bin/g++)

set(PROJECT_LIB ${PROJECT_NAME}_lib)

# find_package( OpenCV REQUIRED )
# include_directories( ${OpenCV_INCLUDE_DIRS} )
message("OPENCV: ${OpenCV_INCLUDE_DIRS}")
# find_package( Eigen3 REQUIRED )

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CONAN_INCLUDE_DIRS}
    ${PROJECT_BINARY_DIR}
)

if (UNIX)
    # UNIX-specific options
    add_compile_options(-std=c++17 -pthread)
    # set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread")
    # set (CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -lstdc++fs")

endif()


set( SLAMdnk_MAIN_OBJS
    ${SLAMdnk_ROOT}/main.cpp
    ${SLAMdnk_ROOT}/CMakeLists.txt
)
set(SLAMdnk_SRC
    # src/DisplayImage.cpp
)
set( SLAMdnk_HDR

)
add_library(${PROJECT_NAME}_lib STATIC ${SLAMdnk_MAIN_OBJS} ${SLAMdnk_SRC})
add_subdirectory(${OPENCV_TEST_DIR})

add_executable(${PROJECT_NAME} ${SLAMdnk_MAIN_OBJS})
target_link_libraries(${PROJECT_NAME} PUBLIC
    ${PROJECT_LIB}    
)

message("MESSAGE: ${CONAN_INCLUDE_DIRS}")
message("MESSAGE: ${CMAKE_BINARY_DIR}")
message("CONAN_LIBS: ${CONAN_LIBS}")

# target_include_directories(${PROJECT_NAME} 
#     PRIVATE
#         src/
#     INTERRACE 
#         src/
# )

target_compile_definitions(${PROJECT_NAME} PUBLIC ${CONAN_DEFINES})
# include_external_msproject(SYSTEM ${CONAN_INCLUDE_DIRS})

# include_directories(${CONAN_INCLUDE_DIRS})
# add_executable( ${PROJECT_NAME}  ${SLAMdnk_SRC})

target_link_libraries( ${PROJECT_NAME}_lib 
            # ${CONAN_LIBS} 
            CONAN_PKG::eigen
            CONAN_PKG::opencv
            # ${OpenCV_LIBS} 
            ${opencv_testing}
)

# target_link_libraries(${PROJECT_NAME} ${PROJECT_LIB})