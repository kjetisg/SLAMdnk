cmake_minimum_required(VERSION 3.8)

project( SLAMdnk LANGUAGES CUDA CXX)

# conan_basic_setup(TARGETS)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package( Eigen3 REQUIRED )

# find_package( CUDA 11.1 REQUIRED)
# include_directories( ${CUDA_INCLUDE_DIRS} )
# include_directories( ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES} )

# include(CheckLanguage)
# check_language(CUDA)

# if(NOT DEFINED CMAKE_CUDA_STANDARD)
#     set(CMAKE_CUDA_STANDARD 11)
#     set(CMAKE_CUDA_STANDARD_REQUIRED ON)
# endif()

# message("$(check_language(CUDA))")
# message("CUDA_LIBRARIES: ${CUDA_LIBRARIES}")
# message("CMAKE_CUDA_COMPILER: ${CMAKE_CUDA_COMPILER}")
# message("CMAKE_CUDA_COMPILER_ID: ${CMAKE_CUDA_COMPILER_ID}")
# message("CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES: ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}")

set(SLAMdnk_SRC
    src/DisplayImage.cpp
)
set( SLAMdnk_HDR

)

add_executable( ${PROJECT_NAME}  ${SLAMdnk_SRC})
# set_target_properties(${PROJECT_NAME} PROPERTIES
#                 CUDA_SEPARABLE_COMPILATION ON)

target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} ${CUDA_LIBRARIES} Eigen3::Eigen)