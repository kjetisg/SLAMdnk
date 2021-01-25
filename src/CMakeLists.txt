
set(OpenCVtesting_SRC
	DisplayImage.cpp
)

#add_library(opencv_testing SHARED ${OpenCVtesting_SRC})
target_sources(${PROJECT_NAME}_lib PRIVATE ${OpenCVtesting_SRC})