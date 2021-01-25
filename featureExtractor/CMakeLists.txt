
set(FEATURE_EXTRACTOR_SRC
${CMAKE_CURRENT_LIST_DIR}/DisplayImage.cpp
)

set(FEATURE_EXTRACTOR_HDR
${CMAKE_CURRENT_LIST_DIR}/featureExtractor.hpp
)

add_library(FeatureExtractorLib)
target_sources(FeatureExtractorLib #${PROJECT_NAME}_lib 
	PRIVATE 
		${FEATURE_EXTRACTOR_SRC}
	PUBLIC
		${FEATURE_EXTRACTOR_HDR}	
)
target_include_directories(FeatureExtractorLib INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
