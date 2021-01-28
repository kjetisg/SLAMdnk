
set(FEATURE_EXTRACTOR_SRC
	${CMAKE_CURRENT_LIST_DIR}/featureExtractor.cpp
)

set(FEATURE_EXTRACTOR_HDR
	${CMAKE_CURRENT_LIST_DIR}/featureExtractor.hpp
)

add_library(FeatureExtractorLib)
target_sources(FeatureExtractorLib
	PRIVATE 
		${FEATURE_EXTRACTOR_SRC}
	PUBLIC
		${FEATURE_EXTRACTOR_HDR}	
)
target_include_directories(FeatureExtractorLib PUBLIC ${CMAKE_CURRENT_LIST_DIR}) #INTERFACE?
