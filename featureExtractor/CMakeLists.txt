
set(FEATURE_EXTRACTOR_SRC
	${CMAKE_CURRENT_LIST_DIR}/featureExtractor.cpp
)

set(FEATURE_EXTRACTOR_HDR
	${CMAKE_CURRENT_LIST_DIR}/featureExtractor.hpp
)

# add_library(FeatureExtractorLib)
target_sources(${PROJECT_LIB}  	#FeatureExtractorLib
	PRIVATE 
		${FEATURE_EXTRACTOR_SRC}
		${FEATURE_EXTRACTOR_HDR}
	PUBLIC
		${FEATURE_EXTRACTOR_HDR}	
)
# target_include_directories(${PROJECT_LIB} PRIVATE ${CMAKE_CURRENT_LIST_DIR})#FeatureExtractorLib PUBLIC ${CMAKE_CURRENT_LIST_DIR}) #INTERFACE?
