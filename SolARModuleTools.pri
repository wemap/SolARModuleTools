HEADERS += interfaces/SolARImage2WorldMapper4Marker2D.h \
interfaces/SolAR2DTransform.h \
interfaces/SolAR3DTransform.h \
interfaces/SolARHomographyValidation.h \
interfaces/SolARSBPatternReIndexer.h \
interfaces/SolARKeypointsReIndexer.h \
interfaces/SolARMapper.h \
interfaces/SolARMapFilter.h \
interfaces/SolARToolsAPI.h \
interfaces/SolARModuleTools_traits.h \
interfaces/SolARBasicMatchesFilter.h \
interfaces/SolARKeyframeSelector.h \
interfaces/SolARBasicSink.h \
interfaces/SolARBasicSource.h \
interfaces/SolARPointCloudManager.h \
interfaces/SolARKeyframesManager.h \
interfaces/SolARCovisibilityGraph.h \
interfaces/SolARBoostCovisibilityGraph.h \
interfaces/SolARLoopCorrector.h \
interfaces/SolARLoopClosureDetector.h \
interfaces/SolAR3D3DcorrespondencesFinder.h \
interfaces/SolAR3DTransformEstimationSACFrom3D3D.h \
interfaces/SolARFiducialMarkerPoseEstimator.h \
interfaces/SolARSLAMBootstrapper.h \
interfaces/SolARSLAMTracking.h \
interfaces/SolARSLAMMapping.h



SOURCES += src/SolARImage2WorldMapper4Marker2D.cpp \
    src/SolAR2DTransform.cpp \
    src/SolAR3DTransform.cpp \
    src/SolARHomographyValidation.cpp \
    src/SolARSBPatternReIndexer.cpp \
    src/SolARKeypointsReIndexer.cpp \
    src/SolARBasicMatchesFilter.cpp \
    src/SolARMapper.cpp \
    src/SolARMapFilter.cpp \
    src/SolARModuleTools.cpp \
    src/SolARKeyframeSelector.cpp \
    src/SolARBasicSink.cpp \
    src/SolARBasicSource.cpp \
    src/SolARPointCloudManager.cpp \
    src/SolARKeyframesManager.cpp \
    src/SolARCovisibilityGraph.cpp \
    src/SolARBoostCovisibilityGraph.cpp \
    src/SolARLoopCorrector.cpp \
    src/SolARLoopClosureDetector.cpp \
    src/SolAR3D3DcorrespondencesFinder.cpp \
    src/SolAR3DTransformEstimationSACFrom3D3D.cpp \
    src/SolARFiducialMarkerPoseEstimator.cpp \
    src/SolARSLAMBootstrapper.cpp \
    src/SolARSLAMTracking.cpp \
    src/SolARSLAMMapping.cpp
