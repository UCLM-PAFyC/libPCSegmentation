#-------------------------------------------------
#
# Project created by QtCreator 2018-10-02T10:03:54
#
#-------------------------------------------------
debug_and_release {
    CONFIG -= debug_and_release
    CONFIG += debug_and_release
}

# ensure one "debug" or "release" in CONFIG so they can be used as
# conditionals instead of writing "CONFIG(debug, debug|release)"...
CONFIG(debug, debug|release) {
    CONFIG -= debug release
    CONFIG += debug
    }
CONFIG(release, debug|release) {
        CONFIG -= debug release
        CONFIG += release
}

QT       -= gui

TARGET = libPCSegmentation
TEMPLATE = lib

DEFINES += LIBPCSEGMENTATION_LIBRARY

SOURCES += libPCSegmentation.cpp \
    3dLineDetection/CommonFunctions.cpp \
    3dLineDetection/LineDetection3D.cpp

#DESTDIR_RELEASE= ./../../../build/release
#DESTDIR_DEBUG= ./../../../build/debug
DESTDIR_RELEASE= ./../../../build_osgeo4w/release
DESTDIR_DEBUG= ./../../../build_osgeo4w/debug

PCL_PATH=./../../../depends/PCL.1.8.1
#VTK_LIBS_PATH=./../../../depends/PCL.1.8.1/3rdParty/VTK
BOOST_LIBS_PATH=./../../../depends/PCL.1.8.1/3rdParty/Boost
FLANN_LIBS_PATH=./../../../depends/PCL.1.8.1/3rdParty/FLANN
#EIGEN_PATH= ./../../../depends/eigen-eigen-323c052e1731
OPENCV_PATH = ./../../../depends/OpenCV-4.1.2/build

#BOOST_LIBS_PATH="E:/Librerias/PCL 1.8.1/3rdParty/Boost"
#FLANN_LIBS_PATH="E:/Librerias/PCL 1.8.1/3rdParty/FLANN"
#OPENCV_PATH = E:\Librerias\OpenCV-4.1.2\build

#INCLUDEPATH += "E:/Librerias/PCL 1.8.1/include/pcl-1.8"
#INCLUDEPATH += "E:/Librerias/PCL 1.8.1/3rdParty/Boost/include/boost-1_64"
#INCLUDEPATH += "E:/Librerias/PCL 1.8.1/3rdParty/Eigen/eigen3"
#INCLUDEPATH += "E:/Librerias/PCL 1.8.1/3rdParty/FLANN/include"
INCLUDEPATH += $$OPENCV_PATH\include
INCLUDEPATH += ./3dLineDetection
INCLUDEPATH += $$PCL_PATH/include/pcl-1.8
INCLUDEPATH += $$PCL_PATH/3rdParty/Boost/include/boost-1_64
INCLUDEPATH += $$PCL_PATH/3rdParty/Eigen/eigen3
INCLUDEPATH += $$PCL_PATH/3rdParty/FLANN/include
#INCLUDEPATH += $$PCL_PATH/3rdParty/Qhull/include
#INCLUDEPATH += $$PCL_PATH/3rdParty/VTK/include/vtk-8.0

HEADERS += libPCSegmentation.h \
    libPCSegmentation_global.h \
    3dLineDetection/CommonFunctions.h \
    3dLineDetection/LineDetection3D.h \
    3dLineDetection/nanoflann.hpp \
    3dLineDetection/Timer.h \
    3dLineDetection/utils.h


#LIBS += "-LE:/Librerias/PCL 1.8.1/lib"
LIBS += -L$$PCL_PATH/lib
LIBS += -lUser32 -lGdi32 # candidato a eliminar
debug{
    DESTDIR = $$DESTDIR_DEBUG
    LIBS += -L$$DESTDIR_DEBUG
    LIBS += -lpcl_common_debug
    LIBS += -lpcl_filters_debug
    LIBS += -lpcl_io_debug
    LIBS += -lpcl_segmentation_debug

#    LIBS += -lpcl_features_debug
#    LIBS += -lpcl_io_ply_debug
    LIBS += -lpcl_kdtree_debug
#    LIBS += -lpcl_keypoints_debug
#    LIBS += -lpcl_octree_debug
#    LIBS += -lpcl_registration_debug
#    LIBS += -lpcl_sample_consensus_debug
    LIBS += -lpcl_search_debug
#    LIBS += -lpcl_surface_debug
#    LIBS += -lpcl_tracking_debug
#    LIBS += -lpcl_visualization_debug

    LIBS += $$BOOST_LIBS_PATH\lib\libboost_thread-vc140-mt-gd-1_64.lib
    LIBS += $$BOOST_LIBS_PATH\lib\libboost_system-vc140-mt-gd-1_64.lib
    LIBS += $$BOOST_LIBS_PATH\lib\libboost_date_time-vc140-mt-gd-1_64.lib
    LIBS += $$BOOST_LIBS_PATH\lib\libboost_chrono-vc140-mt-gd-1_64.lib
    LIBS += $$BOOST_LIBS_PATH\lib\libboost_filesystem-vc140-mt-gd-1_64.lib
    LIBS += $$BOOST_LIBS_PATH\lib\libboost_iostreams-vc140-mt-gd-1_64.lib

    LIBS += $$FLANN_LIBS_PATH\lib\flann.lib
    LIBS += $$OPENCV_PATH\x64\vc14\lib\opencv_world412d.lib

}else{
    DESTDIR = $$DESTDIR_RELEASE
    LIBS += -L$$DESTDIR_RELEASE
    LIBS += -lpcl_common_release
    LIBS += -lpcl_filters_release
    LIBS += -lpcl_io_release
    LIBS += -lpcl_segmentation_release

#    LIBS += -lpcl_features_release
#    LIBS += -lpcl_io_ply_release
    LIBS += -lpcl_kdtree_release
#    LIBS += -lpcl_keypoints_release
#    LIBS += -lpcl_octree_release
#    LIBS += -lpcl_registration_release
#    LIBS += -lpcl_sample_consensus_release
    LIBS += -lpcl_search_release
#    LIBS += -lpcl_surface_release
#    LIBS += -lpcl_tracking_debug
#    LIBS += -lpcl_visualization_release

    LIBS += $$BOOST_LIBS_PATH\lib\libboost_thread-vc140-mt-1_64.lib
    LIBS += $$BOOST_LIBS_PATH\lib\libboost_system-vc140-mt-1_64.lib
    LIBS += $$BOOST_LIBS_PATH\lib\libboost_date_time-vc140-mt-1_64.lib
    LIBS += $$BOOST_LIBS_PATH\lib\libboost_chrono-vc140-mt-1_64.lib
    LIBS += $$BOOST_LIBS_PATH\lib\libboost_filesystem-vc140-mt-1_64.lib
    LIBS += $$BOOST_LIBS_PATH\lib\libboost_iostreams-vc140-mt-1_64.lib

    LIBS += $$FLANN_LIBS_PATH\lib\flann.lib
    LIBS += $$OPENCV_PATH\x64\vc14\lib\opencv_world412.lib
}
