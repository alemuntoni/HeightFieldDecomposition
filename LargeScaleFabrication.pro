QT += core gui opengl xml widgets

CONFIG(debug, debug|release){
    message(debug)
    DEFINES += DEBUG
}
CONFIG(release, debug|release){
    message(release)
    DEFINES += DEBUG
}

unix:!macx{
    QMAKE_CXXFLAGS += -std=c++11  -fopenmp
    LIBS += /usr/lib/x86_64-linux-gnu/libGLU.so
    LIBS += -lboost_system -DBOOST_LOG_DYN_LINK -lboost_log -lboost_thread -lpthread
    LIBS += -lQGLViewer
    LIBS += -lmpfr -lgmp -lCGAL -frounding-math
    INCLUDEPATH += -I /usr/include/eigen3
    DEFINES += CGAL_DEFINED
}

macx{
    CONFIG += c++11
    DEFINES += CGAL_DEFINED
    #QMAKE_CXXFLAGS += -stdlib=libc++
    INCLUDEPATH += -I /libs/include/boost/
    INCLUDEPATH += -I /libs/include/CGAL/
    INCLUDEPATH += /libs/frameworks/QGLViewer/QGLViewer.framework/Headers
    LIBS += -frounding-math
    LIBS += -L/libs/lib/gmp -lgmp
    LIBS += -F/libs/frameworks/QGLViewer -framework QGLViewer
    LIBS += -L/libs/lib/CGAL -lCGAL
}

HEADERS += \
    GUI/managers/dcelmanager.h \
    GUI/objects/cylinder.h \
    GUI/objects/sphere.h \
    GUI/glcanvas.h \
    GUI/mainwindow.h \
    lib/common/bounding_box.h \
    lib/common/comparators.h \
    lib/common/drawable_object.h \
    lib/common/point.h \
    lib/dcel/dcel.h \
    lib/dcel/dcel_face.h \
    lib/dcel/dcel_face_iterators.h \
    lib/dcel/dcel_half_edge.h \
    lib/dcel/dcel_iterators.h \
    lib/dcel/dcel_struct.h \
    lib/dcel/dcel_vertex.h \
    lib/dcel/dcel_vertex_iterators.h \
    lib/dcel/drawable_dcel.h \
    lib/common/common.h \
    GUI/managers/windowmanager.h \
    common.h \
    lib/common/serialize.h \
    lib/common/pickable_object.h \
    lib/dcel/pickable_dcel.h \
    lib/grid/grid.h \
    lib/common/drawabledebugobjects.h \
    GUI/managers/enginemanager.h \
    lib/grid/drawablegrid.h

SOURCES += \
    lib/dcel/dcel_face.cpp \
    lib/dcel/dcel_half_edge.cpp \
    lib/dcel/dcel_vertex.cpp \
    lib/dcel/dcel_struct.cpp \
    main.cpp \
    lib/dcel/drawable_dcel.cpp \
    GUI/glcanvas.cpp \
    GUI/mainwindow.cpp \
    GUI/managers/dcelmanager.cpp \
    GUI/managers/windowmanager.cpp \
    common.cpp \
    lib/dcel/pickable_dcel.cpp \
    lib/grid/grid.cpp \
    lib/common/drawabledebugobjects.cpp \
    GUI/managers/enginemanager.cpp \
    lib/grid/drawablegrid.cpp

FORMS += \
    GUI/mainwindow.ui \
    GUI/managers/dcelmanager.ui \
    GUI/managers/windowmanager.ui \
    GUI/managers/enginemanager.ui

DISTFILES += \
    README.txt
