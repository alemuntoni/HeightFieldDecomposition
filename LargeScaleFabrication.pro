QT += core gui opengl xml widgets

CONFIG(debug, debug|release){
    DEFINES += DEBUG
}
CONFIG(release, debug|release){
    DEFINES -= DEBUG
    unix:!macx{ #just uncomment next lines if you want to ignore asserts and got a more optimized binary
        #QMAKE_CXXFLAGS_RELEASE -= -g
        #QMAKE_CXXFLAGS += -O3 -DNDEBUG
    }
}

unix:!macx{
    QMAKE_CXXFLAGS += -std=c++11  -fopenmp
    QMAKE_LFLAGS +=  -fopenmp
    LIBS += /usr/lib/x86_64-linux-gnu/libGLU.so
    LIBS += -lboost_system -DBOOST_LOG_DYN_LINK -lboost_log -lboost_thread -lpthread
    LIBS += -lQGLViewer    
    INCLUDEPATH += -I /usr/include/eigen3    
}

macx{
    CONFIG += c++11
    DEFINES += CGAL_DEFINED
    #QMAKE_CXXFLAGS += -stdlib=libc++
    INCLUDEPATH += -I /libs/include/boost/
    INCLUDEPATH += /libs/frameworks/QGLViewer/QGLViewer.framework/Headers
    LIBS += -F/libs/frameworks/QGLViewer -framework QGLViewer
}

HEADERS += \
    GUI/managers/dcelmanager.h \
    common.h \
    lib/grid/grid.h \
    GUI/managers/enginemanager.h \
    lib/grid/drawablegrid.h \
    engine/tricubic.h \
    engine/energy.h \
    engine/box.h \
    engine/boxlist.h \
    engine/engine.h

SOURCES += \
    main.cpp \
    GUI/managers/dcelmanager.cpp \
    common.cpp \
    lib/grid/grid.cpp \
    GUI/managers/enginemanager.cpp \
    lib/grid/drawablegrid.cpp \
    engine/tricubic.cpp \
    engine/energy.cpp \
    engine/box.cpp \
    engine/boxlist.cpp \
    engine/engine.cpp

FORMS += \
    GUI/managers/dcelmanager.ui \
    GUI/managers/enginemanager.ui

DISTFILES += \
    README.txt

include(lib/common/common.pri)

include(lib/cgal_interface/cgal.pri)

include(lib/igl_interface/igl.pri)

include (lib/dcel/dcel.pri)

include(GUI/viewer.pri)
