DEFINES += IGL_DEFINED

contains(DEFINES, CGAL_DEFINED){
} else {
unix:!macx{
    LIBS += -lmpfr -lgmp -lCGAL -frounding-math
}

macx{
    DEFINES += CGAL_DEFINED
    INCLUDEPATH += -I /libs/include/CGAL/
    LIBS += -frounding-math
    LIBS += -L/libs/lib/gmp -lgmp
    LIBS += -L/libs/lib/CGAL -lCGAL
}
}

unix:!macx{
    QMAKE_CXXFLAGS += -std=c++11  -fopenmp
    QMAKE_LFLAGS +=  -fopenmp
    #QMAKE_CXXFLAGS += -O3 -DNDEBUG
    LIBS += -lboost_system -DBOOST_LOG_DYN_LINK -lboost_log -lboost_thread -lpthread
    INCLUDEPATH -= /usr/include/eigen3
    INCLUDEPATH += $$(LIBIGL)/include/
    INCLUDEPATH += $$(LIBIGL)/external/nanogui/ext/eigen/
    INCLUDEPATH += $$(LIBIGL)/external/nanogui/ext/glfw/include/
}

HEADERS += \
    $$PWD/iglinterface.h \
    $$PWD/iglmesh.h \
    $$PWD/drawableiglmesh.h

SOURCES += \
    $$PWD/iglinterface.cpp \
    $$PWD/iglmesh.cpp \
    $$PWD/drawableiglmesh.cpp


contains(DEFINES, VIEWER_DEFINED){
    #WARNING: BUG on qmake: comment the following line if viewer is not included
    include ($$PWD/gui/iglmeshmanager.pri)
}
