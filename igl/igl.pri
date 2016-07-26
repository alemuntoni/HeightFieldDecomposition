DEFINES += IGL_DEFINED
CONFIG += IGL_DEFINED
MODULES += IGL

!contains(DEFINES, COMMON_DEFINED){
    error(Igl module requires common module!)
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
    $$PWD/iglmesh.h

SOURCES += \
    $$PWD/iglinterface.cpp \
    $$PWD/iglmesh.cpp

#Note: QtCreator always shows this module included, however files included in it are compiled only if Viewer module is included
VIEWER_DEFINED{
    include ($$PWD/gui/iglmeshmanager.pri)
    include ($$PWD/gui/booleansmanager.pri)
}
