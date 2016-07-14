DEFINES += VIEWER_DEFINED

QT += core gui opengl xml widgets

unix:!macx{
    QMAKE_CXXFLAGS += -std=c++11 
    LIBS += /usr/lib/x86_64-linux-gnu/libGLU.so
    LIBS += -lQGLViewer    
}

macx{
    CONFIG += c++11
    INCLUDEPATH += -I /libs/include/boost/
    INCLUDEPATH += /libs/frameworks/QGLViewer/QGLViewer.framework/Headers
    LIBS += -F/libs/frameworks/QGLViewer -framework QGLViewer
}

HEADERS += \
    $$PWD/glcanvas.h \
    $$PWD/mainwindow.h \
    $$PWD/objects/cylinder.h \
    $$PWD/objects/sphere.h \
    $$PWD/objects/drawabledebugobjects.h \
    $$PWD/managers/windowmanager.h
    
SOURCES += \
    $$PWD/glcanvas.cpp \
    $$PWD/mainwindow.cpp \
    $$PWD/objects/drawabledebugobjects.cpp \
    $$PWD/managers/windowmanager.cpp
    
FORMS += \
    $$PWD/mainwindow.ui \
    $$PWD/managers/windowmanager.ui
