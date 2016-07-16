DEFINES += DCEL_DEFINED
CONFIG += DCEL_DEFINED
MODULES +=  DCEL

!contains(DEFINES, COMMON_DEFINED){
    error(Dcel module requires common module!)
}


HEADERS += \
    $$PWD/dcel.h \
    $$PWD/dcel_face.h \
    $$PWD/dcel_face_iterators.h \
    $$PWD/dcel_half_edge.h \
    $$PWD/dcel_iterators.h \
    $$PWD/dcel_struct.h \
    $$PWD/dcel_vertex.h \
    $$PWD/dcel_vertex_iterators.h \
    $$PWD/drawable_dcel.h \
    $$PWD/pickable_dcel.h


SOURCES += \
    $$PWD/dcel_face.cpp \
    $$PWD/dcel_half_edge.cpp \
    $$PWD/dcel_vertex.cpp \
    $$PWD/dcel_struct.cpp \
    $$PWD/drawable_dcel.cpp \
    $$PWD/pickable_dcel.cpp

#Note: QtCreator always shows this module included, however files included in it are compiled only if Viewer module is included
VIEWER_DEFINED{
    include ($$PWD/gui/dcelmanager.pri)
}
