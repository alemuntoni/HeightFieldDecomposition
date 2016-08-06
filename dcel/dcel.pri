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
    $$PWD/dcel_vertex_iterators.h


SOURCES += \
    $$PWD/dcel_face.cpp \
    $$PWD/dcel_half_edge.cpp \
    $$PWD/dcel_vertex.cpp \
    $$PWD/dcel_struct.cpp \
    $$PWD/dcel_iterators_inline.cpp \
    $$PWD/dcel_vertex_iterators_inline.cpp \
    $$PWD/dcel_inline.cpp \
    $$PWD/dcel_face_iterators_inline.cpp \
    $$PWD/dcel_vertex_inline.cpp \
    $$PWD/dcel_face_inline.cpp \
    $$PWD/dcel_half_edge_inline.cpp

#Note: QtCreator always shows this module included, however files included in it are compiled only if Viewer module is included
VIEWER_DEFINED{
    include ($$PWD/gui/dcelmanager.pri)
}
