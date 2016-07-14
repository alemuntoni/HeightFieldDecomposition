DEFINES += DCEL_DEFINED

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

contains(DEFINES, VIEWER_DEFINED){
    #WARNING: BUG on qmake: comment the following line if viewer is not included
    #include ($$PWD/gui/dcelmanager.pri)
}
