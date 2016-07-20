DEFINES += TRIMESH_DEFINED
CONFIG += TRIMESH_DEFINED
MODULES += TRIMESH

!contains(DEFINES, COMMON_DEFINED){
    error(Trimesh module requires common module!)
}

HEADERS += \
    $$PWD/trimesh.h \
    $$PWD/load_save_trimesh.h \
    $$PWD/drawable_trimesh.h

SOURCES += \
    $$PWD/load_save_trimesh.cpp \
    $$PWD/drawable_trimesh.cpp

#Note: QtCreator always shows this module included, however files included in it are compiled only if Viewer module is included
VIEWER_DEFINED{
    include ($$PWD/gui/trimeshmanager.pri)
}
