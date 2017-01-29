DEFINES += TRIMESH_DEFINED
MODULES +=  TRIMESH

!contains(DEFINES, COMMON_DEFINED){
    error(Trimesh module requires common module!)
}

HEADERS += \
    $$PWD/trimesh/trimesh.h \
    $$PWD/trimesh/load_save_trimesh.h

SOURCES += \
    $$PWD/trimesh/load_save_trimesh.cpp \

contains(DEFINES, VIEWER_DEFINED){
    #WARNING: BUG on qmake: comment the following line if viewer is not included
    include ($$PWD/trimesh/gui/trimeshmanager.pri)
}

INCLUDEPATH += $$PWD
