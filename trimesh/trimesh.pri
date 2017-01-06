DEFINES += TRIMESH_DEFINED
MODULES +=  TRIMESH

!contains(DEFINES, COMMON_DEFINED){
    error(Trimesh module requires common module!)
}

HEADERS += \
    $$PWD/trimesh.h \
    $$PWD/load_save_trimesh.h

SOURCES += \
    $$PWD/load_save_trimesh.cpp \

contains(DEFINES, VIEWER_DEFINED){
    #WARNING: BUG on qmake: comment the following line if viewer is not included
    include ($$PWD/gui/trimeshmanager.pri)
}
