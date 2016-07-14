DEFINES += TRIMESH_DEFINED

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

contains(DEFINES, VIEWER_DEFINED){
    #WARNING: BUG on qmake: comment the following line if viewer is not included
    include ($$PWD/gui/trimeshmanager.pri)
}
