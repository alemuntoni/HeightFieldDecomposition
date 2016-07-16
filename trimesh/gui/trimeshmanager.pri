MODULES += TRIMESH_MANAGER

!contains(DEFINES, VIEWER_DEFINED){
    error(TrimeshManager module requires Viewer module!)
}

FORMS += \
    $$PWD/trimeshmanager.ui

HEADERS += \
    $$PWD/trimeshmanager.h

SOURCES += \
    $$PWD/trimeshmanager.cpp
