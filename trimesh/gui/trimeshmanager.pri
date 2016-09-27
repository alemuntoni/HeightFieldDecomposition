!contains(DEFINES, VIEWER_DEFINED){
    error(TrimeshManager module requires Viewer module!)
}

FORMS += \
    $$PWD/trimeshmanager.ui

HEADERS += \
    $$PWD/trimeshmanager.h \
    $$PWD/drawable_trimesh.h

SOURCES += \
    $$PWD/trimeshmanager.cpp \
    $$PWD/drawable_trimesh.cpp

