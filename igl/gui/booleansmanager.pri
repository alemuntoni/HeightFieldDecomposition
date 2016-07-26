MODULES +=  BOOLEANS_MANAGER

!contains(DEFINES, VIEWER_DEFINED){
    error(IglMeshManager module requires Viewer module!)
}

FORMS += \
    $$PWD/booleansmanager.ui

HEADERS += \
    $$PWD/booleansmanager.h

SOURCES += \
    $$PWD/booleansmanager.cpp
