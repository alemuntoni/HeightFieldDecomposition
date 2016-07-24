MODULES +=  IGL_MESH_MANAGER

!contains(DEFINES, VIEWER_DEFINED){
    error(IglMeshManager module requires Viewer module!)
}

FORMS += \
    $$PWD/iglmeshmanager.ui

HEADERS += \
    $$PWD/iglmeshmanager.h \
    $$PWD/drawableiglmesh.h

SOURCES += \
    $$PWD/iglmeshmanager.cpp \
    $$PWD/drawableiglmesh.cpp
