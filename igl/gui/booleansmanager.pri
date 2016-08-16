MODULES +=  BOOLEANS_MANAGER

!contains(DEFINES, VIEWER_DEFINED){
    error(BooleansManager module requires Viewer module!)
}

!contains(DEFINES, CGAL_DEFINED){
    error(BooleansManager module requires Cgal module!)
}

FORMS += \
    $$PWD/booleansmanager.ui

HEADERS += \
    $$PWD/booleansmanager.h

SOURCES += \
    $$PWD/booleansmanager.cpp
