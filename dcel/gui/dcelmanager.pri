!contains(DEFINES, VIEWER_DEFINED){
    error(DcelManager module requires Viewer module!)
}

HEADERS += \
    $$PWD/dcelmanager.h

SOURCES += \
    $$PWD/dcelmanager.cpp

FORMS += \
    $$PWD/dcelmanager.ui
