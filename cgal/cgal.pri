DEFINES += CGAL_DEFINED

unix:!macx{
    LIBS += -lmpfr -lgmp -lCGAL -frounding-math
}

macx{
    DEFINES += CGAL_DEFINED
    INCLUDEPATH += -I /libs/include/CGAL/
    LIBS += -frounding-math
    LIBS += -L/libs/lib/gmp -lgmp
    LIBS += -L/libs/lib/CGAL -lCGAL
}

HEADERS += \
    $$PWD/aabbtree.h \
    $$PWD/cgalinterface.h

SOURCES += \
    $$PWD/aabbtree.cpp \
    $$PWD/cgalinterface.cpp
