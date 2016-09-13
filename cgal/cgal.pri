DEFINES += CGAL_DEFINED
CONFIG += CGAL_DEFINED
MODULES += CGAL

!contains(DEFINES, COMMON_DEFINED){
    error(Cgal module requires common module!)
}

unix:!macx{
    LIBS += -lmpfr -lgmp -lCGAL -frounding-math
    LIBS += -lboost_system -DBOOST_LOG_DYN_LINK -lboost_log -lboost_thread -lpthread
}

macx{
    DEFINES += CGAL_DEFINED
    INCLUDEPATH += -I /libs/include/CGAL/
    LIBS += -frounding-math
    LIBS += -L/libs/lib/gmp -lgmp
    LIBS += -L/libs/lib/CGAL -lCGAL
}

contains(DEFINES, COMMON_WITH_EIGEN){
    DEFINES += CGAL_EIGEN3_ENABLED
}

HEADERS += \
    $$PWD/aabbtree.h \
    $$PWD/cgalinterface.h

SOURCES += \
    $$PWD/aabbtree.cpp \
    $$PWD/cgalinterface.cpp
