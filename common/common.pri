DEFINES += COMMON_DEFINED

unix:!macx{
    QMAKE_CXXFLAGS += -std=c++11
    exists(/usr/include/eigen3){
        DEFINES += COMMON_WITH_EIGEN
        INCLUDEPATH += -I /usr/include/eigen3
    }
}

macx{
    CONFIG += c++11
    exists(/libs/include/eigen3){
        DEFINES += COMMON_WITH_EIGEN
        INCLUDEPATH += -I /libs/include/eigen3/
    }
}

HEADERS += \
    $$PWD/bounding_box.h \
    $$PWD/comparators.h \
    $$PWD/drawable_object.h \
    $$PWD/point.h \
    $$PWD/point2d.h \
    $$PWD/common.h \
    $$PWD/serialize.h \
    $$PWD/pickable_object.h \
    $$PWD/arrays.h \
    $$PWD/timer.h
