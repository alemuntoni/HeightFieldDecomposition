CONFIG(debug, debug|release){
    DEFINES += DEBUG
}
CONFIG(release, debug|release){
    DEFINES -= DEBUG
    #just uncomment next lines if you want to ignore asserts and got a more optimized binary
    CONFIG += FINAL_RELEASE
}

#CONFIG += c++14

CONFIG += ALL
#CONFIG += SERVER_MODE
#CONFIG += CONVERTER_MODE

ALL {
    CONFIG += USE_LIBIGL_EIGEN
    #CONFIG += MULTI_LABEL_OPTIMIZATION
    CONFIG += CG3_ALL
    include(cg3lib/cg3.pri)
}

SERVER_MODE {
    DEFINES += SERVER_MODE
    CONFIG += FINAL_RELEASE
}

SERVER_HOME {
    DEFINES += SERVER_HOME
    CONFIG += FINAL_RELEASE
}

SERVER_AFTER {
    DEFINES += SERVER_AFTER
    CONFIG += FINAL_RELEASE
}

CONVERTER_MODE {
    DEFINES += CONVERTER_MODE
}

message(Included modules: $$MODULES)
FINAL_RELEASE {
    message(Final Release!)
}

FINAL_RELEASE {
    unix:!macx{
        QMAKE_CXXFLAGS_RELEASE -= -g -O2
        QMAKE_CXXFLAGS += -O3 -DNDEBUG
    }
}

exists($$(GUROBI_HOME)){
    message (Gurobi)
    INCLUDEPATH += $$(GUROBI_HOME)/include
    LIBS += -L$$(GUROBI_HOME)/lib -lgurobi_g++5.2 -lgurobi70
    DEFINES += GUROBI_DEFINED
}


MULTI_LABEL_OPTIMIZATION {
    DEFINES += MULTI_LABEL_OPTIMIZATION_INCLUDED
    INCLUDEPATH += $$PWD/lib/multi_label_optimization
    DEPENDPATH += $$PWD/lib/multi_label_optimization
    SOURCES += \
        lib/multi_label_optimization/GCoptimization.cpp \
        lib/multi_label_optimization/graph.cpp \
        lib/multi_label_optimization/LinkedBlockList.cpp \
        lib/multi_label_optimization/maxflow.cpp \
        lib/multi_label_optimization/example.cpp
}

HEADERS += \
    common.h \
    GUI/managers/enginemanager.h \
    engine/tricubic.h \
    engine/energy.h \
    engine/box.h \
    engine/boxlist.h \
    engine/engine.h \
    engine/heightfieldslist.h \
    engine/packing.h \
    engine/splitting.h \
    engine/reconstruction.h \
    lib/grid/grid.h \
    lib/grid/drawablegrid.h \
    lib/grid/drawableirregulargrid.h \
    lib/grid/irregulargrid.h \
    lib/packing/binpack2d.h \
    lib/dcel_segmentation/chart.h \
    lib/dcel_segmentation/segmentation_iterators.h \
    lib/dcel_segmentation/segmentation_struct.h \
    lib/dcel_segmentation/segmentation.h \
    engine/orientation.h \
    lib/graph/bipartitegraph.h \
    lib/graph/undirectednode.h \
    lib/graph/bipartitegraphiterators.h \
    lib/graph/directedgraph.h \
    lib/csgtree/aabbcsgtree.h \
    lib/octree/octree_node.h \
    engine/tinyfeaturedetection.h \
    fouraxischecker/polylinesCheck.h \
    GUI/managers/fouraxischeckermanager.h \
    fouraxischecker/fouraxischecker.h

SOURCES += \
    main.cpp \
    common.cpp \
    GUI/managers/enginemanager.cpp \
    engine/tricubic.cpp \
    engine/energy.cpp \
    engine/box.cpp \
    engine/boxlist.cpp \
    engine/engine.cpp \
    engine/heightfieldslist.cpp \
    engine/packing.cpp \
    engine/splitting.cpp \
    engine/reconstruction.cpp \
    lib/grid/grid.cpp \
    lib/grid/drawablegrid.cpp \
    lib/grid/drawableirregulargrid.cpp \
    lib/dcel_segmentation/chart.cpp \
    lib/dcel_segmentation/segmentation_struct.cpp \
    engine/orientation.cpp \
    lib/csgtree/aabbcsgtree.cpp \
    lib/octree/octree_node.cpp \
    engine/tinyfeaturedetection.cpp \
    fouraxischecker/polylinesCheck.cpp \
    GUI/managers/fouraxischeckermanager.cpp \
    fouraxischecker/fouraxischecker.cpp

FORMS += \
    GUI/managers/enginemanager.ui \
    GUI/managers/fouraxischeckermanager.ui

DISTFILES += \
    README.txt
