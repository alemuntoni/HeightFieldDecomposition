CONFIG(debug, debug|release){
    DEFINES += DEBUG
}
CONFIG(release, debug|release){
    DEFINES -= DEBUG
    #just uncomment next lines if you want to ignore asserts and got a more optimized binary
    #CONFIG += FINAL_RELEASE
}

CONFIG += ALL
#CONFIG += SERVER_MODE
#CONFIG += CONVERTER_MODE

ALL {
    #Add or remove all the modules you need
    #Before pushing the project with your new module, please double check that everything works keeping uncommentend
    #only the modules that are required by your module. Also please write here required and optional modules for your module

    #Common module: contains classes and common functions used on all the other modules
    #Optional: Eigen
    include (common/common.pri)

    #Viewer module: contains classes for a simple viewer
    #Requires: Common module, libQGLViewer, boost
    include (viewer/viewer.pri)

    #Dcel module: contains a Double Connected-Edge List data structure
    #Requires: Common module, boost;
    #Optional: Cgal module, viewer module
    include (dcel/dcel.pri)

    #Cgal module: contains an interface to some functionalities of CGAL library
    #Requires: Common module, libCgal; Optional: Dcel module
    include (cgal/cgal.pri)

    #Trimesh module: contains a Trimesh data structure
    #Requires: Common module
    #Optional: Viewer module
    include (trimesh/trimesh.pri)

    include (eigenmesh/eigenmesh.pri)

    #Igl module: coontaint an intergace to some functionalities of libIGL
    #Requires: Common module, libIGL (an environment variable named LIBIGL containing the root directory of the library must be setted)
    #Optional: Viewer module, Cgal module
    include (igl/igl.pri)

    include(lib/cinolib.pri)

    HEADERS += \
        common.h \
        GUI/managers/enginemanager.h \
        GUI/managers/engineworker.h \
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
        lib/graph/graph.h \
        lib/graph/directedgraph.h \
        lib/packing/binpack2d.h \
        lib/dcel_segmentation/chart.h \
        lib/dcel_segmentation/segmentation_iterators.h \
        lib/dcel_segmentation/segmentation_struct.h \
        lib/dcel_segmentation/segmentation.h

    SOURCES += \
        main.cpp \
        common.cpp \
        GUI/managers/enginemanager.cpp \
        GUI/managers/engineworker.cpp \
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
        lib/dcel_segmentation/segmentation_struct.cpp

    FORMS += \
        GUI/managers/enginemanager.ui

    DISTFILES += \
        README.txt
}

SERVER_MODE {
    DEFINES += SERVER_MODE
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
