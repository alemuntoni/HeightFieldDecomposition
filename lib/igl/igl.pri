unix:!macx{
    QMAKE_CXXFLAGS += -std=c++11  -fopenmp
    QMAKE_LFLAGS +=  -fopenmp
    QMAKE_CXXFLAGS += -O3 -DNDEBUG
    LIBS += -lboost_system -DBOOST_LOG_DYN_LINK -lboost_log -lboost_thread -lpthread
    INCLUDEPATH -= /usr/include/eigen3
    INCLUDEPATH += $$(LIBIGL)/include/
    INCLUDEPATH += $$(LIBIGL)/external/nanogui/ext/eigen/
    INCLUDEPATH += $$(LIBIGL)/external/nanogui/ext/glfw/include/
}

HEADERS += \
    $$PWD/iglinterface.h

SOURCES += \
    $$PWD/iglinterface.cpp
