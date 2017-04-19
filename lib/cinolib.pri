TEMPLATE         = app
CONFIG          += c++11
CONFIG          += qt opengl
CONFIG          -= app_bundle
macx{
    QMAKE_CXXFLAGS   = -Wno-c++11-extensions
}
INCLUDEPATH     += $$(CINOLIB_HOME) #-> link to cinolib
