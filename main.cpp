/*
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @copyright Alessandro Muntoni 2016.
 */

#include "viewer/mainwindow.h"
#include "dcel/gui/dcelmanager.h"
//#include "igl/gui/iglmeshmanager.h"
#include "igl/gui/booleansmanager.h"
#include "viewer/managers/windowmanager.h"
#include "GUI/managers/enginemanager.h"
#include "common.h"
#include <QApplication>
//#include "common/comparators.h"
#include "engine/engine.h"

int main(int argc, char *argv[]) {

    #ifndef SERVER_MODE

    QApplication app(argc, argv);

    MainWindow gui;  // finestra principale, contiene la canvas di QGLViewer

    // Creo un window manager e lo aggiungo alla mainwindow
    WindowManager wm(&gui);
    WINDOW_MANAGER_ID = gui.addManager(&wm, "Window");

    // Creo un dcel manager e lo aggiungo alla mainwindow
    DcelManager d(&gui);
    DCEL_MANAGER_ID = gui.addManager(&d, "Dcel");

    EngineManager e(&gui);
    ENGINE_MANAGER_ID = gui.addManager(&e, "Engine");

    //IGLMeshManager mm(&gui);
    //gui.addManager(&mm, "IGL Mesh Manager");

    BooleansManager bm(&gui);
    gui.addManager(&bm, "Booleans Manager");

    gui.setCurrentIndexToolBox(ENGINE_MANAGER_ID); // il dcel manager sarà quello visualizzato di default
    gui.updateGlCanvas();
    gui.show();

    return app.exec();

    #else
    if (argc > 4){
        std::string filename_smooth(argv[1]);
        std::string filename(argv[2]);
        IGLInterface::IGLMesh original;
        original.readFromFile(filename);
        Dcel d;
        BoxList solutions;
        d.loadFromObjFile(filename_smooth);
        BoundingBox bb= d.getBoundingBox();
        int scale = std::stoi(argv[3]);
        Engine::scaleAndRotateDcel(d, 0, scale);
        original.scale(bb, d.getBoundingBox());
        double kernelDistance = std::stod(argv[4]);
        Engine::createAndMinimizeAllBoxes(solutions, d, kernelDistance, true, true, 0.000, 0.05);
        size_t lastindex = filename_smooth.find_last_of(".");
        std::string rawname = filename_smooth.substr(0, lastindex);
        std::ofstream myfile;
        myfile.open (rawname + std::to_string((int)kernelDistance) + ".bin", std::ios::out | std::ios::binary);
        d.serialize(myfile);
        bool b = true;
        Serializer::serialize(b, myfile);
        solutions.serialize(myfile);
        Serializer::serialize(b, myfile);
        original.serialize(myfile);
        myfile.close();
    }
    else
        std::cerr << "Error! Number argument lower than 4\n";
    #endif
    return 0;
}
