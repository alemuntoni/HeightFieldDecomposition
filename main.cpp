/*
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @copyright Alessandro Muntoni 2016.
 */

#define USER_INTERFACE 1

#if USER_INTERFACE==1
#include "viewer/mainwindow.h"
#include "dcel/gui/dcelmanager.h"
#include "igl/gui/iglmeshmanager.h"
#include "viewer/managers/windowmanager.h"
#include "GUI/managers/enginemanager.h"
#include "common.h"
#include <QApplication>
#include <common/comparators.h>
#else
#include "engine/engine.h"
#endif



int main(int argc, char *argv[]) {

    #if USER_INTERFACE==1

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

    IGLMeshManager mm(&gui);
    gui.addManager(&mm, "IGL Mesh Manager");


    gui.setCurrentIndexToolBox(ENGINE_MANAGER_ID); // il dcel manager sarà quello visualizzato di default
    gui.updateGlCanvas();
    gui.show();

    return app.exec();

    #else
    /*bool b = false;
    if (argc > 1) {
        b = true;
        Dcel d;
        d.loadFromObjFile(argv[1]);
        Engine::largeScaleFabrication(d);
    }

    return !b;*/

    //Dcel d;
    //d.loadFromObjFile(argv[1]);
    //Engine::makePreprocessingAndSave(d, argv[2], 15);
    Engine::Server::expandBoxesFromPreprocessing(argv[1], argv[2]);

    #endif
}
