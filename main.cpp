/*
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @copyright Alessandro Muntoni 2016.
 */

#define USER_INTERFACE 1

#if USER_INTERFACE==1
#include "GUI/mainwindow.h"
#include "GUI/managers/dcelmanager.h"
#include "GUI/managers/windowmanager.h"
#include "GUI/managers/enginemanager.h"
#include "common.h"
#include <QApplication>
#include <lib/common/comparators.h>
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


    gui.setCurrentIndexToolBox(ENGINE_MANAGER_ID); // il dcel manager sarà quello visualizzato di default
    gui.updateGlCanvas();
    gui.show();

    return app.exec();

    #else
    bool b = false;
    if (argc > 1) {
        b = true;
        Dcel d;
        d.loadFromObjFile(argv[1]);
        Engine::largeScaleFabrication(d);
    }

    return !b;

    #endif
}
