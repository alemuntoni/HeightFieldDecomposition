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
#include "trimesh/gui/trimeshmanager.h"

int main(int argc, char *argv[]) {

    #ifdef SERVER_MODE
    if (argc > 4){
        std::string filename_smooth(argv[1]);
        std::string filename(argv[2]);
        IGLInterface::IGLMesh original;
        original.readFromFile(filename);
        Dcel d;
        BoxList solutions;
        BoxList allSolutions;
        d.loadFromObjFile(filename_smooth);
        BoundingBox bb= d.getBoundingBox();
        double scale = std::stod(argv[3]);
        Engine::scaleAndRotateDcel(d, 0, scale);
        original.scale(bb, d.getBoundingBox());
        double kernelDistance = std::stod(argv[4]);
        double tolerance = 0.0;
        bool file = false;
        bool tol = true;
        bool decimate = true;
        if (argc > 5){
            tolerance = std::stod(argv[5]);
            if (argc > 6){
                file = std::stoi(argv[6]);
                if (argc > 7){
                    tol = std::stoi(argv[7]);
                    if (argc > 8)
                        decimate = std::stoi(argv[8]);
                }
            }
        }
        Engine::optimizeAndDeleteBoxes(solutions, d, kernelDistance, false, Pointd(), tol, true, 0.000, tolerance, file, decimate, allSolutions);
        size_t lastindex = filename_smooth.find_last_of(".");
        std::string rawname = filename_smooth.substr(0, lastindex);

        bool b = true;

        std::ofstream myfile;
        myfile.open (rawname + "0" + std::to_string((int)(kernelDistance*100)) + ".bin", std::ios::out | std::ios::binary);
        d.serialize(myfile);
        Serializer::serialize(b, myfile);
        solutions.serialize(myfile);
        Serializer::serialize(b, myfile);
        original.serialize(myfile);
        Serializer::serialize(scale, myfile);
        Serializer::serialize(kernelDistance, myfile);
        myfile.close();

        std::ofstream mysecondfile;
        mysecondfile.open (rawname + "0" + std::to_string((int)kernelDistance*100) + "_allsolutions.bin", std::ios::out | std::ios::binary);
        d.serialize(mysecondfile);
        Serializer::serialize(b, mysecondfile);
        allSolutions.serialize(mysecondfile);
        Serializer::serialize(b, mysecondfile);
        original.serialize(mysecondfile);
        Serializer::serialize(scale, mysecondfile);
        Serializer::serialize(kernelDistance, mysecondfile);
        mysecondfile.close();
    }
    else
        std::cerr << "Error! Number argument lower than 4\n";
    /*if (argc > 3){
        std::ofstream myfile;
        myfile.open (argv[1], std::ios::out | std::ios::binary | std::ios::app);
        double factor = std::stod(argv[2]);
        double kernel = std::stod(argv[3]);
        Serializer::serialize(factor, myfile);
        Serializer::serialize(kernel, myfile);
        myfile.close();
    }*/
    #else
    #ifdef CONVERTER_MODE
    if (argc > 2){
        std::string filename(argv[1]);
        int file = std::stoi(argv[2]);
        if (file == 0){
            //deserialize
            std::ifstream inputfile;
            inputfile.open (filename, std::ios::in | std::ios::binary);
            DrawableDcel d;
            BoxList solutions;
            IGLInterface::DrawableIGLMesh originalMesh;
            if (! d.deserialize(inputfile)) return false;
            bool bb = false;
            if (! Serializer::deserialize(bb, inputfile)) return false;
            if (bb){
                if (! solutions.deserialize(inputfile)) return false;
            }
            if (Serializer::deserialize(bb, inputfile)){
                originalMesh.deserialize(inputfile);
            }
            inputfile.close();
            //serialize

            std::ofstream binaryFile;
            binaryFile.open (filename, std::ios::out | std::ios::binary);
            d.serialize(binaryFile);
            bool b = true;
            Serializer::serialize(b, binaryFile);
            solutions.serialize(binaryFile);
            Serializer::serialize(bb, binaryFile);
            if (bb)
                originalMesh.serialize(binaryFile);
            binaryFile.close();
        }
        else {
            //deserializeBC
            std::ifstream inputfile;
            inputfile.open (filename, std::ios::in | std::ios::binary);
            DrawableDcel d;
            BoxList solutions;
            IGLInterface::DrawableIGLMesh baseComplex;
            HeightfieldsList he;
            IGLInterface::DrawableIGLMesh originalMesh;
            d.deserialize(inputfile);
            solutions.deserialize(inputfile);
            baseComplex.deserialize(inputfile);
            he.deserialize(inputfile);
            originalMesh.deserialize(inputfile);
            inputfile.close();

            //serializeBC
            std::ofstream binaryFile;
            binaryFile.open (filename, std::ios::out | std::ios::binary);
            d.serialize(binaryFile);
            solutions.serialize(binaryFile);
            baseComplex.serialize(binaryFile);
            he.serialize(binaryFile);
            originalMesh.serialize(binaryFile);
            binaryFile.close();

        }
    }
    #else

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

    TrimeshManager tm(&gui);
    gui.addManager(&tm, "Trimesh Manager");

    gui.setCurrentIndexToolBox(ENGINE_MANAGER_ID); // il dcel manager sarà quello visualizzato di default
    gui.updateGlCanvas();
    gui.show();

    return app.exec();
    #endif
    #endif
    return 0;
}
