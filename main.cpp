/*
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @copyright Alessandro Muntoni 2016.
 */

#include "viewer/mainwindow.h"
#include "dcel/gui/dcelmanager.h"
#include "viewer/managers/windowmanager.h"
#include "GUI/managers/enginemanager.h"
#include "common.h"
#include <QApplication>
//#include "common/comparators.h"
#include "engine/engine.h"
//#include "trimesh/gui/trimeshmanager.h"
#include "lib/graph/directedgraph.h"

#include "eigenmesh/gui/eigenmeshmanager.h"
#include "eigenmesh/gui/booleansmanager.h"

#include "lib/graph/bipartitegraph.h"
#include <typeinfo>       // operator typeid
#include <lib/csgtree/aabbcsgtree.h>


int main(int argc, char *argv[]) {
    //std::cerr << sizeof(double) << "\n";
    /*BipartiteGraph<int, float> cacca;
    cacca.addUNode(1);
    cacca.addUNode(2);
    cacca.addUNode(3);
    cacca.addUNode(4);
    cacca.addUNode(5);
    cacca.addVNode(1.5);
    cacca.addVNode(2.5);
    cacca.addVNode(3.5);
    cacca.addVNode(4.5);
    cacca.addVNode(5.5);
    cacca.addVNode(6.5);
    cacca.addArc(1, 2.5);
    cacca.addArc(1, 4.5);
    cacca.addArc(2, 1.5);
    cacca.addArc(2, 2.5);
    cacca.addArc(3, 2.5);
    cacca.addArc(3, 3.5);
    cacca.addArc(4, 2.5);
    cacca.addArc(4, 4.5);
    cacca.addArc(4, 6.5);
    cacca.addArc(5, 4.5);
    cacca.addArc(5, 5.5);
    cacca.addArc(5, 6.5);

    cacca.deleteVNode(5.5);
    for (const float& f : cacca.vNodeIterator()){
        std::cerr << f << "; ";
    }
    std::cerr << "\n";
    for (int i : cacca.uNodeIterator()){
        std::cerr << i << "; ";
    }
    std::cerr << "\n";*/

    /*Graph g(6);
    g.addEdge(0,1);
    g.addEdge(0,3);
    g.addEdge(1,2);
    g.addEdge(1,4);
    g.addEdge(2,0);
    g.addEdge(2,5);
    g.addEdge(3,0);
    g.addEdge(3,5);
    g.addEdge(4,3);
    g.addEdge(4,1);
    g.addEdge(5,4);
    g.addEdge(5,2);

    std::vector< std::vector<unsigned int>> circuits =  g.getCircuits();
    for (unsigned int i = 0; i < circuits.size(); i++){
        std::cout << "Circuit " << i << ": ";
        for (unsigned int j = 0; j < circuits[i].size(); j++){
            std::cout << circuits[i][j] << "; ";
        }
        std::cout << "\n";
    }*/

    //buddha
    /*Graph g(9);
    g.addEdge(0,7);
    g.addEdge(2,1);
    g.addEdge(3,1);
    g.addEdge(5,1);
    g.addEdge(8,1);
    g.addEdge(2,4);
    g.addEdge(4,2);
    g.addEdge(3,4);
    g.addEdge(4,3);
    g.addEdge(3,8);
    g.addEdge(4,5);
    g.addEdge(5,4);
    g.addEdge(4,6);
    g.addEdge(8,6);
    std::vector< std::vector<unsigned int>> scc = g.getStronglyConnectedComponents();
    for (unsigned int i = 0; i < scc.size(); i++){
        std::cout << "Scc " << i << ": ";
        for (unsigned int j = 0; j < scc[i].size(); j++){
            std::cout << scc[i][j] << "; ";
        }
        std::cout << "\n";
    }


    std::vector< std::vector<unsigned int>> circuits =  g.getCircuits();
    for (unsigned int i = 0; i < circuits.size(); i++){
        std::cout << "Circuit " << i << ": ";
        for (unsigned int j = 0; j < circuits[i].size(); j++){
            std::cout << circuits[i][j] << "; ";
        }
        std::cout << "\n";
    }*/


    #ifdef SERVER_MODE
    if (argc > 4){
        std::string filename_smooth(argv[1]);
        std::string filename(argv[2]);
        EigenMesh original;
        original.readFromObj(filename);
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
        bool tol = false;
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
            DrawableEigenMesh originalMesh;
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
            DrawableEigenMesh baseComplex;
            HeightfieldsList he;
            DrawableEigenMesh originalMesh;
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

    BooleansManager bm(&gui);
    gui.addManager(&bm, "Booleans Manager");

    //TrimeshManager tm(&gui);
    //gui.addManager(&tm, "Trimesh Manager");

    EigenMeshManager em(&gui);
    gui.addManager(&em, "EigenMesh Manager");

    gui.setCurrentIndexToolBox(ENGINE_MANAGER_ID); // il dcel manager sarà quello visualizzato di default
    gui.updateGlCanvas();
    gui.show();

    return app.exec();
    #endif
    #endif
    return 0;
}
