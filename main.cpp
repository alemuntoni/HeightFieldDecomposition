/*
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @copyright Alessandro Muntoni 2016.
 */

#include "cg3/viewer/mainwindow.h"
#include "cg3/viewer/managers/dcel_manager/dcel_manager.h"
#include "cg3/viewer/managers/window_manager/window_manager.h"
#include "GUI/managers/enginemanager.h"
#include "GUI/managers/fouraxischeckermanager.h"
#include "common.h"
#include <QApplication>
//#include "common/comparators.h"
#include "engine/engine.h"
//#include "trimesh/gui/trimeshmanager.h"
#include "lib/graph/directedgraph.h"

#include "cg3/viewer/managers/eigenmesh_manager/eigenmesh_manager.h"
#include "cg3/viewer/managers/booleans_manager/booleans_manager.h"
#include <cg3/utilities/string.h>
#include <cg3/utilities/system.h>

#include "lib/graph/bipartitegraph.h"
#include <typeinfo>       // operator typeid
#include <lib/csgtree/aabbcsgtree.h>

#include "engine/reconstruction.h"

#include "foo.h"

using namespace cg3;

#if defined(SERVER_MODE) || defined(SERVER_HOME) || defined(SERVER_AFTER)
void serializeBeforeBooleans(const std::string& filename, const Dcel& d, const EigenMesh& originalMesh, const BoxList& solutions, double factor, double kernel);
void deserializeBeforeBooleans(const std::string& filename, Dcel& d, EigenMesh& originalMesh, BoxList& solutions, double &factor, double &kernel);
void serializeAfterBooleans(const std::string& filename, const Dcel& d, const EigenMesh& originalMesh, const BoxList& solutions, const EigenMesh& baseComplex, const HeightfieldsList& he, double factor, double kernel, const BoxList& originalSolutions, const std::map<unsigned int, unsigned int>& splittedBoxesToOriginals, const std::list<unsigned int> &priorityBoxes);
void deserializeAfterBooleans(const std::string& filename, Dcel& d, EigenMesh& originalMesh, BoxList& solutions, EigenMesh& baseComplex, HeightfieldsList& he, double &factor, double &kernel, BoxList& originalSolutions, std::map<unsigned int, unsigned int>& splittedBoxesToOriginals, std::list<unsigned int> &priorityBoxes);
#endif

int main(int argc, char *argv[]) {

    #ifdef SERVER_MODE
    //usage
    // ./HeightFieldDecomposition filename.obj precision kernel snapping orientation (t/f)
    if (argc > 3){
        bool smoothed = true;
        std::string filename(argv[1]);
        if (! fileExists(filename)){
            std::cerr << filename << " not found. Exiting.";
            return -1;
        }
        std::string rawname, extension;
        separateExtensionFromFilename(filename, rawname, extension);
        std::string filename_smooth = rawname + "_smooth" + extension;

        //reading files
        EigenMesh original;
        original.readFromObj(filename);
        Dcel d;
        if (fileExists(filename_smooth)){
            d.loadFromObjFile(filename_smooth);
            std::cerr << "Smooth File found.\n";
        }
        else {
            std::cerr << "Smooth File not found. Using original file.\n";
            d = original;
            smoothed = false;
        }

        //precision and kernel
        double precision = std::stod(argv[2]);
        double kernelDistance = std::stod(argv[3]);

        //creating folder
        std::string foldername = rawname + "_";
        bool optimal = true;
        if (argc == 6 && std::string(argv[5]) == "f"){ // if no optimal orientation required, there will be different foldername
            foldername += "noo_";
            optimal = false;
        }
        foldername += toStringWithPrecision(precision) + "_" + toStringWithPrecision(kernelDistance) + "/";
        executeCommand("mkdir " + foldername);

        //log
        std::ofstream logFile;
        logFile.open(foldername + "log.txt");

        //scaling meshes
        BoundingBox bb= d.getBoundingBox();
        Engine::scaleAndRotateDcel(d, 0, precision);
        original.scale(bb, d.getBoundingBox());



        //optimal orientation
        if (optimal){ // if optimal orientation
            Engine::findOptimalOrientation(d, original);
        }
        d.updateFaceNormals();
        d.updateVertexNormals();
        d.saveOnObjFile(foldername + rawname + "r_smooth.obj");
        original.saveOnObj(foldername + rawname + "r.obj");

        //solutions
        BoxList solutions;

        //grow boxes
        double timerBoxGrowing = Engine::optimize(solutions, d, kernelDistance, false, Pointd(), true, true, 0, 0, false, true);

        logFile << timerBoxGrowing << ": Box Growing\n";

        serializeBeforeBooleans(foldername + "all.bin", d, original, solutions, precision, kernelDistance);

        Engine::boxPostProcessing(solutions, d);
        double timerMinimalCovering = Engine::deleteBoxes(solutions, d);
        logFile << timerMinimalCovering << ": Minimal Covering\n";

        serializeBeforeBooleans(foldername + "mc.bin", d, original, solutions, precision, kernelDistance);

        double snapStep;
        if (argc == 5)
            snapStep = std::stod(argv[4]);
        else
            snapStep = 2;

        //snapping
        Engine::stupidSnapping(d, solutions, snapStep);

        //new: forced snapping
        Engine::smartSnapping(d, solutions);

        //merging
        Engine::merging(d, solutions);

        //setting ids
        solutions.sortByTrianglesCovered();
        solutions.setIds();
        BoxList originalSolutions = solutions;
        std::map<unsigned int, unsigned int> splittedBoxesToOriginals;
        std::list<unsigned int> priorityBoxes;
        std::vector<std::pair<unsigned int, unsigned int>> userArcs;
        HeightfieldsList he;
        EigenMesh baseComplex;

        double timerSplitting = 0;
        double timerBooleans = 0;
        int it = 0;
        do {
            //splitting and sorting
            solutions = originalSolutions;
            Timer tSplitting("ts");
            Array2D<int> ordering = Splitting::getOrdering(solutions, d, splittedBoxesToOriginals, priorityBoxes, userArcs);
            solutions.sort(ordering);
            tSplitting.stop();
            timerSplitting += tSplitting.delay();

            logFile << tSplitting.delay() << ": Splitting n. " << std::to_string(it) << "\n";

            //booleans
            d.updateFaceNormals();
            d.updateVertexNormals();
            baseComplex = d;
            he = HeightfieldsList();
            Timer tBooleans("tb");
            Engine::booleanOperations(he, baseComplex, solutions, false);
            Engine::splitConnectedComponents(he, solutions, splittedBoxesToOriginals);
            Engine::glueInternHeightfieldsToBaseComplex(he, solutions, baseComplex, d);
            tBooleans.stop();
            timerBooleans += tBooleans.delay();
            logFile << tBooleans.delay() << ": Booleans n. " << std::to_string(it) << "\n";
            cgal::AABBTree tree(d);
            Engine::updatePiecesNormals(tree, he);
            Engine::colorPieces(d, he);

            serializeAfterBooleans(foldername + "bools" + std::to_string(it) + ".hfd", d, original, solutions, baseComplex, he, precision, kernelDistance, originalSolutions, splittedBoxesToOriginals, priorityBoxes);
            it++;
        } while(false);

        logFile << timerSplitting << ": Total time Splitting\n";
        logFile << timerBooleans << ": Total time Booleans\n";
        logFile.close();
        //restore hf
        if (smoothed){
            //Common::executeCommand("./restorehf " + foldername + "bools" + std::to_string(it-1) + ".hfd " + foldername);
            std::vector< std::pair<int,int> > mapping = Reconstruction::getMapping(d, he);
            Reconstruction::reconstruction(d, mapping, original, solutions);

            baseComplex = d;
            d.updateFaceNormals();
            d.updateVertexNormals();
            he = HeightfieldsList();
            Engine::booleanOperations(he, baseComplex, solutions, false);
            Engine::splitConnectedComponents(he, solutions, splittedBoxesToOriginals);
            Engine::glueInternHeightfieldsToBaseComplex(he, solutions, baseComplex, d);
            cgal::AABBTree tree(d);
            Engine::updatePiecesNormals(tree, he);
            Engine::colorPieces(d, he);
        }
        serializeAfterBooleans(foldername + "final.hfd", d, original, solutions, baseComplex, he, precision, kernelDistance, originalSolutions, splittedBoxesToOriginals, priorityBoxes);

    }
    else
        std::cerr << "Error! Number argument lower than 4\n";
    #else
    #ifdef SERVER_HOME
    if (argc > 3){
        bool smoothed = true;
        std::string filename(argv[1]);
        if (! fileExists(filename)){
            std::cerr << filename << " not found. Exiting.";
            return -1;
        }
        std::string rawname, extension;
        separateExtensionFromFilename(filename, rawname, extension);
        std::string filename_smooth = rawname + "_smooth" + extension;

        //reading files
        EigenMesh original;
        original.readFromObj(filename);
        Dcel d;
        if (fileExists(filename_smooth)){
            d.loadFromObjFile(filename_smooth);
            std::cerr << "Smooth File found.\n";
        }
        else {
            std::cerr << "Smooth File not found. Using original file.\n";
            d = original;
            smoothed = false;
        }

        //precision and kernel
        double precision = std::stod(argv[2]);
        double kernelDistance = std::stod(argv[3]);

        //creating folder
        std::string foldername = rawname + "_";
        bool optimal = true;
        if (argc == 5 && std::string(argv[5]) == "f"){ // if no optimal orientation required, there will be different foldername
            foldername += "noo_";
            optimal = false;
        }
        foldername += toStringWithPrecision(precision) + "_" + toStringWithPrecision(kernelDistance) + "/";
        executeCommand("mkdir " + foldername);

        //log
        std::ofstream logFile;
        logFile.open(foldername + "log.txt");

        //scaling meshes
        BoundingBox bb= d.getBoundingBox();
        Engine::scaleAndRotateDcel(d, 0, precision);
        original.scale(bb, d.getBoundingBox());



        //optimal orientation
        if (optimal){
            Engine::findOptimalOrientation(d, original);
        }
        d.updateFaceNormals();
        d.updateVertexNormals();
        d.saveOnObjFile(foldername + rawname + "r_smooth.obj");
        original.saveOnObj(foldername + rawname + "r.obj");

        //solutions
        BoxList solutions;

        //grow boxes
        double timerBoxGrowing = Engine::optimize(solutions, d, kernelDistance, false, Pointd(), true, true, 0, 0, false, true);

        logFile << timerBoxGrowing << ": Box Growing\n";

        serializeBeforeBooleans(foldername + "all.bin", d, original, solutions, precision, kernelDistance);

        logFile.close();
    }
    #else
    #ifdef SERVER_AFTER
    if (argc > 1){
        std::string foldername(argv[1]);
        if (foldername[foldername.size()-1] != '/')
            foldername +="/";
        EigenMesh original;
        Dcel d;
        bool smoothed = true;
        BoxList solutions;

        //precision and kernel
        double precision;
        double kernelDistance;

        //log
        std::ofstream logFile;
        logFile.open(foldername + "log.txt", std::ofstream::out | std::ofstream::app);

        //deserialize all.bin
        deserializeBeforeBooleans(foldername + "all.bin", d, original, solutions, precision, kernelDistance);

        Engine::boxPostProcessing(solutions, d);
        double timerMinimalCovering = Engine::deleteBoxes(solutions, d);
        logFile << timerMinimalCovering << ": Minimal Covering\n";

        serializeBeforeBooleans(foldername + "mc.bin", d, original, solutions, precision, kernelDistance);

        double snapStep;
        if (argc == 3)
            snapStep = std::stod(argv[2]);
        else
            snapStep = 2;

        //snapping
        Engine::stupidSnapping(d, solutions, snapStep);

        //new: forced snapping
        Engine::smartSnapping(d, solutions);

        //merging
        Engine::merging(d, solutions);

        //setting ids
        solutions.sortByTrianglesCovered();
        solutions.setIds();
        BoxList originalSolutions = solutions;
        std::map<unsigned int, unsigned int> splittedBoxesToOriginals;
        std::list<unsigned int> priorityBoxes;
        std::vector<std::pair<unsigned int, unsigned int>> userArcs;
        HeightfieldsList he;
        EigenMesh baseComplex;

        double timerSplitting = 0;
        double timerBooleans = 0;
        int it = 0;
        do {
            //splitting and sorting
            solutions = originalSolutions;
            Timer tSplitting("ts");
            Array2D<int> ordering = Splitting::getOrdering(solutions, d, splittedBoxesToOriginals, priorityBoxes, userArcs);
            solutions.sort(ordering);
            tSplitting.stop();
            timerSplitting += tSplitting.delay();

            logFile << tSplitting.delay() << ": Splitting n. " << std::to_string(it) << "\n";

            //booleans
            d.updateFaceNormals();
            d.updateVertexNormals();
            baseComplex = d;
            he = HeightfieldsList();
            Timer tBooleans("tb");
            Engine::booleanOperations(he, baseComplex, solutions, false);
            Engine::splitConnectedComponents(he, solutions, splittedBoxesToOriginals);
            Engine::glueInternHeightfieldsToBaseComplex(he, solutions, baseComplex, d);
            tBooleans.stop();
            timerBooleans += tBooleans.delay();
            logFile << tBooleans.delay() << ": Booleans n. " << std::to_string(it) << "\n";
            cgal::AABBTree tree(d);
            Engine::updatePiecesNormals(tree, he);
            Engine::colorPieces(d, he);

            serializeAfterBooleans(foldername + "bools" + std::to_string(it) + ".hfd", d, original, solutions, baseComplex, he, precision, kernelDistance, originalSolutions, splittedBoxesToOriginals, priorityBoxes);
            it++;
        } while(false);

        logFile << timerSplitting << ": Total time Splitting\n";
        logFile << timerBooleans << ": Total time Booleans\n";
        logFile.close();
        //restore hf
        if (smoothed){
            //Common::executeCommand("./restorehf " + foldername + "bools" + std::to_string(it-1) + ".hfd " + foldername);
            std::vector< std::pair<int,int> > mapping = Reconstruction::getMapping(d, he);
            Reconstruction::reconstruction(d, mapping, original, solutions);

            baseComplex = d;
            d.updateFaceNormals();
            d.updateVertexNormals();
            he = HeightfieldsList();
            Engine::booleanOperations(he, baseComplex, solutions, false);
            Engine::splitConnectedComponents(he, solutions, splittedBoxesToOriginals);
            Engine::glueInternHeightfieldsToBaseComplex(he, solutions, baseComplex, d);
            cgal::AABBTree tree(d);
            Engine::updatePiecesNormals(tree, he);
            Engine::colorPieces(d, he);
        }
        serializeAfterBooleans(foldername + "final.hfd", d, original, solutions, baseComplex, he, precision, kernelDistance, originalSolutions, splittedBoxesToOriginals, priorityBoxes);

    }
    else
        std::cerr << "Error! Number argument lower than 2\n";
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

    /*Array2D<int> a(4,4);
    for (unsigned int i = 0; i < 4; i++){
        for (unsigned int j = 0; j < 4; j++)
            a(i,j) = i;
    }
    std::ofstream myfile;
    myfile.open ("provaser.ppp", std::ios::out | std::ios::binary);
    a.serialize(myfile);
    myfile.close();*/

    /*Array2D<int> a;
    std::ifstream myfile;
    myfile.open ("provaser.ppp", std::ios::in | std::ios::binary);
    a.deserialize(myfile);
    myfile.close();
    std::cerr << a << "\n";*/




    QApplication app(argc, argv);

    MainWindow gui;  // finestra principale, contiene la canvas di QGLViewer
    mw = &gui;

    // Creo un window manager e lo aggiungo alla mainwindow
    WindowManager wm(&gui);
    WINDOW_MANAGER_ID = gui.addManager(&wm, "Window");

    // Creo un dcel manager e lo aggiungo alla mainwindow
    DcelManager d(&gui);
    DCEL_MANAGER_ID = gui.addManager(&d, "Dcel");

    EngineManager e(&gui);
    ENGINE_MANAGER_ID = gui.addManager(&e, "Engine");

    FourAxisMillingManager fm(&gui);
    gui.addManager(&fm, "Four Axis Milling Manager");

    BooleansManager bm(&gui);
    gui.addManager(&bm, "Booleans Manager");

    //TrimeshManager tm(&gui);
    //gui.addManager(&tm, "Trimesh Manager");

    EigenMeshManager em(&gui);
    gui.addManager(&em, "EigenMesh Manager");

    if (argc >= 2){
        std::cerr << argv[1] << "\n";
        std::string filename = argv[1];
        std::string rawname, extension, path, fn;
        separateExtensionFromFilename(filename, rawname, extension);
        separateFilenameFromPath(filename, path, fn);
        std::cerr << rawname << "; " << extension << "\n";
        if (extension == ".hfd"){
            e.deserializeBCOld(filename);
            e.setHfdPath(path);
            e.setBinPath(path);
            e.setObjPath(path);
        }
        else if (extension == ".bin"){
            std::ifstream myfile;
            myfile.open (filename, std::ios::in | std::ios::binary);
            e.deserializeOld(myfile);
            myfile.close();
            e.setHfdPath(path);
            e.setBinPath(path);
            e.setObjPath(path);
        }
        /*else if (extension == ".obj"){
            Dcel d;
            d.loadFromObjFile(argv[1]);
            cgal::AABBTree tree(d);
            Dcel d2;
            d2.loadFromObjFile(argv[2]);
            for (Dcel::Vertex* v2 : d2.vertexIterator()){
                const Dcel::Vertex* v = tree.getNearestDcelVertex(v2->getCoordinate());
                v2->setNormal(v->getNormal());
            }
            std::cerr << "ok\n";
            d2.saveOnObjFile(path + "/Modified.obj");
        }*/
    }

    gui.setCurrentIndexToolBox(ENGINE_MANAGER_ID); // il dcel manager sarà quello visualizzato di default
    gui.updateGlCanvas();
    gui.show();

    return app.exec();
    #endif
    #endif
    #endif
    #endif
    return 0;
}

#if defined(SERVER_MODE) || defined(SERVER_HOME) || defined(SERVER_AFTER)
void serializeBeforeBooleans(const std::string& filename, const Dcel& d, const EigenMesh& originalMesh, const BoxList& solutions, double factor, double kernel) {
    std::ofstream binaryFile;
    binaryFile.open (filename, std::ios::out | std::ios::binary);
    d.serialize(binaryFile);
    bool bb = true;
    Serializer::serialize(bb, binaryFile);
    solutions.serialize(binaryFile);
    Serializer::serialize(bb, binaryFile);
    originalMesh.serialize(binaryFile);
    Serializer::serialize(factor, binaryFile);
    Serializer::serialize(kernel, binaryFile);
    binaryFile.close();
}

void deserializeBeforeBooleans(const std::string& filename, Dcel& d, EigenMesh& originalMesh, BoxList& solutions, double &factor, double &kernel) {
    std::ifstream binaryFile;
    binaryFile.open (filename, std::ios::in | std::ios::binary);
    d.deserialize(binaryFile);
    bool bb;
    Serializer::deserialize(bb, binaryFile);
    solutions.deserialize(binaryFile);
    Serializer::deserialize(bb, binaryFile);
    originalMesh.deserialize(binaryFile);
    Serializer::deserialize(factor, binaryFile);
    Serializer::deserialize(kernel, binaryFile);
    binaryFile.close();
}

void serializeAfterBooleans(const std::string& filename, const Dcel& d, const EigenMesh& originalMesh, const BoxList& solutions, const EigenMesh& baseComplex, const HeightfieldsList& he, double factor, double kernel, const BoxList& originalSolutions, const std::map<unsigned int, unsigned int>& splittedBoxesToOriginals, const std::list<unsigned int> &priorityBoxes) {
    std::ofstream myfile;
    myfile.open (filename, std::ios::out | std::ios::binary);
    d.serialize(myfile);
    solutions.serialize(myfile);
    baseComplex.serialize(myfile);
    he.serialize(myfile);
    originalMesh.serialize(myfile);
    Serializer::serialize(factor, myfile);
    Serializer::serialize(kernel, myfile);
    bool b = true;
    Serializer::serialize(b, myfile);
    originalSolutions.serialize(myfile);
    Serializer::serialize(splittedBoxesToOriginals, myfile);
    Serializer::serialize(priorityBoxes, myfile);
    myfile.close();
}

void deserializeAfterBooleans(const std::string& filename, Dcel& d, EigenMesh& originalMesh, BoxList& solutions, EigenMesh& baseComplex, HeightfieldsList& he, double &factor, double &kernel, BoxList& originalSolutions, std::map<unsigned int, unsigned int>& splittedBoxesToOriginals, std::list<unsigned int> &priorityBoxes){
    std::ifstream myfile;
    myfile.open (filename, std::ios::in | std::ios::binary);
    d.deserialize(myfile);
    solutions.deserialize(myfile);
    baseComplex.deserialize(myfile);
    he.deserialize(myfile);
    originalMesh.deserialize(myfile);
    Serializer::deserialize(factor, myfile);
    Serializer::deserialize(kernel, myfile);
    bool b;
    if (Serializer::deserialize(b, myfile) && b == true){
        originalSolutions.deserialize(myfile);
        Serializer::deserialize(splittedBoxesToOriginals, myfile);
        Serializer::deserialize(priorityBoxes, myfile);
        for (unsigned int i = 0; i < originalSolutions.size(); i++)
            originalSolutions[i].setId(i);
    }
}

#endif
