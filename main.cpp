/*
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @copyright Alessandro Muntoni 2016.
 */

#include "cg3/viewer/mainwindow.h"
#include "cg3/viewer/managers/dcel_manager.h"
#include "GUI/managers/enginemanager.h"
#include "common.h"
#include <QApplication>
//#include "common/comparators.h"
#include "engine/engine.h"
//#include "trimesh/gui/trimeshmanager.h"
#include "lib/graph/directedgraph.h"

#include "cg3/viewer/managers/eigenmesh_manager.h"
#include "cg3/viewer/managers/booleans_manager.h"
#include <cg3/utilities/string.h>
#include <cg3/utilities/system.h>

#include <typeinfo>       // operator typeid

#include "engine/reconstruction.h"
#include "cg3/utilities/command_line_argument_manager.h"

using namespace cg3;

#if defined(SERVER_MODE) || defined(SERVER_HOME) || defined(SERVER_AFTER)
void serializeBeforeBooleans(const std::string& filename, const Dcel& d, const EigenMesh& originalMesh, const BoxList& solutions, double factor, double kernel);
void deserializeBeforeBooleans(const std::string& filename, Dcel& d, EigenMesh& originalMesh, BoxList& solutions, double &factor, double &kernel);
void serializeAfterBooleans(const std::string& filename, const Dcel& d, const EigenMesh& originalMesh, const BoxList& solutions, const EigenMesh& baseComplex, const HeightfieldsList& he, double factor, double kernel, const BoxList& originalSolutions, const std::map<unsigned int, unsigned int>& splittedBoxesToOriginals, const std::list<unsigned int> &priorityBoxes);
void deserializeAfterBooleans(const std::string& filename, Dcel& d, EigenMesh& originalMesh, BoxList& solutions, EigenMesh& baseComplex, HeightfieldsList& he, double &factor, double &kernel, BoxList& originalSolutions, std::map<unsigned int, unsigned int>& splittedBoxesToOriginals, std::list<unsigned int> &priorityBoxes);
#endif

static Point3d getCustomLimits(const Dcel &m, double lx, double ly, double lz)
{
	Point3d limits;
	limits.x() = m.boundingBox().diag() * lx;
	limits.y() = m.boundingBox().diag() * ly;
	limits.z() = m.boundingBox().diag() * lz;
	return limits;
}

int main(int argc, char *argv[]) {
    #ifdef SERVER_MODE
	/**
	 * Implementation of the paper Axis-Aligned Height-Field Block Decomposition of 3D Shapes, TOG(2018).
	 *
	 * usage
	 * ./HeightFieldDecomposition <filename>.obj -<option>=<value>
	 *
	 * Possible options:
	 *
	 * [-s, -smooth]=<filename>: To use smoothed mesh for the optimization and then reintroduce details after
	 * (Sec 4.5 of the paper, Fig. 11). If not set, checks if <filename>_smooth.obj is inside the same directory of filename.obj.
	 * You can use any type of smoothing you want to obtain filename_smooth.obj (ex: taubin smoothing).
	 *
	 * [-p, -precision]=<value> (double > 0, default=1): controls the how fit is the grid constructed in the input mesh. precision = 1 means that the unit
	 *   edge of the grid is the avg of the edge-length of the input mesh;
	 *   precision = 2   -> avg_length * 0.5;
	 *   precision = 0.5 -> avg_length * 2;
	 *
	 * [-k, -kernel]=<value> (double [0-1], default=0): controls the constraint of the intirior void of the decomposition. 0 means that no intirior void
	 *   is required. The values between 0 and 1 are proportional to the farthest and nearest point from the surface.
	 *
	 * [-s, -snapping]=<value> (double > 0, default=2): numer of grid unit to snap boxes when they have similar coordinates.
	 *
	 * [-o, -orientat]=<value> (t/f, default=t): true if we want to execute a suboptimal orientation on the input mesh in preprocessing
	 *   (sec 4.1 of the paper);
	 *
	 * [-c, -conservative]=<value> (t/f, default=f): how grid vertices in borders are set in the initial interpolation scheme.
	 *
	 * [-x], [-y], [-z] = <value> (double [0, 1...], default=2): maximum block sizes constraints wrt the diagonal of the bounding box. For no limit, use a value
	 *   greater than 1.
	 *
	 * Example of calls:
	 *   ./HeightFieldDecomposition cube_spike.obj
	 *   ./HeightFieldDecomposition cube_spike.obj -s=cssmooth.obj -k=0.1 -p=1.1 -z=0.2
	 */

	cg3::CommandLineArgumentManager argManager(argc, argv);

	std::string filename = argManager[0];
	std::string filename_smooth;

	//variables
	Dcel d;
	EigenMesh original;
	bool smoothed = false, optimal_orientation = true, conservative = false;
	double precision = 1, kernel = 0, snapStep = 2;
	double lx = 2, ly = 2, lz = 2; //size constraints

	/**** Argument Management */
	//input mesh
	if (! fileExists(filename)){
		std::cerr << filename << " not found. Exiting.";
		return -1;
	}
	original.loadFromObj(filename);

	//smooth mesh
	if (argManager.exists("smooth") && fileExists(argManager.value("smooth"))){
		filename_smooth = argManager.value("smooth");
		if (!fileExists(filename_smooth)){
			std::cerr << filename_smooth << " not found. Exiting.";
			return -1;
		}
		else {
			d.loadFromObj(filename_smooth);
			smoothed = true;
			std::cerr << "Using smoothed mesh.\n";
		}
	}
	else {
		std::string rawname, extension;
		separateExtensionFromFilename(filename, rawname, extension);
		filename_smooth = rawname + "_smooth" + extension;
		if (fileExists(filename_smooth)){
			d.loadFromObj(filename_smooth);
			smoothed = true;
			std::cerr << filename_smooth << " found!\n";
		}
		else{
			d = cg3::Dcel(original);
		}
	}

	//precision
	if (argManager.exists("p") || argManager.exists("precision")){
		if (argManager.exists("p"))
			precision = std::stod(argManager.value("p"));
		else
			precision = std::stod(argManager.value("precision"));
	}

	//kernel
	if (argManager.exists("k") || argManager.exists("kernel")){
		if (argManager.exists("k"))
			kernel = std::stod(argManager.value("k"));
		else
			kernel = std::stod(argManager.value("kernel"));
	}

	//kernel
	if (argManager.exists("s") || argManager.exists("snap")){
		if (argManager.exists("s"))
			snapStep = std::stod(argManager.value("s"));
		else
			snapStep = std::stod(argManager.value("snap"));
	}

	//optimal orientation
	if (argManager.exists("o") || argManager.exists("orient")){
		if (argManager.exists("o")){
			if (argManager.value("o") != "")
				optimal_orientation = !(argManager.value("o") == "f");
		}
		else{
			if (argManager.value("orient") != "")
				optimal_orientation = !(argManager.value("orient") == "f");
		}
	}

	//conservative
	if (argManager.exists("c") || argManager.exists("conservative")){
		if (argManager.exists("c")){
			conservative = argManager.value("c") == "t";
		}
		else{
			conservative = argManager.value("conservative") == "t";
		}
	}

	//x
	if (argManager.exists("x")){
		lx = std::stod(argManager.value("x"));
	}

	//y
	if (argManager.exists("y")){
		ly = std::stod(argManager.value("y"));
	}

	//z
	if (argManager.exists("z")){
		lz = std::stod(argManager.value("z"));
	}



	//actual algorithm ...
	//setting up paths
	std::string rawname, extension;
	separateExtensionFromFilename(filename, rawname, extension);
	std::string foldername = rawname + "_";
	if (!optimal_orientation)
		foldername += "noo_";
	if (conservative)
		foldername += "cons_";

	foldername += toStringWithPrecision(precision) + "_" + toStringWithPrecision(kernel) + "/";
	executeCommand("mkdir " + foldername);

	//log
	std::ofstream logFile;
	logFile.open(foldername + "log.txt");

	if (smoothed)
		logFile << "Using Smoothed Mesh.\n";
	else
		logFile << "No smoothing was applied.\n";

	//optimal orientation
	if (optimal_orientation){ // if optimal orientation
		logFile << "Using Optimal Orientation! \n";
	}
	else {
		logFile << "Not using Optimal Orientation.\n";
	}
	if (conservative){
		logFile << "Conservative Strategy used.\n";
	}
	else {
		logFile << "Non-conservative Strategy used.\n";
	}

	logFile << "Parameters: \n\tPrecision: " << precision << "\n\tKernel: " << kernel << "\n\tSnapping: " << snapStep << "\n";

	logFile << "\tSize X limit: " << lx << "\n\tSize Y limit: " << ly << "\n\tSize Z limit: " << lz << "\n";
	logFile.flush();

	//scaling meshes
	BoundingBox3 bb= d.boundingBox();
	Engine::scaleAndRotateDcel(d, 0, precision);
	original.scale(bb, d.boundingBox());

	//optimal orientation
	if (optimal_orientation){ // if optimal orientation
		Eigen::Matrix3d m3d = Engine::findOptimalOrientation(d, original);
		logFile << "Optimal Orientation Rotation Matrix:\n";
		logFile << m3d << "\n";
	}
	logFile.flush();

	d.updateFaceNormals();
	d.updateVertexNormals();
	d.saveOnObj(foldername + rawname + "r_smooth.obj");
	original.saveOnObj(foldername + rawname + "r.obj");

	//solutions
	BoxList solutions;

	//grow boxes                              //boxes    mesh  kernel       limit  limit     toler           only  areatol  angletol  fileus  decim
	//double timerBoxGrowing = Engine::optimize(solutions, d, kernelDistance, false, Pointd(), !conservative,  true, 0,       0,        false,  true);
	double timerBoxGrowing = Engine::optimize(solutions, d, kernel, true, getCustomLimits(d, lx, ly, lz), !conservative,  true, 0,       0,        false,  true);

	logFile << timerBoxGrowing << ": Box Growing\n";
	logFile.flush();

	serializeBeforeBooleans(foldername + "all.bin", d, original, solutions, precision, kernel);

	Engine::boxPostProcessing(solutions, d);

	Timer tGurobi("Gurobi");
	Engine::minimalCovering(solutions, d);
	tGurobi.stopAndPrint();
	logFile << tGurobi.delay() << ": Minimal Covering\n";
	logFile.flush();
	//

	logFile << tGurobi.delay() << ": Minimal Covering\n";
	logFile << "Total Survived boxes: " << solutions.getNumberBoxes() << "\n";
	logFile.flush();
	std::cerr << "Total Survived boxes: " << solutions.getNumberBoxes() << "\n";

	serializeBeforeBooleans(foldername + "mc.bin", d, original, solutions, precision, kernel);



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

	//splitting and sorting
	solutions = originalSolutions;
	Timer tSplitting("ts");
	Array2D<int> ordering = Splitting::getOrdering(solutions, d, splittedBoxesToOriginals, priorityBoxes, userArcs);
	solutions.sort(ordering);
	tSplitting.stop();
	timerSplitting += tSplitting.delay();

	logFile << tSplitting.delay() << ": Splitting\n";
	logFile.flush();

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
	logFile << tBooleans.delay() << ": Booleans \n";
	logFile.flush();
	cgal::AABBTree3 tree(d);
	Engine::updatePiecesNormals(tree, he);
	Engine::colorPieces(d, he);

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
		cgal::AABBTree3 tree(d);
		Engine::updatePiecesNormals(tree, he);
		Engine::colorPieces(d, he);
	}
	serializeAfterBooleans(foldername + "final.hfd", d, original, solutions, baseComplex, he, precision, kernel, originalSolutions, splittedBoxesToOriginals, priorityBoxes);

	for(unsigned int i = 0; i < he.getNumHeightfields(); ++i){
		he.getHeightfield(i).saveOnObj(foldername + "block" + std::to_string(i) + ".obj");
	}

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
    if (argc > 1){
        std::string filename(argv[1]);
        std::string tmp, ext;
        cg3::separateExtensionFromFilename(filename, tmp, ext);
        if (ext == ".bin"){
            cg3::executeCommand("mkdir new/");
            DrawableDcel tmpd;
            BoxList tmpsol;
            EigenMesh originalMesh;
            double factor, kernel;
            bool b;
            std::ifstream infile;
            infile.open(filename, std::ios::in | std::ios::binary);
            tmpd.deserializeOld(infile);
            SerializerOld::deserialize(b, infile);
            tmpsol.deserializeOld(infile);
            SerializerOld::deserialize(b, infile);
            originalMesh.deserializeOld(infile);
            SerializerOld::deserialize(factor, infile);
            SerializerOld::deserialize(kernel, infile);
            infile.close();

            std::ofstream outfile;
            outfile.open("new/" + filename, std::ios::out | std::ios::binary);
            Serializer::serializeObjectAttributes("HFDBeforeSplitting", outfile, tmpd, tmpsol, originalMesh, factor, kernel);
            outfile.close();

        }
        else if ( ext == ".hfd"){
            cg3::executeCommand("mkdir new/");
            DrawableDcel tmpd;
            BoxList tmpsol;
            EigenMesh originalMesh;
            EigenMesh baseComplex;
            HeightfieldsList he;
            double factor, kernel;
            bool b;
            std::ifstream infile;
            infile.open(filename, std::ios::in | std::ios::binary);
            tmpd.deserializeOld(infile);
            tmpsol.deserializeOld(infile);
            baseComplex.deserializeOld(infile);
            he.deserializeOld(infile);
            originalMesh.deserializeOld(infile);
            SerializerOld::deserialize(factor, infile);
            SerializerOld::deserialize(kernel, infile);
            BoxList originalSolutions;
            std::map<unsigned int, unsigned int> splittedBoxesToOriginals;
            std::list<unsigned int> priorityBoxes;
            SerializerOld::deserialize(b, infile);
            originalSolutions.deserializeOld(infile);
            SerializerOld::deserialize(splittedBoxesToOriginals, infile);
            SerializerOld::deserialize(priorityBoxes, infile);
            infile.close();

            std::ofstream outfile;
            outfile.open("new/" + filename, std::ios::out | std::ios::binary);
            Serializer::serializeObjectAttributes("HFDBeforeSplitting", outfile, tmpd, tmpsol, originalMesh, factor, kernel);
            Serializer::serializeObjectAttributes("HFDAfterBooleans", outfile, baseComplex, he, originalSolutions, splittedBoxesToOriginals, priorityBoxes);
            outfile.close();
        }
    }
    #else

    QApplication app(argc, argv);

    cg3::viewer::MainWindow gui;  // finestra principale, contiene la canvas di QGLViewer
    mw = &gui;

    // Creo un dcel manager e lo aggiungo alla mainwindow
    cg3::viewer::DcelManager d(&gui);
    DCEL_MANAGER_ID = gui.addManager(&d, "Dcel");

    EngineManager e(&gui);
    ENGINE_MANAGER_ID = gui.addManager(&e, "Engine");

    cg3::viewer::BooleansManager bm(&gui);
    gui.addManager(&bm, "Booleans Manager");

    //TrimeshManager tm(&gui);
    //gui.addManager(&tm, "Trimesh Manager");

    cg3::viewer::EigenMeshManager em(&gui);
    gui.addManager(&em, "EigenMesh Manager");

    if (argc >= 2){
        std::cerr << argv[1] << "\n";
        std::string filename = argv[1];
        std::string rawname, extension, path, fn;
        separateExtensionFromFilename(filename, rawname, extension);
        separateFilenameFromPath(filename, path, fn);
        std::cerr << rawname << "; " << extension << "\n";
        if (extension == ".hfd"){
            e.deserializeBC(filename);
            e.setHfdPath(path);
            e.setBinPath(path);
            e.setObjPath(path);
        }
        else if (extension == ".bin"){
            std::ifstream myfile;
            myfile.open (filename, std::ios::in | std::ios::binary);
            e.deserialize(myfile);
            myfile.close();
            e.setHfdPath(path);
            e.setBinPath(path);
            e.setObjPath(path);
        }
    }

    gui.setCurrentManager(ENGINE_MANAGER_ID); // il dcel manager sarà quello visualizzato di default
    gui.canvas.update();
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
    cg3::serializeObjectAttributes("HFDBeforeSplitting", binaryFile, d, solutions, originalMesh, factor, kernel);
    binaryFile.close();
}

void deserializeBeforeBooleans(const std::string& filename, Dcel& d, EigenMesh& originalMesh, BoxList& solutions, double &factor, double &kernel) {
    std::ifstream binaryFile;
    binaryFile.open (filename, std::ios::in | std::ios::binary);
    cg3::deserializeObjectAttributes("HFDBeforeSplitting", binaryFile, d, solutions, originalMesh, factor, kernel);
    binaryFile.close();
}

void serializeAfterBooleans(const std::string& filename, const Dcel& d, const EigenMesh& originalMesh, const BoxList& solutions, const EigenMesh& baseComplex, const HeightfieldsList& he, double factor, double kernel, const BoxList& originalSolutions, const std::map<unsigned int, unsigned int>& splittedBoxesToOriginals, const std::list<unsigned int> &priorityBoxes) {
    std::ofstream myfile;
    myfile.open (filename, std::ios::out | std::ios::binary);
    cg3::serializeObjectAttributes("HFDBeforeSplitting", myfile, d, solutions, originalMesh, factor, kernel);
    cg3::serializeObjectAttributes("HFDAfterBooleans", myfile, baseComplex, he, originalSolutions, splittedBoxesToOriginals, priorityBoxes);
    myfile.close();
}

void deserializeAfterBooleans(const std::string& filename, Dcel& d, EigenMesh& originalMesh, BoxList& solutions, EigenMesh& baseComplex, HeightfieldsList& he, double &factor, double &kernel, BoxList& originalSolutions, std::map<unsigned int, unsigned int>& splittedBoxesToOriginals, std::list<unsigned int> &priorityBoxes){
    std::ifstream myfile;
    myfile.open (filename, std::ios::in | std::ios::binary);
    cg3::deserializeObjectAttributes("HFDBeforeSplitting", myfile, d, solutions, originalMesh, factor, kernel);
    cg3::deserializeObjectAttributes("HFDAfterBooleans", myfile, baseComplex, he, originalSolutions, splittedBoxesToOriginals, priorityBoxes);
    myfile.close();
}

#endif
