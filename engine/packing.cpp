#include "packing.h"
#include "lib/packing/binpack2d.h"
#include <cg3/geometry/transformations3.h>

using namespace cg3;

void Packing::rotateAllPieces(HeightfieldsList& he) {
    for (unsigned int i = 0; i < he.getNumHeightfields(); i++){
        EigenMesh m = he.getHeightfield(i);
		Vec3d normal = he.getTarget(i);
		Vec3d zAxis(0,0,1);
		Vec3d axis = normal.cross(zAxis);
        axis.normalize();
        double dot = normal.dot(zAxis);
        double angle = acos(dot);

        Eigen::Matrix3d r = Eigen::Matrix3d::Zero();
        if (normal != zAxis){
            if (normal == -zAxis){
				axis = Vec3d(1,0,0);
            }
			cg3::rotationMatrix(axis, angle, r);
        }
        else {
            r = Eigen::Matrix3d::Identity();
        }
        m.rotate(r);
        m.updateBoundingBox();
		m.translate(Point3d(0,0,-m.boundingBox().min().z()));
        m.updateBoundingBox();
        he.setHeightfield(m,i);
    }
}

int Packing::getMaximum(const HeightfieldsList& he, const BoundingBox3& block, double& factor) {
	assert(block.lengthX() > 0 || block.lengthY() > 0 || block.lengthZ() > 0);

    double maxx= 0, maxy=0, maxz=0;
    int ix=0, iy=0, iz=0;
    for (unsigned int i = 0; i < he.getNumHeightfields(); i++){
		if (maxx < he.getHeightfield(i).boundingBox().lengthX()){
            ix = i;
			maxx = he.getHeightfield(i).boundingBox().lengthX();
        }
		if (maxy < he.getHeightfield(i).boundingBox().lengthY()){
            iy = i;
			maxy = he.getHeightfield(i).boundingBox().lengthY();
        }
		if (maxz < he.getHeightfield(i).boundingBox().lengthZ()){
            iz = i;
			maxz = he.getHeightfield(i).boundingBox().lengthZ();
        }
    }
	double factorx = block.lengthX() > 0 ? (block.lengthX()-5) / maxx : std::numeric_limits<double>::max();
	double factory = block.lengthY() > 0 ? (block.lengthY()-5) / maxy : std::numeric_limits<double>::max();
	double factorz = block.lengthZ() > 0 ? (block.lengthZ()-1) / maxz : std::numeric_limits<double>::max();
    if (factorx <= factory && factorx <= factorz){
        factor = factorx;
        return ix;
    }
    if (factory <= factorx && factory <= factorz){
        factor = factory;
        return iy;
    }
    if (factorz <= factorx && factorz <= factory){
        factor = factorz;
        return iz;
    }
    assert(0);
    return -1;

}

void Packing::scaleAll(HeightfieldsList& he, double factor) {
	Vec3d vecFac(factor, factor, factor);
    for (unsigned int i = 0; i < he.getNumHeightfields(); i++){
        he.getHeightfield(i).scale(vecFac);
        he.getHeightfield(i).updateBoundingBox();
    }
}

std::vector< std::vector<std::pair<int, Point3d> > > Packing::pack(const HeightfieldsList& he, const BoundingBox3 &packSize, int distance) {
    if (distance <= 0){
		distance = std::min(packSize.lengthX(), packSize.lengthY()) / 2;
    }
	std::vector< std::vector<std::pair<int, Point3d> > > packs;
    std::set<unsigned int> piecesToPack;
    for (unsigned int i = 0; i < he.getNumHeightfields(); i++)
        piecesToPack.insert(i);

    unsigned int oldSize = 0;
    while (piecesToPack.size() > 0 && oldSize != piecesToPack.size()){
        oldSize = piecesToPack.size();
		std::vector<std::pair<int, Point3d> > actualPack;

        // Create some 'content' to work on.
        BinPack2D::ContentAccumulator<int> inputContent;

        for(int i : piecesToPack) {

            // random size for this content
			int width  = he.getHeightfield(i).boundingBox().lengthX()*10 + distance;
			int height = he.getHeightfield(i).boundingBox().lengthY()*10 + distance;

            // whatever data you want to associate with this content

            int mycontent= i;

            // Add it
            inputContent += BinPack2D::Content<int>(mycontent, BinPack2D::Coord(), BinPack2D::Size(width, height), false);
        }

        // Sort the input content by size... usually packs better.
        inputContent.Sort();

        // Create some bins!
        BinPack2D::CanvasArray<int> canvasArray =
				BinPack2D::UniformCanvasArrayBuilder<int>(packSize.lengthX()*10,packSize.lengthY()*10,1).Build();

        // A place to store content that didnt fit into the canvas array.
        BinPack2D::ContentAccumulator<int> remainder;

        // try to pack content into the bins.
        canvasArray.Place( inputContent, remainder );

        // A place to store packed content.
        BinPack2D::ContentAccumulator<int> outputContent;

        // Read all placed content.
        canvasArray.CollectContent( outputContent );

        // parse output.
        typedef BinPack2D::Content<int>::Vector::iterator binpack2d_iterator;
        printf("PLACED:\n");
        for( binpack2d_iterator itor = outputContent.Get().begin(); itor != outputContent.Get().end(); itor++ ) {

            const BinPack2D::Content<int> &content = *itor;

            // retreive your data.
            const int &myContent = content.content;

            piecesToPack.erase(myContent);

			Point3d pos((double)content.coord.x/10 + 0.1, (double)content.coord.y/10 + 0.1, (double)content.coord.z/10 + + 0.1);

			std::pair<int, Point3d> pair;
            pair.first = content.rotated ? -(myContent+1): (myContent+1);
            pair.second = pos;
            actualPack.push_back(pair);

            printf("\t%d of size %3dx%3d at position %3d,%3d,%2d rotated=%s\n",
                   myContent,
                   content.size.w,
                   content.size.h,
                   content.coord.x,
                   content.coord.y,
                   content.coord.z,
                   (content.rotated ? "yes":" no"));
        }
        packs.push_back(actualPack);

        printf("NOT PLACED:\n");
        for( binpack2d_iterator itor = remainder.Get().begin(); itor != remainder.Get().end(); itor++ ) {

            const BinPack2D::Content<int> &content = *itor;

            const int &myContent = content.content;

            printf("\t%d of size %3dx%3d\n",
                   myContent,
                   content.size.w,
                   content.size.h);
        }
    }
    if (piecesToPack.size() != 0)
        std::cerr << "Some pieces cannot be putted on a pack with the given sizes\n";
    return packs;
}

std::vector<std::vector<EigenMesh> > Packing::getPacks(std::vector<std::vector<std::pair<int, Point3d> > >& packing, const HeightfieldsList& he) {
    std::vector<std::vector<EigenMesh> > out;
    for (unsigned int i = 0; i < packing.size(); i++){
        std::vector<EigenMesh> actualPack;
        for (unsigned int j = 0; j < packing[i].size(); j++){
			std::pair<int, Point3d> pair = packing[i][j];
            int id;
            EigenMesh mesh;
            if (pair.first < 0){
                id = (-pair.first)-1;
                mesh = he.getHeightfield(id);
                Eigen::Matrix3d rot;
				cg3::rotationMatrix(Vec3d(0,0,1),M_PI/2,rot);
                mesh.rotate(rot);
            }
            else{
                id = pair.first-1;
                mesh = he.getHeightfield(id);
            }
            mesh.updateBoundingBox();
			mesh.translate(- mesh.boundingBox().min() + pair.second);
            actualPack.push_back(mesh);
        }
        out.push_back(actualPack);
    }
    return out;
}
