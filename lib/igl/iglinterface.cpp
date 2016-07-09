#include "iglinterface.h"

namespace IGLInterface {
    /**
     * @brief generateGridAndDistanceField
     *
     * The bounding box  must be sufficiently larger to build a Grid with a step 2x2x2.
     *
     * The Grid generated will have a border of two points guaranteed to be outside the mesh.
     *
     * Saves a "tmp.bin" file containing, in order:
     * Eigen::RowVector3i minimum of the Grid
     * Eigen::RowVector3i maximum of the Grid
     * Eigen::RowVector3i resolution of the Grid
     * Eigen::MatrixXd n*3 vector of points, every row is a 3d point of the grid
     * Eigen::VectorXd signed distance, vector parallel to the grid vector, every falue is the distance to the surface
     *
     * Use the "serialize.h" file to load these informations
     *
     * @param[in] s: obj file containing the input mesh
     * @return true if all operations are done, false otherwise
     */
    bool generateGridAndDistanceField(const std::string& s) {
        Eigen::MatrixXi F;
        Eigen::MatrixXd V;
        // Read in inputs as double precision floating point meshes
        bool b = igl::read_triangle_mesh(s,V,F);

        if (b){
            // Bounding Box
            Eigen::RowVector3d Vmin = V.colwise().minCoeff();
            Eigen::RowVector3d Vmax = V.colwise().maxCoeff();

            // create grid GV
            Eigen::RowVector3i border(10, 10, 10);
            Eigen::RowVector3i nGmin = (Vmin).cast<int>() - border;
            Eigen::RowVector3i nGmax = (Vmax).cast<int>() + border; //bounding box of the Grid
            Eigen::RowVector3i res = (nGmax - nGmin)/2; res(0)+=1; res(1)+=1; res(2)+=1;
            Eigen::MatrixXd GV(res(0)*res(1)*res(2),3);

            int xi = nGmin(0), yi = nGmin(1), zi = nGmin(2);
            for (int i = 0; i < res(0); ++i){
                yi = nGmin(1);
                for (int j = 0; j < res(1); ++j){
                    zi = nGmin(2);
                    for (int k = 0; k < res(2); ++k){
                        //GV.row(i+res(0)*(j + res(1)*k)) = Eigen::RowVector3i(xi,yi,zi).cast<double>();
                        GV.row(k+res(2)*(j + res(1)*i)) = Eigen::RowVector3i(xi,yi,zi).cast<double>();
                        zi+=2;
                    }
                    yi+=2;
                }
                xi += 2;
            }

            // compute values
            Eigen::VectorXd S;
            Eigen::VectorXi I;
            Eigen::MatrixXd C,N;
            igl::signed_distance(GV,V,F,igl::SIGNED_DISTANCE_TYPE_PSEUDONORMAL,S,I,C,N);

            // S contiene le signed distances di dimensioni res(0)*res(1)*res(2)
            // Si accede ai suoi elementi con xi+res(0)*(yi + res(1)*zi)
            // Le coordinate nello spazio per ogni elemento invece si trovano in GV

            std::ofstream file;
            file.open ("tmp.bin", std::ios::out | std::ios::binary);
            Serializer::serialize(nGmin, file);
            Serializer::serialize(nGmax, file);
            Serializer::serialize(res, file);
            Serializer::serialize(GV, file);
            Serializer::serialize(S, file);
            file.close();
        }
        return b;
    }
}
