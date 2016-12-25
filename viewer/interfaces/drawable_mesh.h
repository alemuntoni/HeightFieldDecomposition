#ifndef DRAWABLEMESH_H
#define DRAWABLEMESH_H

#ifdef COMMON_WITH_EIGEN
#include <Eigen/Core>
#endif

#include "drawable_object.h"

// From: https://blog.nobel-joergensen.com/2013/01/29/debugging-opengl-using-glgeterror/
void _check_gl_error(const char *file, int line);

///
/// Usage
/// [... some opengl calls]
/// glCheckError();
///
#define check_gl_error() _check_gl_error(__FILE__,__LINE__)

/**
 * @brief The DrawableMesh class
 * This is a non-instantiable class.
 * You can only inherit this class (protected constructors).
 */
class DrawableMesh : public DrawableObject{
    public:

        void init();

        // Implementation of the
        // DrawableObject interface
        void draw() const;
        virtual Pointd sceneCenter() const = 0;
        virtual double sceneRadius() const = 0;
        bool isVisible() const;

        // rendering options
        //
        void setWireframe(bool b);
        void setFlatShading();
        void setSmoothShading();
        void setPointsShading();
        void setWireframeColor(float r, float g, float b);
        void setWireframeWidth(float width);
        void setEnableVertexColor();
        void setEnableTriangleColor();
        void setVisible(bool b);

    private:

        typedef enum {
            STD, EIGEN
        } MeshType;

        const std::vector<double> * pCoords; /** \~Italian @brief vettore di coordinate usate per la visualizzazione: per aggiornare utilizzare metodo update() */
        const std::vector<int> * pTriangles; /** \~Italian @brief vettore di triangoli (da considerare a triple di indici) usati per la visualizzazione: per aggiornare utilizzare il metodo update() */
        const std::vector<double> * pVertexNormals; /** \~Italian @brief vettore di normali ai vertici usate per la visualizzazione: per aggiornare utilizzare il metodo update() */
        const std::vector<float> * pVertexColors; /** \~Italian @brief vettore di colori associati ai vertici (da considerare come triple rgb float) usati per la visualizzazione: per aggiornare utilizzare il metodo update() */
        const std::vector<double> * pTriangleNormals; /** \~Italian @brief vettore di normali ai triangoli usate per la visualizzazione: per aggiornare utilizzare il metodo update() */
        const std::vector<float> * pTriangleColors; /** \~Italian @brief vettore di colori associati ai triangoli (da considerare come triple rgb float) usati per la visualizzazione: per aggiornare utilizzare il metodo update() */

        #ifdef COMMON_WITH_EIGEN
        const Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> * pV;
        const Eigen::Matrix<int, Eigen::Dynamic, 3, Eigen::RowMajor> * pF;
        const Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> * pNV;
        const Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> * pCV;
        const Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> * pNF;
        const Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> * pCF;
        #endif

    protected:
        DrawableMesh(const std::vector<double> &coords, const std::vector<int> &triangles, const std::vector<double> &vertexNormals, const std::vector<float> &vertexColors, const std::vector<double> &triangleNormals, const std::vector<float> &triangleColors);
        #ifdef COMMON_WITH_EIGEN
        DrawableMesh(const Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>& V, const Eigen::Matrix<int, Eigen::Dynamic, 3, Eigen::RowMajor>& F, const Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>& NV, const Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>& CV, const Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>& NF, const Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>& CF);
        #endif

        void renderPass() const;
        void renderPass(unsigned int nv, unsigned int nt, const double* pCoords, const int* pTriangles, const double* pVertexNormals, const float* pVertexColors, const double* pTriangleNormals, const float* pTriangleColors) const;
        void updatePointers(const std::vector<double> &coords, const std::vector<int> &triangles, const std::vector<double> &vertexNormals, const std::vector<float> &vertexColors, const std::vector<double> &triangleNormals, const std::vector<float> &triangleColors);
        #ifdef COMMON_WITH_EIGEN
        void updatePointers(const Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>& V, const Eigen::Matrix<int, Eigen::Dynamic, 3, Eigen::RowMajor>& F, const Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>& NV, const Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>& CV, const Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>& NF, const Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>& CF);
        #endif

        enum {
            DRAW_MESH        = 0b00000001,
            DRAW_POINTS      = 0b00000010,
            DRAW_FLAT        = 0b00000100,
            DRAW_SMOOTH      = 0b00001000,
            DRAW_WIREFRAME   = 0b00010000,
            DRAW_FACECOLOR   = 0b00100000,
            DRAW_VERTEXCOLOR = 0b01000000
        };

        int   drawMode;
        MeshType meshType;
        int   wireframeWidth;
        float wireframeColor[3];
};

#endif // DRAWABLEMESH_H
