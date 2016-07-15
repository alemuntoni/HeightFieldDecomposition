#ifndef DRAWABLEIGLMESH_H
#define DRAWABLEIGLMESH_H

#include "common/drawable_object.h"
#include "iglmesh.h"

class DrawableIGLMesh : public IGLMesh, public DrawableObject {
    public:
        DrawableIGLMesh();
        DrawableIGLMesh(const SimpleIGLMesh &m);
        DrawableIGLMesh(const IGLMesh &m);
        virtual ~DrawableIGLMesh();

        void init();
        void update();

        // DrawableObject interface
        void draw() const;
        Pointd sceneCenter() const;
        double sceneRadius() const;
        bool isVisible() const;
        void setVisible(bool);

        // rendering options
        //
        void setWireframe(bool b);
        void setFlatShading();
        void setSmoothShading();
        void setPointsShading();

    protected:

        void renderPass() const;

        enum {
            DRAW_MESH           = 0b00000001,
            DRAW_POINTS         = 0b00000010,
            DRAW_FLAT           = 0b00000100,
            DRAW_SMOOTH         = 0b00001000,
            DRAW_WIREFRAME      = 0b00010000,
            DRAW_FACECOLOR      = 0b00100000,
        };

        int   drawMode; /** \~Italian @brief intero interpretato come stringa di bit rappresentante la modalità di visualizzazione della dcel*/
        std::vector<double> coords; /** \~Italian @brief vettore di coordinate usate per la visualizzazione: per aggiornare utilizzare metodo update() */
        std::vector<double> v_norm; /** \~Italian @brief vettore di normali ai vertici usate per la visualizzazione: per aggiornare utilizzare il metodo update() */
        std::vector<int> tris; /** \~Italian @brief vettore di triangoli (da considerare a triple di indici) usati per la visualizzazione: per aggiornare utilizzare il metodo update() */
        std::vector<float> colors; /** \~Italian @brief vettore di colori associati ai triangoli (da considerare come triple rgb float) usati per la visualizzazione: per aggiornare utilizzare il metodo update() */
        std::vector<float> wireframe_colors;

};

#endif // DRAWABLEIGLMESH_H
