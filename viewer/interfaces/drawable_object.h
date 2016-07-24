/*
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @author    Marco Livesu (marco.livesu@gmail.com)
 * @copyright Marco Livesu 2014.
 */

#ifndef DRAWABLE_OBJECT_H
#define DRAWABLE_OBJECT_H

#include <float.h>
#include "../../common/point.h"

/**
 * \~English
 * @interface DrawableObject
 * @brief The DrawableObject Interface models a renderable model for a GLCanvas.
 *
 * It contains methods that must be implemented by the classes that inherit from a DrawableObject
 * to be rendered by a GLCanvas.
 *
 * @author Marco Livesu (marco.livesu@gmail.com)
 * @author Alessandro Muntoni (muntoni.alessandro@gmail.com)
 *
 * \~Italian
 * @interface DrawableObject
 * @brief L'interfaccia DrawableObject modella un oggetto renderizzabile da una GLCanvas.
 *
 * Contiene dei metodi che devono essere implementati dalle classi che ereditano da DrawableObject
 * per essere renderizzate da una GLCanvas.
 *
 * @author Marco Livesu (marco.livesu@gmail.com)
 * @author Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
class DrawableObject
{
    public :

        DrawableObject() {}                      /**< \~Italian @brief Costruttore, vuoto
                                                      \~English @brief Empty constructor */

        virtual ~DrawableObject() {}

        virtual void  draw()          const = 0; /**< \~Italian @brief Disegna l'oggetto mediante chiamate OpenGL
                                                      \~English @brief Draws the object through OpenGL */
        virtual Pointd sceneCenter()  const = 0; /**< \~Italian @brief Restituisce la posizione del centro dell'oggetto
                                                      \~English @brief Returns the position of the center of the objetc */
        virtual double sceneRadius()  const = 0; /**< \~Italian @brief Restituisce il raggio della bounding sphere (o del bounding box) dell'oggetto
                                                      \~English @brief Returns the ray of the bounding sphere (or the bounding box)* of the object */
        virtual bool isVisible()      const = 0; /**< \~Italian @brief Restituisce true se l'oggetto è visibile, false altrimenti
                                                      \~English @brief Returns true if the object is visible, false otherwise */
        virtual void setVisible(bool)       = 0; /**< \~Italian @brief Setta la visibilità dell'oggetto
                                                      \~English @brief Sets the visibility of the object */
};

#endif // DRAWABLE_OBJECT_H
