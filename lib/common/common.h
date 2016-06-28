/*
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @copyright Alessandro Muntoni 2016.
 */

/**
 * \~English
 * @brief This header contains some generic functions that may be useful.
 *
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @copyright Alessandro Muntoni 2016.
 *
 * \~Italian
 * @brief Questo header contiene alcune funzioni generiche che possono essere utili.
 *
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */

#ifndef LIB_COMMON_H
#define LIB_COMMON_H

#define EPSILON 0.0000001

#include <vector>
#include "point.h"
#ifdef CGAL_DEFINED
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Polygon_2.h>
#endif

/**
 * \~English
 * @brief This function computes a binary search of an element on a sorted std::vector
 * @param[in] n: the object that we are searching on the vector
 * @param[in] v: a sorted std::vector
 * @return the position index of the object if it is found, -1 otherwise
 *
 * \~Italian
 * @brief Questa funzione esegue una ricerca binaria di un elemento su un std::vector ordinato
 * @param[in] n: l'oggetto che si deve cercare all'interno del vector
 * @param[in] v: un std::vector ordinato
 * @return l'indice di posizione dell'oggetto se è stato trovato, -1 altrimenti
 */
template <typename T>
int binarySearch(const T &n, const std::vector<T> &v) {
    int first = 0, last =v.size()-1, mid;
    while (first <= last){
        mid = (first + last) / 2;
        if (v[mid] < n) first = mid + 1;
        else if (v[mid] == n) return mid;
        else last = mid - 1;
    }
    return -1;
}

/**
 * \~English
 * @brief This functions computer an equality between two parameters considering an epsilon offset
 * @param[in] x: first parameter
 * @param[in] v: second parameter
 * @param[in] epsilon: offset for equality, default value 0.0000001
 * @return true if the two parameters are equals on the epsilon interval, false otherwise
 *
 * \~Italian
 * @brief Questa funzione calcola l'uguaglianza tra due parametri tenendo conto di un possibile offset pari a epsilon
 * @param[in] x: primo parametro
 * @param[in] v: secondo parametro
 * @param[in] epsilon: offset di uguaglianza, valore di default 0.0000001
 * @return true se i due parametri sono uguali a meno del valore di epsilon, falso altrimenti
 */
template <typename T>
inline bool epsilonEqual(T x, T v, double epsilon = EPSILON) {
    if ((v-epsilon) <= x && (v+epsilon) >= x) return true;
    return false;
}

inline bool epsilonEqual(const Vec3 &x, const Vec3 &v, double epsilon = EPSILON) {
    if ((epsilonEqual(x.x(), v.x(), epsilon)) && (epsilonEqual(x.y(), v.y(), epsilon)) && (epsilonEqual(x.z(), v.z(), epsilon))) return true;
    else return false;

}

/**
  CGAL
  */
#ifdef CGAL_DEFINED
struct FaceInfo2
{
        FaceInfo2(){}
        int nesting_level;
        bool in_domain(){
            return nesting_level%2 == 1;
        }
};


typedef CGAL::Exact_predicates_inexact_constructions_kernel             K;
typedef CGAL::Triangulation_vertex_base_2<K>                            Vb;
typedef CGAL::Triangulation_face_base_with_info_2<FaceInfo2,K>          Fbb;
typedef CGAL::Constrained_triangulation_face_base_2<K,Fbb>              Fb;
typedef CGAL::Triangulation_data_structure_2<Vb,Fb>                     TDS;
typedef CGAL::Exact_predicates_tag                                      Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, TDS, Itag>        CDT;
typedef CDT::Point                                                      CGALPoint;
typedef CGAL::Polygon_2<K>                                              Polygon_2;

typedef CGAL::Epick                                                     E;
typedef CGAL::Triangulation_ds_face_base_2<TDS>                         TDFB2;
typedef CGAL::Triangulation_face_base_2<E, TDFB2>                       TFB2;
typedef CGAL::Triangulation_face_base_with_info_2<FaceInfo2, E, TFB2>   TFBI;
typedef CGAL::Constrained_triangulation_face_base_2<E, TFBI >           Triangle;

inline void markDomains(CDT& ct, CDT::Face_handle start, int index, std::list<CDT::Edge>& border){
    if(start->info().nesting_level != -1){
        return;
    }
    std::list<CDT::Face_handle> queue;
    queue.push_back(start);
    while(! queue.empty()){
        CDT::Face_handle fh = queue.front();
        queue.pop_front();
        if(fh->info().nesting_level == -1){
            fh->info().nesting_level = index;
            for(int i = 0; i < 3; i++){
                CDT::Edge e(fh,i);
                CDT::Face_handle n = fh->neighbor(i);
                if(n->info().nesting_level == -1){
                    if(ct.is_constrained(e)) border.push_back(e);
                    else queue.push_back(n);
                }
            }
        }
    }
}

inline void markDomains(CDT& cdt) {
    for(CDT::All_faces_iterator it = cdt.all_faces_begin(); it != cdt.all_faces_end(); ++it){
        it->info().nesting_level = -1;
    }
    std::list<CDT::Edge> border;
    markDomains(cdt, cdt.infinite_face(), 0, border);
    while(! border.empty()){
        CDT::Edge e = border.front();
        border.pop_front();
        CDT::Face_handle n = e.first->neighbor(e.second);
        if(n->info().nesting_level == -1){
            markDomains(cdt, n, e.first->info().nesting_level+1, border);
        }
    }
}
#endif

inline void getRotationMatrix(Vec3 axis, double angle, Eigen::Matrix3d &m) {
    axis.normalize();
    double cosa = cos(angle);
    double sina = sin(angle);
    m(0,0) = cosa + (axis.x() * axis.x())*(1-cosa);
    m(0,1) = axis.x() * axis.y() * (1-cosa) - axis.z() * sina;
    m(0,2) = axis.x() * axis.z() * (1-cosa) + axis.y() * sina;
    m(1,0) = axis.y() * axis.x() * (1-cosa) + axis.z() * sina;
    m(1,1) = cosa + (axis.y() * axis.y())*(1-cosa);
    m(1,2) = axis.y() * axis.z() * (1-cosa) - axis.x() * sina;
    m(2,0) = axis.z() * axis.x() * (1-cosa) - axis.y() * sina;
    m(2,1) = axis.z() * axis.y() * (1-cosa) + axis.x() * sina;
    m(2,2) = cosa + (axis.z() * axis.z())*(1-cosa);
}

inline void getRotationMatrix(Vec3 axis, double angle, double m[][3]) {
    axis.normalize();
    double cosa = cos(angle);
    double sina = sin(angle);
    m[0][0] = cosa + (axis.x() * axis.x())*(1-cosa);
    m[0][1] = axis.x() * axis.y() * (1-cosa) - axis.z() * sina;
    m[0][2] = axis.x() * axis.z() * (1-cosa) + axis.y() * sina;
    m[1][0] = axis.y() * axis.x() * (1-cosa) + axis.z() * sina;
    m[1][1] = cosa + (axis.y() * axis.y())*(1-cosa);
    m[1][2] = axis.y() * axis.z() * (1-cosa) - axis.x() * sina;
    m[2][0] = axis.z() * axis.x() * (1-cosa) - axis.y() * sina;
    m[2][1] = axis.z() * axis.y() * (1-cosa) + axis.x() * sina;
    m[2][2] = cosa + (axis.z() * axis.z())*(1-cosa);
}

#endif // LIB_COMMON_H

