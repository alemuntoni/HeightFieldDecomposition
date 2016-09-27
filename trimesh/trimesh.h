/**
 @author    Marco Livesu (marco.livesu@gmail.com)
 @copyright Marco Livesu 2014.
*/

#ifndef TRIMESH_H
#define TRIMESH_H

#include <assert.h>
#include <float.h>
#include <vector>
#include <map>
#include <set>
#include <stdlib.h>

#include "../common/bounding_box.h"
#include "../common/point.h"
#include "load_save_trimesh.h"

#ifdef IGL_DEFINED
#include "../igl/iglmesh.h"
#endif

//using namespace std;

typedef std::pair<int,int> edge;

/*-------------- INLINE UTILITY FUNCTIONS --------------*/

inline edge uniqueEdge(int v0, int v1)
{
    edge e;
    e.first  = std::min(v0,v1);
    e.second = std::max(v0,v1);
    return e;
}

template<typename C>
inline void checkBounds(const C & container, int index)
{
    assert(index < (int)container.size());
}

/*-------------- INLINE UTILITY FUNCTIONS --------------*/

template<typename real> class Trimesh
{

    public:

        Trimesh(){}

        Trimesh(const char * filename)
        {
            load(filename);
            init();
        }

        Trimesh(const std::vector<real> & coords,
                const std::vector<int>  & tris)
        {
            coords = coords;
            this->tris   = tris;
            init();
        }

        Trimesh(const IGLInterface::SimpleIGLMesh& simpleIGLMesh)
        {
            unsigned int nVertices=simpleIGLMesh.getNumberVertices();
            unsigned int nFaces=simpleIGLMesh.getNumberFaces();

            coords.resize(nVertices*3);
            tris.resize(nFaces*3);

            for(unsigned int i=0;i<nVertices;++i)
            {
                unsigned int j=i*3;
                coords[j]=simpleIGLMesh.getVertex(i).x();
                coords[j+1]=simpleIGLMesh.getVertex(i).y();
                coords[j+2]=simpleIGLMesh.getVertex(i).z();

            }

            for(unsigned int i=0;i<nFaces;++i)
            {
                unsigned int j=i*3;
                tris[j]=simpleIGLMesh.getFace(i).x();
                tris[j+1]=simpleIGLMesh.getFace(i).y();
                tris[j+2]=simpleIGLMesh.getFace(i).z();
            }

            init();

        }

        inline BoundingBox getBoundingBox() const {
            return bbox;
        }


    protected:

        BoundingBox                bbox;
        std::vector<real>          coords;
        std::vector<int>           tris;
        std::vector<real>          vertexNormals;
        std::vector<real>          triangleNormals;
        std::vector< std::vector<int> > vtx2tri;
        std::vector< std::vector<int> > vtx2vtx;
        std::vector< std::vector<int> > tri2tri;

        std::vector<real> toRealArray (const std::vector<double> & in) const
        {
            std::vector<real> res(in.size());
            for(int i=0; i<(int)in.size(); i++)
            {
                res[i] = in[i];
            }
            return res;
        }

        std::vector<double> fromRealArray(const std::vector<real> & in) const
        {
            std::vector<double> res(in.size());
            for( int i=0; i<in.size(); i++ )
            {
                res[i] = in[i];
            }
            return res;
        }

        void clear()
        {
            coords.clear();
            tris.clear();
            vtx2vtx.clear();
            vtx2tri.clear();
            tri2tri.clear();
        }

        void init()
        {
            buildAdjacency();
            updateNormals();
            updateBbox();
        }

        void buildAdjacency()
        {
            vtx2vtx.clear();
            vtx2tri.clear();
            tri2tri.clear();

            vtx2vtx.resize(numVertices());
            vtx2tri.resize(numVertices());
            tri2tri.resize(numTriangles());

            std::set<edge>     edges;
            std::map<edge,int> edge2tri;

            for(int tid=0; tid<numTriangles(); ++tid)
            {
                int tid_ptr = tid * 3;
                for(int i=0; i<3; ++i)
                {
                    int vid = tris[tid_ptr + i];
                    vtx2tri[vid].push_back(tid);

                    int adj = tris[tid_ptr + (i+1)%3];
                    edge e = uniqueEdge(vid,adj);
                    edges.insert(e);

                    std::map<edge,int>::iterator query = edge2tri.find(e);
                    if (query == edge2tri.end())
                    {
                        edge2tri[e] = tid;
                    }
                    else
                    {
                        int nbr_tri = query->second;
                        tri2tri[tid].push_back(nbr_tri);
                        tri2tri[nbr_tri].push_back(tid);
                    }
                }
            }

            for(std::set<edge>::iterator it=edges.begin(); it!=edges.end(); ++it)
            {
                edge e = *it;
                vtx2vtx[e.first].push_back(e.second);
                vtx2vtx[e.second].push_back(e.first);
            }
        }

        void updateTriangleNormals()
        {
            triangleNormals.clear();
            triangleNormals.resize(numTriangles()*3);

            for(int tid=0; tid<numTriangles(); ++tid)
            {
                int tid_ptr = tid * 3;

                Point<real> v0 = vertex(tris[tid_ptr+0]);
                Point<real> v1 = vertex(tris[tid_ptr+1]);
                Point<real> v2 = vertex(tris[tid_ptr+2]);

                Point<real> u = v1 - v0;    u.normalize();
                Point<real> v = v2 - v0;    v.normalize();
                Point<real> n = u.cross(v); n.normalize();

                triangleNormals[tid_ptr + 0] = n.x();
                triangleNormals[tid_ptr + 1] = n.y();
                triangleNormals[tid_ptr + 2] = n.z();
            }
        }

        void updateVertexNormals()
        {
                    vertexNormals.clear();
                    vertexNormals.resize(numVertices()*3);

                    for(int vid=0; vid<numVertices(); ++vid)
                    {
                        std::vector<int> nbrs = adj_vtx2tri(vid);
                        int vid_ptr = vid * 3;
                        if (nbrs.size() == 0) {
                            vertexNormals[vid_ptr + 0] = 0;
                            vertexNormals[vid_ptr + 1] = 0;
                            vertexNormals[vid_ptr + 2] = 0;
                        }
                        else {
                            Point<real> sum(0,0,0);
                            for(int i=0; i<(int)nbrs.size(); ++i)
                            {
                                sum += triangleNormal(nbrs[i]);
                            }

                            assert(nbrs.size() > 0);
                            sum /= nbrs.size();
                            sum.normalize();


                            vertexNormals[vid_ptr + 0] = sum.x();
                            vertexNormals[vid_ptr + 1] = sum.y();
                            vertexNormals[vid_ptr + 2] = sum.z();
                        }
                    }
        }

        void load(const char * filename)
        {
            clear();
            std::vector<double> tmp_coords;

            std::string str(filename);
            std::string filetype = str.substr(str.size()-3,3);

            if (filetype.compare("obj") == 0 ||
                filetype.compare("OBJ") == 0)
            {
                loadObj(filename, tmp_coords, tris);
            }
            else
            {
                std::cerr << "ERROR : " << __FILE__ << ", line " << __LINE__ << " : load() : file format not supported yet " << std::endl;
                exit(-1);
            }

            coords = toRealArray(tmp_coords);

            std::cout << tris.size() / 3   << " triangles read" << std::endl;
            std::cout << coords.size() / 3 << " vertices  read" << std::endl;

            buildAdjacency();
            updateNormals();
            updateBbox();
        }

    public:

        inline std::vector<real> & vectorCoords()     { return coords;        }
        inline std::vector<int>  & vectorTriangles()  { return tris;          }
        inline std::vector<real> & vectorVertexNormals()    { return vertexNormals; }
        inline const std::vector<std::vector<int>> & vectorAdjTri2Tri() {return tri2tri;}

        inline int numVertices()  const { return coords.size()/3; }
        inline int numTriangles() const { return tris.size()/3;   }

        inline std::vector<int> adj_vtx2tri(int vid) const { checkBounds(vtx2tri, vid); return vtx2tri[vid]; }
        inline std::vector<int> adj_vtx2vtx(int vid) const { checkBounds(vtx2vtx, vid); return vtx2vtx[vid]; }
        inline std::vector<int> adj_tri2tri(int tid) const { checkBounds(tri2tri, tid); return tri2tri[tid]; }

        inline int tri_vertex_id(int t_id, int v_id) const
        {
            return tris[t_id*3 + v_id];
        }

        inline Point<real> triangleNormal(int tid) const
        {
            int tid_ptr = tid * 3;
            checkBounds(triangleNormals, tid_ptr+2);
            return Point<real>(triangleNormals[tid_ptr + 0],
                              triangleNormals[tid_ptr + 1],
                              triangleNormals[tid_ptr + 2]);
        }

        inline Point<real> vertexNormal(int vid) const
        {
            int vid_ptr = vid * 3;
            checkBounds(vertexNormals, vid_ptr+2);
            return Point<real>(vertexNormals[vid_ptr + 0],
                              vertexNormals[vid_ptr + 1],
                              vertexNormals[vid_ptr + 2]);
        }

        inline Point<real> vertex(int vid) const
        {
            int vid_ptr = vid * 3;
            checkBounds(coords, vid_ptr+2);
            return Point<real>(coords[vid_ptr + 0],
                              coords[vid_ptr + 1],
                              coords[vid_ptr + 2]);
        }

        inline void setVertex(int vid, Point<real> pos)
        {
            int vid_ptr = vid * 3;
            checkBounds(coords, vid_ptr+2);
            coords[vid_ptr + 0] = pos.x();
            coords[vid_ptr + 1] = pos.y();
            coords[vid_ptr + 2] = pos.z();
        }

        void updateNormals()
        {
            updateTriangleNormals();
            updateVertexNormals();
        }

        void updateBbox()
        {
            bbox.reset();
            for(int vid=0; vid<numVertices(); ++vid)
            {
                Point<real> v = vertex(vid);
                bbox.setMin(bbox.getMin().min(Pointd(v.x(), v.y(), v.z())));
                bbox.setMax(bbox.getMax().max(Pointd(v.x(), v.y(), v.z())));
            }
        }


};

#endif // TRIMESH_H
