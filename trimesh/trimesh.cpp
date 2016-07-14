#include "trimesh.h"
#include "load_save_trimesh.h"
#include "pca.h"

#include <assert.h>
#include <float.h>
#include <map>
#include <set>

using namespace  std;

typedef std::pair<int,int> edge;

static edge unique_edge(int v0, int v1)
{
    edge e;
    e.first  = std::min(v0,v1);
    e.second = std::max(v0,v1);
    return e;
}

Trimesh::Trimesh()
{
    type = TRIMESH;
}

Trimesh::Trimesh(const char * filename)
{
    load(filename);
    init();
}

Trimesh::Trimesh(const vector<double> & xyz,
                 const vector<int>    & tri)
{
    coords = xyz;
    tris   = tri;
    init();
}

void Trimesh::init()
{
    build_adjacency();
    updateNormals();
    updateBbox();
}

void Trimesh::clear()
{
    coords.clear();
    tris.clear();

    vtx2vtx.clear();
    vtx2tri.clear();
    tri2tri.clear();
}

void Trimesh::build_adjacency()
{
    vtx2vtx.clear();
    vtx2tri.clear();
    tri2tri.clear();

    vtx2vtx.resize(numVertices());
    vtx2tri.resize(numVertices());
    tri2tri.resize(numTriangles());

    set<edge>     edges;
    map<edge,int> edge2tri;

    for(int tid=0; tid<numTriangles(); ++tid)
    {
        int tid_ptr = tid * 3;
        for(int i=0; i<3; ++i)
        {
            int vid = tris[tid_ptr + i];
            vtx2tri[vid].push_back(tid);

            int adj = tris[tid_ptr + (i+1)%3];
            edge e = unique_edge(vid,adj);
            edges.insert(e);

            map<edge,int>::iterator query = edge2tri.find(e);
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

    for(set<edge>::iterator it=edges.begin(); it!=edges.end(); ++it)
    {
        edge e = *it;
        vtx2vtx[e.first].push_back(e.second);
        vtx2vtx[e.second].push_back(e.first);
    }
}

void Trimesh::update_t_normals()
{
    t_norm.clear();
    t_norm.resize(numTriangles()*3);

    for(int tid=0; tid<numTriangles(); ++tid)
    {
        int tid_ptr = tid * 3;

        vec3d v0 = vertex(tris[tid_ptr+0]);
        vec3d v1 = vertex(tris[tid_ptr+1]);
        vec3d v2 = vertex(tris[tid_ptr+2]);

        vec3d u = v1 - v0;    u.normalize();
        vec3d v = v2 - v0;    v.normalize();
        vec3d n = u.cross(v); n.normalize();

        t_norm[tid_ptr + 0] = n.x();
        t_norm[tid_ptr + 1] = n.y();
        t_norm[tid_ptr + 2] = n.z();
    }
}

void Trimesh::update_v_normals()
{
    v_norm.clear();
    v_norm.resize(numVertices()*3);

    for(int vid=0; vid<numVertices(); ++vid)
    {
        vector<int> nbrs = adj_vtx2tri(vid);

        vec3d sum(0,0,0);
        for(int i=0; i<(int)nbrs.size(); ++i)
        {
            sum += triangleNormal(nbrs[i]);
        }

        assert(nbrs.size() > 0);
        sum /= (double)nbrs.size();
        sum.normalize();

        int vid_ptr = vid * 3;
        v_norm[vid_ptr + 0] = sum.x();
        v_norm[vid_ptr + 1] = sum.y();
        v_norm[vid_ptr + 2] = sum.z();
    }
}


void Trimesh::load(const char * filename)
{
    clear();
    vector<double> tmp_coords;

    string str(filename);
    string filetype = str.substr(str.size()-3,3);

    if (filetype.compare("off") == 0 ||
        filetype.compare("OFF") == 0)
    {
        load_OFF(filename, tmp_coords, tris);
    }
    else
    if (filetype.compare("obj") == 0 ||
        filetype.compare("OBJ") == 0)
    {
        load_OBJ(filename, tmp_coords, tris);
    }
    else
    {
        cerr << "ERROR : " << __FILE__ << ", line " << __LINE__ << " : load() : file format not supported yet " << endl;
        exit(-1);
    }

    coords = tmp_coords;

    cout << endl;
    cout << tris.size()   / 3 << " triangles read" << endl;
    cout << coords.size() / 3 << " vertices  read" << endl;
    cout << endl;
}

void Trimesh::updateNormals()
{
    update_t_normals();
    update_v_normals();
}

void Trimesh::updateBbox()
{
    bbox.reset();
    for(int vid=0; vid<numVertices(); ++vid)
    {
        vec3d v = vertex(vid);
        bbox.min = bbox.min.min(v);
        bbox.max = bbox.max.max(v);
    }
}

void Trimesh::center_bbox()
{
    vec3d center = bbox.center();
    for(int vid=0; vid<numVertices(); ++vid)
    {
        vec3d pos = vertex(vid) - center;
        setVertex(vid, pos);
    }
    bbox.min -= center;
    bbox.max -= center;
}

void Trimesh::align_to_PCA()
{
    center_bbox();
    PCA(coords);
    updateBbox();
    updateNormals();
}
