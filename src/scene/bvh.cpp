#include "bvh.h"
#include <algorithm>
#include <atomic>
#include <limits>
#include <cstdlib>

#ifdef _OPENMP
#include <omp.h>
#endif

#if defined(_OPENMP) && (_OPENMP >= 200805)
#define PT_BVH_OMP_TASKS 1
#else
#define PT_BVH_OMP_TASKS 0
#endif

void BVH::build(const std::vector<Triangle>& tris) {
    if(tris.empty()){
        idx.clear();
        nodes.clear();
        return;
    }

    idx.resize(tris.size());
    for(int i=0;i<(int)tris.size();i++) idx[i]=i;

    size_t maxNodesSize = std::max<size_t>(1, tris.size() * 2);
    if(maxNodesSize > (size_t)std::numeric_limits<int>::max()) {
        maxNodesSize = (size_t)std::numeric_limits<int>::max();
    }
    const int maxNodes = (int)maxNodesSize;
    nodes.clear();
    nodes.resize(maxNodes);

    std::atomic<int> nextNode{0};

#if PT_BVH_OMP_TASKS
    #pragma omp parallel
    {
        #pragma omp single nowait
        {
            buildRec(tris, 0, (int)tris.size(), 0, nextNode);
        }
    }
#else
    buildRec(tris, 0, (int)tris.size(), 0, nextNode);
#endif

    int used = std::max(1, nextNode.load(std::memory_order_relaxed));
    nodes.resize(used);
}

int BVH::buildRec(const std::vector<Triangle>& tris,
                  int start, int end, int depth,
                  std::atomic<int>& nextNode) {
    int id = nextNode.fetch_add(1, std::memory_order_relaxed);
    if(id < 0 || id >= (int)nodes.size()) {
        std::abort();
    }
    BVHNode& node = nodes[id];
    node = {};

    for(int i=start;i<end;i++){
        const Triangle& t = tris[idx[i]];
        node.box.expand(t.v0); node.box.expand(t.v1); node.box.expand(t.v2);
    }

    int count = end-start;
    if(count<=4 || depth>28){
        node.triStart=start; node.triCount=count; return id;
    }

    Vec3  ext  = node.box.mx - node.box.mn;
    int   axis = (ext.x>=ext.y && ext.x>=ext.z)?0:(ext.y>=ext.z?1:2);
    float mid  = (node.box.mn[axis]+node.box.mx[axis])*0.5f;

    int m = (int)(std::partition(idx.data()+start, idx.data()+end,
        [&](int i){
            const Triangle& t=tris[i];
            return (t.v0[axis]+t.v1[axis]+t.v2[axis])/3.f < mid;
        }) - idx.data());

    if(m==start||m==end) m=start+count/2;

    int leftId = -1;
    int rightId = -1;

#if PT_BVH_OMP_TASKS
    const bool spawnParallel = (depth < 3) && (count >= 4096);
    if(spawnParallel){
        #pragma omp task shared(leftId, nextNode, tris) firstprivate(start, m, depth)
        leftId = buildRec(tris, start, m, depth+1, nextNode);

        #pragma omp task shared(rightId, nextNode, tris) firstprivate(m, end, depth)
        rightId = buildRec(tris, m, end, depth+1, nextNode);

        #pragma omp taskwait
    } else {
        leftId  = buildRec(tris, start, m, depth+1, nextNode);
        rightId = buildRec(tris, m, end, depth+1, nextNode);
    }
#else
    leftId  = buildRec(tris, start, m, depth+1, nextNode);
    rightId = buildRec(tris, m, end, depth+1, nextNode);
#endif

    node.left  = leftId;
    node.right = rightId;
    return id;
}

bool BVH::intersect(const Ray& r, const std::vector<Triangle>& tris,
                    const std::vector<Texture>& texs,
                    const std::vector<Material>& mats,
                    float tmin, float tmax, HitRecord& rec) const {
    return traverseClosest(r, tris, texs, mats, 0, tmin, tmax, rec);
}

bool BVH::traverseClosest(const Ray& r, const std::vector<Triangle>& tris,
                           const std::vector<Texture>& texs,
                           const std::vector<Material>& mats,
                           int nodeId, float tmin, float tmax,
                           HitRecord& rec) const {
    const BVHNode& n = nodes[nodeId];
    if(!n.box.hit(r,tmin,tmax)) return false;
    if(n.isLeaf()){
        bool hit=false;
        for(int i=n.triStart;i<n.triStart+n.triCount;i++){
            HitRecord tmp;
            if(intersectTri(r, tris[idx[i]], tmin, tmax, tmp)){
                const Material& mat=mats[tmp.matId];
                if(mat.texD>=0 && texs[mat.texD].valid())
                    if(texs[mat.texD].sampleAlpha(tmp.uv)<0.5f) continue;
                tmax=tmp.t; rec=tmp; hit=true;
            }
        }
        return hit;
    }
    bool h1=traverseClosest(r,tris,texs,mats,n.left, tmin,tmax,rec);
    if(h1) tmax=rec.t;
    bool h2=traverseClosest(r,tris,texs,mats,n.right,tmin,tmax,rec);
    return h1||h2;
}

bool BVH::occluded(const Ray& r, const std::vector<Triangle>& tris,
                   const std::vector<Texture>& texs,
                   const std::vector<Material>& mats,
                   float tmin, float tmax) const {
    return traverseShadow(r,tris,texs,mats,0,tmin,tmax);
}

bool BVH::traverseShadow(const Ray& r, const std::vector<Triangle>& tris,
                          const std::vector<Texture>& texs,
                          const std::vector<Material>& mats,
                          int nodeId, float tmin, float tmax) const {
    const BVHNode& n = nodes[nodeId];
    if(!n.box.hit(r,tmin,tmax)) return false;
    if(n.isLeaf()){
        for(int i=n.triStart;i<n.triStart+n.triCount;i++){
            HitRecord tmp;
            if(intersectTri(r, tris[idx[i]], tmin, tmax, tmp)){
                const Material& mat=mats[tmp.matId];
                if(mat.texD>=0 && texs[mat.texD].valid())
                    if(texs[mat.texD].sampleAlpha(tmp.uv)<0.5f) continue;
                return true;
            }
        }
        return false;
    }
    return traverseShadow(r,tris,texs,mats,n.left, tmin,tmax)
        || traverseShadow(r,tris,texs,mats,n.right,tmin,tmax);
}
