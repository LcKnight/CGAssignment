#pragma once
#include "../math/vec.h"
#include "../math/ray.h"
#include "mesh.h"
#include <vector>
#include <algorithm>
#include <atomic>

// ----------------------------------------------------------------
// AABB
// ----------------------------------------------------------------
struct AABB {
    Vec3 mn{ PT_INF, PT_INF, PT_INF};
    Vec3 mx{-PT_INF,-PT_INF,-PT_INF};

    void expand(const Vec3& p) {
        for(int i=0;i<3;i++){
            if(p[i]<mn[i]) mn[i]=p[i];
            if(p[i]>mx[i]) mx[i]=p[i];
        }
    }
    void expand(const AABB& b) { expand(b.mn); expand(b.mx); }

    bool hit(const Ray& r, float tmin, float tmax) const {
        for(int a=0;a<3;a++){
            float inv = 1.f/r.d[a];
            float t0  = (mn[a]-r.o[a])*inv;
            float t1  = (mx[a]-r.o[a])*inv;
            if(inv<0) std::swap(t0,t1);
            tmin = std::max(tmin,t0);
            tmax = std::min(tmax,t1);
            if(tmax < tmin) return false;
        }
        return true;
    }
};

// ----------------------------------------------------------------
// BVHNode
// ----------------------------------------------------------------
struct BVHNode {
    AABB box;
    int  left=-1, right=-1, triStart=-1, triCount=0;
    bool isLeaf() const { return left==-1; }
};

// ----------------------------------------------------------------
// Moller-Trumbore triangle intersection
// ----------------------------------------------------------------
inline bool intersectTri(const Ray& r, const Triangle& tri,
                         float tmin, float tmax, HitRecord& rec)
{
    Vec3 e1=tri.v1-tri.v0, e2=tri.v2-tri.v0;
    Vec3 h=r.d.cross(e2);
    float a=e1.dot(h);
    if(std::abs(a)<1e-15f) return false;
    float f=1.f/a;
    Vec3  s=r.o-tri.v0;
    float u=f*s.dot(h);
    if(u<0||u>1) return false;
    Vec3  q=s.cross(e1);
    float v=f*r.d.dot(q);
    if(v<0||u+v>1) return false;
    float t=f*e2.dot(q);
    if(t<tmin||t>tmax) return false;
    rec.t   = t;
    rec.pos = r.at(t);
    float w = 1.f-u-v;
    rec.geomNormal = e1.cross(e2).normalize();
    rec.normal = (tri.n0*w + tri.n1*u + tri.n2*v).normalize();
    rec.uv     = {tri.uv0.x*w+tri.uv1.x*u+tri.uv2.x*v,
                  tri.uv0.y*w+tri.uv1.y*u+tri.uv2.y*v};
    rec.matId  = tri.matId;
    rec.front  = r.d.dot(rec.geomNormal) < 0;
    if(rec.normal.lengthSq() < 1e-8f) rec.normal = rec.geomNormal;
    if(!rec.front) rec.normal = -rec.normal;
    if(!rec.front) rec.geomNormal = -rec.geomNormal;
    return true;
}

// ----------------------------------------------------------------
// BVH
// ----------------------------------------------------------------
struct BVH {
    std::vector<BVHNode> nodes;
    std::vector<int>     idx;   // sorted triangle indices

    void build(const std::vector<Triangle>& tris);

    bool intersect(const Ray& r,
                   const std::vector<Triangle>& tris,
                   const std::vector<Texture>&  texs,
                   const std::vector<Material>& mats,
                   float tmin, float tmax, HitRecord& rec) const;

    bool occluded(const Ray& r,
                  const std::vector<Triangle>& tris,
                  const std::vector<Texture>&  texs,
                  const std::vector<Material>& mats,
                  float tmin, float tmax) const;

private:
    int buildRec(const std::vector<Triangle>& tris,
                 int start, int end, int depth,
                 std::atomic<int>& nextNode);

    bool traverseClosest(const Ray& r,
                         const std::vector<Triangle>& tris,
                         const std::vector<Texture>&  texs,
                         const std::vector<Material>& mats,
                         int nodeId, float tmin, float tmax,
                         HitRecord& rec) const;

    bool traverseShadow(const Ray& r,
                        const std::vector<Triangle>& tris,
                        const std::vector<Texture>&  texs,
                        const std::vector<Material>& mats,
                        int nodeId, float tmin, float tmax) const;
};
