#pragma once
#include <cmath>
#include <algorithm>

static constexpr float PT_PI  = 3.14159265358979323846f;
static constexpr float PT_INF = 1e30f;
static constexpr float PT_EPS = 1e-5f;

// ----------------------------------------------------------------
// Vec2
// ----------------------------------------------------------------
struct Vec2 { float x, y; };

// ----------------------------------------------------------------
// Vec3  (used by both PathTracer and VulkanApp)
// ----------------------------------------------------------------
struct Vec3 {
    float x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(float a, float b, float c) : x(a), y(b), z(c) {}

    Vec3 operator+(const Vec3& v) const { return {x+v.x, y+v.y, z+v.z}; }
    Vec3 operator-(const Vec3& v) const { return {x-v.x, y-v.y, z-v.z}; }
    Vec3 operator*(float s)       const { return {x*s,   y*s,   z*s};   }
    Vec3 operator*(const Vec3& v) const { return {x*v.x, y*v.y, z*v.z}; }
    Vec3 operator/(float s)       const { float i=1.f/s; return *this*i; }
    Vec3 operator-()              const { return {-x,-y,-z}; }
    Vec3& operator+=(const Vec3& v){ x+=v.x; y+=v.y; z+=v.z; return *this; }

    float dot(const Vec3& v)   const { return x*v.x+y*v.y+z*v.z; }
    Vec3  cross(const Vec3& v) const {
        return {y*v.z-z*v.y, z*v.x-x*v.z, x*v.y-y*v.x};
    }
    float length()   const { return std::sqrt(x*x+y*y+z*z); }
    float lengthSq() const { return x*x+y*y+z*z; }
    Vec3  normalize() const {
        float l = length();
        return l > 0 ? *this*(1.f/l) : Vec3(0,0,0);
    }
    float maxComp() const { return std::max({x,y,z}); }
    const float& operator[](int i) const { return (&x)[i]; }
    float&       operator[](int i)       { return (&x)[i]; }
};
inline Vec3 operator*(float s, const Vec3& v) { return v*s; }

// ----------------------------------------------------------------
// Mat4  (used by VulkanApp for transforms)
// ----------------------------------------------------------------
struct Mat4 {
    float m[4][4];

    static Mat4 identity() {
        Mat4 r{}; for(int i=0;i<4;i++) r.m[i][i]=1.f; return r;
    }
    static Mat4 scale(float s) {
        Mat4 r = identity();
        r.m[0][0]=s; r.m[1][1]=s; r.m[2][2]=s; return r;
    }
    static Mat4 translate(float x, float y, float z) {
        Mat4 r = identity();
        r.m[0][3]=x; r.m[1][3]=y; r.m[2][3]=z; return r;
    }
    static Mat4 perspective(float fov, float aspect, float znear, float zfar) {
        Mat4 r{};
        float t = std::tan(fov/2.f);
        r.m[0][0] =  1.f/(aspect*t);
        r.m[1][1] = -1.f/t;
        r.m[2][2] = -zfar/(zfar-znear);
        r.m[2][3] = -(zfar*znear)/(zfar-znear);
        r.m[3][2] = -1.f;
        return r;
    }
    static Mat4 rotateX(float a) {
        Mat4 r = identity();
        float c=std::cos(a), s=std::sin(a);
        r.m[1][1]=c; r.m[1][2]=-s; r.m[2][1]=s; r.m[2][2]=c; return r;
    }
    static Mat4 rotateY(float a) {
        Mat4 r = identity();
        float c=std::cos(a), s=std::sin(a);
        r.m[0][0]=c; r.m[0][2]=s; r.m[2][0]=-s; r.m[2][2]=c; return r;
    }
    static Mat4 rotateZ(float a) {
        Mat4 r = identity();
        float c=std::cos(a), s=std::sin(a);
        r.m[0][0]=c; r.m[0][1]=-s; r.m[1][0]=s; r.m[1][1]=c; return r;
    }
    Vec3 transformPoint(const Vec3& v) const {
        float x=m[0][0]*v.x+m[0][1]*v.y+m[0][2]*v.z+m[0][3];
        float y=m[1][0]*v.x+m[1][1]*v.y+m[1][2]*v.z+m[1][3];
        float z=m[2][0]*v.x+m[2][1]*v.y+m[2][2]*v.z+m[2][3];
        float w=m[3][0]*v.x+m[3][1]*v.y+m[3][2]*v.z+m[3][3];
        if(w!=0.f){x/=w;y/=w;z/=w;}
        return {x,y,z};
    }
    Mat4 operator*(const Mat4& r) const {
        Mat4 res{};
        for(int i=0;i<4;i++)
            for(int j=0;j<4;j++)
                for(int k=0;k<4;k++)
                    res.m[i][j]+=m[i][k]*r.m[k][j];
        return res;
    }
};
