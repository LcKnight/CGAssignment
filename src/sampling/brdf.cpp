#include "brdf.h"
#include "rng.h"
#include "../math/vec.h"
#include <cmath>
#include <algorithm>

static constexpr float PI = PT_PI;

Vec3 reflect(const Vec3& v, const Vec3& n) {
    return v - n*(2.f*v.dot(n));
}

bool refract(const Vec3& v, const Vec3& n, float eta, Vec3& out) {
    float cosI  = (-v).dot(n);
    float sin2T = eta*eta*(1.f-cosI*cosI);
    if(sin2T>1.f) return false;
    float cosT = std::sqrt(1.f-sin2T);
    out = v*eta + n*(eta*cosI - cosT);
    return true;
}

float schlick(float cosI, float eta) {
    float r0 = (1.f-eta)/(1.f+eta); r0*=r0;
    return r0 + (1.f-r0)*std::pow(1.f-cosI, 5.f);
}

void buildONB(const Vec3& n, Vec3& u, Vec3& v) {
    Vec3 a = (std::abs(n.x)>0.9f) ? Vec3(0,1,0) : Vec3(1,0,0);
    u = n.cross(a).normalize();
    v = n.cross(u);
}

Vec3 sampleCosineHemisphere(const Vec3& n, float u1, float u2) {
    Vec3 u,v; buildONB(n,u,v);
    float phi  = 2.f*PI*u1;
    float cosT = std::sqrt(u2);
    float sinT = std::sqrt(1.f-u2);
    return (u*std::cos(phi)*sinT + v*std::sin(phi)*sinT + n*cosT).normalize();
}

Vec3 samplePhongLobe(const Vec3& R, float Ns, float u1, float u2) {
    float cosA = std::pow(u1, 1.f/(Ns+1.f));
    float sinA = std::sqrt(1.f-cosA*cosA);
    float phi  = 2.f*PI*u2;
    Vec3 u,v; buildONB(R,u,v);
    return (u*std::cos(phi)*sinA + v*std::sin(phi)*sinA + R*cosA).normalize();
}

float powerHeuristic(float pdfA, float pdfB) {
    float a2=pdfA*pdfA;
    return a2/(a2+pdfB*pdfB+1e-10f);
}

Vec3 evalBRDF(const Vec3& Kd, const Vec3& Ks, float Ns,
              const Vec3& N, const Vec3& wo, const Vec3& wi) {
    Vec3  diff = Kd*(1.f/PI);
    Vec3  R    = reflect(-wo,N);
    float cosA = std::max(0.f, R.dot(wi));
    Vec3  spec = Ks*((Ns+2.f)/(2.f*PI))*std::pow(cosA,Ns);
    return diff+spec;
}

float pdfBRDF(const Vec3& Kd, const Vec3& Ks, float Ns,
              const Vec3& N, const Vec3& wo, const Vec3& wi) {
    float dMax  = Kd.maxComp(), sMax = Ks.maxComp();
    float pDiff = dMax/(dMax+sMax+1e-6f);
    float cosTh = std::max(0.f, N.dot(wi));
    float pdfD  = cosTh/PI;
    Vec3  R     = reflect(-wo,N);
    float cosA  = std::max(0.f, R.dot(wi));
    float pdfS  = (Ns+1.f)/(2.f*PI)*std::pow(cosA,Ns);
    return pDiff*pdfD + (1.f-pDiff)*pdfS;
}

Vec3 sampleBRDF(const Vec3& Kd, const Vec3& Ks, float Ns,
                const Vec3& N,  const Vec3& wo,
                Vec3& wi, float& pdf, uint32_t& rng) {
    float dMax  = Kd.maxComp(), sMax = Ks.maxComp();
    float pDiff = dMax/(dMax+sMax+1e-6f);
    if(randf(rng)<pDiff){
        wi = sampleCosineHemisphere(N, randf(rng), randf(rng));
    } else {
        Vec3 R = reflect(-wo,N);
        wi = samplePhongLobe(R, Ns, randf(rng), randf(rng));
        if(wi.dot(N)<=0){ pdf=0; return {0,0,0}; }
    }
    pdf = pdfBRDF(Kd,Ks,Ns,N,wo,wi);
    if(pdf<1e-8f){ pdf=0; return {0,0,0}; }
    return evalBRDF(Kd,Ks,Ns,N,wo,wi);
}