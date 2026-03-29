#pragma once
#include "../math/vec.h"
#include <cstdint>

// Sampling helpers
Vec3  reflect(const Vec3& v, const Vec3& n);
bool  refract(const Vec3& v, const Vec3& n, float etaRatio, Vec3& out);
float schlick(float cosI, float eta);
void  buildONB(const Vec3& n, Vec3& u, Vec3& v);
Vec3  sampleCosineHemisphere(const Vec3& n, float u1, float u2);
Vec3  samplePhongLobe(const Vec3& R, float Ns, float u1, float u2);
float powerHeuristic(float pdfA, float pdfB);

// Phong BRDF
Vec3  evalBRDF(const Vec3& Kd, const Vec3& Ks, float Ns,
               const Vec3& N,  const Vec3& wo, const Vec3& wi);
float pdfBRDF(const Vec3& Kd, const Vec3& Ks, float Ns,
              const Vec3& N,  const Vec3& wo, const Vec3& wi);
Vec3  sampleBRDF(const Vec3& Kd, const Vec3& Ks, float Ns,
                 const Vec3& N,  const Vec3& wo,
                 Vec3& wi, float& pdf, uint32_t& rng);