#pragma once

#include <stdio.h>
#include <cmath>
#include <array>
#include "nvmath/nvmath.h"


#define PI 3.14159265358979323846

static const char* basis[] = { "1","e0","e1","e2","e3","e01","e02","e03","e12","e31","e23","e021","e013","e032","e123","e0123" };

class PGA3D {
  public:
    nvmath::mat4f as_mat4x4();
    
    PGA3D();
    PGA3D(float f, int idx = 0);
    float& operator [] (size_t idx);
    const float& operator [] (size_t idx) const;
    PGA3D Conjugate(); 
    PGA3D Involute();
    float norm();
    float inorm();
    PGA3D normalized();
private:
    float mvec[16];
};

//***********************
// PGA3D.Reverse : res = ~a
// Reverse the order of the basis blades.
//***********************
PGA3D operator ~ (const PGA3D &a);

//***********************
// PGA3D.Dual : res = !a
// Poincare duality operator.
//***********************
PGA3D operator ! (const PGA3D &a) ;

//***********************
// PGA3D.Mul : res = a * b 
// The geometric product.
//***********************
PGA3D operator * (const PGA3D &a, const PGA3D &b);

//***********************
// PGA3D.Wedge : res = a ^ b 
// The outer product. (MEET)
//***********************
PGA3D operator ^ (const PGA3D &a, const PGA3D &b);

//***********************
// PGA3D.Vee : res = a & b 
// The regressive product. (JOIN)
//***********************
PGA3D operator & (const PGA3D &a, const PGA3D &b);

//***********************
// PGA3D.Dot : res = a | b 
// The inner product.
//***********************
PGA3D operator | (const PGA3D &a, const PGA3D &b);

//***********************
// PGA3D.Add : res = a + b 
// Multivector addition
//***********************
PGA3D operator + (const PGA3D &a, const PGA3D &b);

//***********************
// PGA3D.Sub : res = a - b 
// Multivector subtraction
//***********************
PGA3D operator - (const PGA3D &a, const PGA3D &b);

//***********************
// PGA3D.smul : res = a * b 
// scalar/multivector multiplication
//***********************
PGA3D operator * (const float &a, const PGA3D &b);

//***********************
// PGA3D.muls : res = a * b 
// multivector/scalar multiplication
//***********************
PGA3D operator * (const PGA3D &a, const float &b);

//***********************
// PGA3D.sadd : res = a + b 
// scalar/multivector addition
//***********************
PGA3D operator + (const float& a, const PGA3D& b);

//***********************
// PGA3D.adds : res = a + b 
// multivector/scalar addition
//***********************
PGA3D operator + (const PGA3D &a, const float &b);

//***********************
// PGA3D.ssub : res = a - b 
// scalar/multivector subtraction
//***********************
PGA3D operator - (const float &a, const PGA3D &b);

//***********************
// PGA3D.subs : res = a - b 
// multivector/scalar subtraction
//***********************
PGA3D operator - (const PGA3D &a, const float &b);


// A rotor (Euclidean line) and translator (Ideal line)
 PGA3D rotor(float angle, PGA3D line);
 PGA3D translator(float dist, PGA3D line);

// PGA is plane based. Vectors are planes. (think linear functionals)
static PGA3D e0(1.0f,1), e1(1.0f,2), e2(1.0f,3), e3(1.0f,4), pga_1(1.0f,0), pga_I(1.0f,15);

// A plane is defined using its homogenous equation ax + by + cz + d = 0
PGA3D plane(float a, float b, float c, float d);

static PGA3D  e01(1.0f,5), e02(1.0f,6), e03(1.0f,7), e12(1.0f,8), e31(1.0f,9), e23(1.0f,10);

// PGA points are trivectors.
static PGA3D e123(1.0f, 11), e032(1.0f, 12), e013 (1.0f, 13), e021(1.0f, 14);


// A point is just a homogeneous point, euclidean coordinates plus the origin
 PGA3D point(float x, float y, float z);
