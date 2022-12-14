// 3D Projective Geometric Algebra
// Written by a generator written by enki.
#include "pga3d.hpp"
#define POW(a) ((a)*(a))


PGA3D::PGA3D() {
  std::fill(mvec, mvec + sizeof(mvec) / 4, 0.0f);
}

PGA3D::PGA3D(float f, int idx) { std::fill(mvec, mvec + sizeof(mvec) / 4, 0.0f); mvec[idx] = f; }


float& PGA3D::operator [] (size_t idx) { return mvec[idx]; }
const float& PGA3D::operator [] (size_t idx) const { return mvec[idx]; }
//***********************
// PGA3D.Reverse : res = ~a
// Reverse the order of the basis blades.
//***********************
PGA3D operator ~ (const PGA3D& a) {
  PGA3D res;
  res[0] = a[0];
  res[1] = a[1];
  res[2] = a[2];
  res[3] = a[3];
  res[4] = a[4];
  res[5] = -a[5];
  res[6] = -a[6];
  res[7] = -a[7];
  res[8] = -a[8];
  res[9] = -a[9];
  res[10] = -a[10];
  res[11] = -a[11];
  res[12] = -a[12];
  res[13] = -a[13];
  res[14] = -a[14];
  res[15] = a[15];
  return res;
};

//***********************
// PGA3D.Dual : res = !a
// Poincare duality operator.
//***********************
PGA3D operator ! (const PGA3D& a) {
  PGA3D res;
  res[0] = a[15];
  res[1] = a[14];
  res[2] = a[13];
  res[3] = a[12];
  res[4] = a[11];
  res[5] = a[10];
  res[6] = a[9];
  res[7] = a[8];
  res[8] = a[7];
  res[9] = a[6];
  res[10] = a[5];
  res[11] = a[4];
  res[12] = a[3];
  res[13] = a[2];
  res[14] = a[1];
  res[15] = a[0];
  return res;
};

//***********************
// PGA3D.Conjugate : res = a.Conjugate()
// Clifford Conjugation
//***********************
PGA3D PGA3D::Conjugate() {
  PGA3D res;
  res[0] = this->mvec[0];
  res[1] = -this->mvec[1];
  res[2] = -this->mvec[2];
  res[3] = -this->mvec[3];
  res[4] = -this->mvec[4];
  res[5] = -this->mvec[5];
  res[6] = -this->mvec[6];
  res[7] = -this->mvec[7];
  res[8] = -this->mvec[8];
  res[9] = -this->mvec[9];
  res[10] = -this->mvec[10];
  res[11] = this->mvec[11];
  res[12] = this->mvec[12];
  res[13] = this->mvec[13];
  res[14] = this->mvec[14];
  res[15] = this->mvec[15];
  return res;
};

//***********************
// PGA3D.Involute : res = a.Involute()
// Main involution
//***********************
PGA3D PGA3D::Involute() {
  PGA3D res;
  res[0] = this->mvec[0];
  res[1] = -this->mvec[1];
  res[2] = -this->mvec[2];
  res[3] = -this->mvec[3];
  res[4] = -this->mvec[4];
  res[5] = this->mvec[5];
  res[6] = this->mvec[6];
  res[7] = this->mvec[7];
  res[8] = this->mvec[8];
  res[9] = this->mvec[9];
  res[10] = this->mvec[10];
  res[11] = -this->mvec[11];
  res[12] = -this->mvec[12];
  res[13] = -this->mvec[13];
  res[14] = -this->mvec[14];
  res[15] = this->mvec[15];
  return res;
};

//***********************
// PGA3D.Mul : res = a * b 
// The geometric product.
//***********************
PGA3D operator * (const PGA3D& a, const PGA3D& b) {
  PGA3D res;
  res[0] = b[0] * a[0] + b[2] * a[2] + b[3] * a[3] + b[4] * a[4] - b[8] * a[8] - b[9] * a[9] - b[10] * a[10] - b[14] * a[14];
  res[1] = b[1] * a[0] + b[0] * a[1] - b[5] * a[2] - b[6] * a[3] - b[7] * a[4] + b[2] * a[5] + b[3] * a[6] + b[4] * a[7] + b[11] * a[8] + b[12] * a[9] + b[13] * a[10] + b[8] * a[11] + b[9] * a[12] + b[10] * a[13] + b[15] * a[14] - b[14] * a[15];
  res[2] = b[2] * a[0] + b[0] * a[2] - b[8] * a[3] + b[9] * a[4] + b[3] * a[8] - b[4] * a[9] - b[14] * a[10] - b[10] * a[14];
  res[3] = b[3] * a[0] + b[8] * a[2] + b[0] * a[3] - b[10] * a[4] - b[2] * a[8] - b[14] * a[9] + b[4] * a[10] - b[9] * a[14];
  res[4] = b[4] * a[0] - b[9] * a[2] + b[10] * a[3] + b[0] * a[4] - b[14] * a[8] + b[2] * a[9] - b[3] * a[10] - b[8] * a[14];
  res[5] = b[5] * a[0] + b[2] * a[1] - b[1] * a[2] - b[11] * a[3] + b[12] * a[4] + b[0] * a[5] - b[8] * a[6] + b[9] * a[7] + b[6] * a[8] - b[7] * a[9] - b[15] * a[10] - b[3] * a[11] + b[4] * a[12] + b[14] * a[13] - b[13] * a[14] - b[10] * a[15];
  res[6] = b[6] * a[0] + b[3] * a[1] + b[11] * a[2] - b[1] * a[3] - b[13] * a[4] + b[8] * a[5] + b[0] * a[6] - b[10] * a[7] - b[5] * a[8] - b[15] * a[9] + b[7] * a[10] + b[2] * a[11] + b[14] * a[12] - b[4] * a[13] - b[12] * a[14] - b[9] * a[15];
  res[7] = b[7] * a[0] + b[4] * a[1] - b[12] * a[2] + b[13] * a[3] - b[1] * a[4] - b[9] * a[5] + b[10] * a[6] + b[0] * a[7] - b[15] * a[8] + b[5] * a[9] - b[6] * a[10] + b[14] * a[11] - b[2] * a[12] + b[3] * a[13] - b[11] * a[14] - b[8] * a[15];
  res[8] = b[8] * a[0] + b[3] * a[2] - b[2] * a[3] + b[14] * a[4] + b[0] * a[8] + b[10] * a[9] - b[9] * a[10] + b[4] * a[14];
  res[9] = b[9] * a[0] - b[4] * a[2] + b[14] * a[3] + b[2] * a[4] - b[10] * a[8] + b[0] * a[9] + b[8] * a[10] + b[3] * a[14];
  res[10] = b[10] * a[0] + b[14] * a[2] + b[4] * a[3] - b[3] * a[4] + b[9] * a[8] - b[8] * a[9] + b[0] * a[10] + b[2] * a[14];
  res[11] = b[11] * a[0] - b[8] * a[1] + b[6] * a[2] - b[5] * a[3] + b[15] * a[4] - b[3] * a[5] + b[2] * a[6] - b[14] * a[7] - b[1] * a[8] + b[13] * a[9] - b[12] * a[10] + b[0] * a[11] + b[10] * a[12] - b[9] * a[13] + b[7] * a[14] - b[4] * a[15];
  res[12] = b[12] * a[0] - b[9] * a[1] - b[7] * a[2] + b[15] * a[3] + b[5] * a[4] + b[4] * a[5] - b[14] * a[6] - b[2] * a[7] - b[13] * a[8] - b[1] * a[9] + b[11] * a[10] - b[10] * a[11] + b[0] * a[12] + b[8] * a[13] + b[6] * a[14] - b[3] * a[15];
  res[13] = b[13] * a[0] - b[10] * a[1] + b[15] * a[2] + b[7] * a[3] - b[6] * a[4] - b[14] * a[5] - b[4] * a[6] + b[3] * a[7] + b[12] * a[8] - b[11] * a[9] - b[1] * a[10] + b[9] * a[11] - b[8] * a[12] + b[0] * a[13] + b[5] * a[14] - b[2] * a[15];
  res[14] = b[14] * a[0] + b[10] * a[2] + b[9] * a[3] + b[8] * a[4] + b[4] * a[8] + b[3] * a[9] + b[2] * a[10] + b[0] * a[14];
  res[15] = b[15] * a[0] + b[14] * a[1] + b[13] * a[2] + b[12] * a[3] + b[11] * a[4] + b[10] * a[5] + b[9] * a[6] + b[8] * a[7] + b[7] * a[8] + b[6] * a[9] + b[5] * a[10] - b[4] * a[11] - b[3] * a[12] - b[2] * a[13] - b[1] * a[14] + b[0] * a[15];
  return res;
};

//***********************
// PGA3D.Wedge : res = a ^ b 
// The outer product. (MEET)
//***********************
PGA3D operator ^ (const PGA3D& a, const PGA3D& b) {
  PGA3D res;
  res[0] = b[0] * a[0];
  res[1] = b[1] * a[0] + b[0] * a[1];
  res[2] = b[2] * a[0] + b[0] * a[2];
  res[3] = b[3] * a[0] + b[0] * a[3];
  res[4] = b[4] * a[0] + b[0] * a[4];
  res[5] = b[5] * a[0] + b[2] * a[1] - b[1] * a[2] + b[0] * a[5];
  res[6] = b[6] * a[0] + b[3] * a[1] - b[1] * a[3] + b[0] * a[6];
  res[7] = b[7] * a[0] + b[4] * a[1] - b[1] * a[4] + b[0] * a[7];
  res[8] = b[8] * a[0] + b[3] * a[2] - b[2] * a[3] + b[0] * a[8];
  res[9] = b[9] * a[0] - b[4] * a[2] + b[2] * a[4] + b[0] * a[9];
  res[10] = b[10] * a[0] + b[4] * a[3] - b[3] * a[4] + b[0] * a[10];
  res[11] = b[11] * a[0] - b[8] * a[1] + b[6] * a[2] - b[5] * a[3] - b[3] * a[5] + b[2] * a[6] - b[1] * a[8] + b[0] * a[11];
  res[12] = b[12] * a[0] - b[9] * a[1] - b[7] * a[2] + b[5] * a[4] + b[4] * a[5] - b[2] * a[7] - b[1] * a[9] + b[0] * a[12];
  res[13] = b[13] * a[0] - b[10] * a[1] + b[7] * a[3] - b[6] * a[4] - b[4] * a[6] + b[3] * a[7] - b[1] * a[10] + b[0] * a[13];
  res[14] = b[14] * a[0] + b[10] * a[2] + b[9] * a[3] + b[8] * a[4] + b[4] * a[8] + b[3] * a[9] + b[2] * a[10] + b[0] * a[14];
  res[15] = b[15] * a[0] + b[14] * a[1] + b[13] * a[2] + b[12] * a[3] + b[11] * a[4] + b[10] * a[5] + b[9] * a[6] + b[8] * a[7] + b[7] * a[8] + b[6] * a[9] + b[5] * a[10] - b[4] * a[11] - b[3] * a[12] - b[2] * a[13] - b[1] * a[14] + b[0] * a[15];
  return res;
};

//***********************
// PGA3D.Vee : res = a & b 
// The regressive product. (JOIN)
//***********************
PGA3D operator & (const PGA3D& a, const PGA3D& b) {
  PGA3D res;
  res[15] = 1 * (a[15] * b[15]);
  res[14] = -1 * (a[14] * -1 * b[15] + a[15] * b[14] * -1);
  res[13] = -1 * (a[13] * -1 * b[15] + a[15] * b[13] * -1);
  res[12] = -1 * (a[12] * -1 * b[15] + a[15] * b[12] * -1);
  res[11] = -1 * (a[11] * -1 * b[15] + a[15] * b[11] * -1);
  res[10] = 1 * (a[10] * b[15] + a[13] * -1 * b[14] * -1 - a[14] * -1 * b[13] * -1 + a[15] * b[10]);
  res[9] = 1 * (a[9] * b[15] + a[12] * -1 * b[14] * -1 - a[14] * -1 * b[12] * -1 + a[15] * b[9]);
  res[8] = 1 * (a[8] * b[15] + a[11] * -1 * b[14] * -1 - a[14] * -1 * b[11] * -1 + a[15] * b[8]);
  res[7] = 1 * (a[7] * b[15] + a[12] * -1 * b[13] * -1 - a[13] * -1 * b[12] * -1 + a[15] * b[7]);
  res[6] = 1 * (a[6] * b[15] - a[11] * -1 * b[13] * -1 + a[13] * -1 * b[11] * -1 + a[15] * b[6]);
  res[5] = 1 * (a[5] * b[15] + a[11] * -1 * b[12] * -1 - a[12] * -1 * b[11] * -1 + a[15] * b[5]);
  res[4] = 1 * (a[4] * b[15] - a[7] * b[14] * -1 + a[9] * b[13] * -1 - a[10] * b[12] * -1 - a[12] * -1 * b[10] + a[13] * -1 * b[9] - a[14] * -1 * b[7] + a[15] * b[4]);
  res[3] = 1 * (a[3] * b[15] - a[6] * b[14] * -1 - a[8] * b[13] * -1 + a[10] * b[11] * -1 + a[11] * -1 * b[10] - a[13] * -1 * b[8] - a[14] * -1 * b[6] + a[15] * b[3]);
  res[2] = 1 * (a[2] * b[15] - a[5] * b[14] * -1 + a[8] * b[12] * -1 - a[9] * b[11] * -1 - a[11] * -1 * b[9] + a[12] * -1 * b[8] - a[14] * -1 * b[5] + a[15] * b[2]);
  res[1] = 1 * (a[1] * b[15] + a[5] * b[13] * -1 + a[6] * b[12] * -1 + a[7] * b[11] * -1 + a[11] * -1 * b[7] + a[12] * -1 * b[6] + a[13] * -1 * b[5] + a[15] * b[1]);
  res[0] = 1 * (a[0] * b[15] + a[1] * b[14] * -1 + a[2] * b[13] * -1 + a[3] * b[12] * -1 + a[4] * b[11] * -1 + a[5] * b[10] + a[6] * b[9] + a[7] * b[8] + a[8] * b[7] + a[9] * b[6] + a[10] * b[5] - a[11] * -1 * b[4] - a[12] * -1 * b[3] - a[13] * -1 * b[2] - a[14] * -1 * b[1] + a[15] * b[0]);
  return res;
};

//***********************
// PGA3D.Dot : res = a | b 
// The inner product.
//***********************
PGA3D operator | (const PGA3D& a, const PGA3D& b) {
  PGA3D res;
  res[0] = b[0] * a[0] + b[2] * a[2] + b[3] * a[3] + b[4] * a[4] - b[8] * a[8] - b[9] * a[9] - b[10] * a[10] - b[14] * a[14];
  res[1] = b[1] * a[0] + b[0] * a[1] - b[5] * a[2] - b[6] * a[3] - b[7] * a[4] + b[2] * a[5] + b[3] * a[6] + b[4] * a[7] + b[11] * a[8] + b[12] * a[9] + b[13] * a[10] + b[8] * a[11] + b[9] * a[12] + b[10] * a[13] + b[15] * a[14] - b[14] * a[15];
  res[2] = b[2] * a[0] + b[0] * a[2] - b[8] * a[3] + b[9] * a[4] + b[3] * a[8] - b[4] * a[9] - b[14] * a[10] - b[10] * a[14];
  res[3] = b[3] * a[0] + b[8] * a[2] + b[0] * a[3] - b[10] * a[4] - b[2] * a[8] - b[14] * a[9] + b[4] * a[10] - b[9] * a[14];
  res[4] = b[4] * a[0] - b[9] * a[2] + b[10] * a[3] + b[0] * a[4] - b[14] * a[8] + b[2] * a[9] - b[3] * a[10] - b[8] * a[14];
  res[5] = b[5] * a[0] - b[11] * a[3] + b[12] * a[4] + b[0] * a[5] - b[15] * a[10] - b[3] * a[11] + b[4] * a[12] - b[10] * a[15];
  res[6] = b[6] * a[0] + b[11] * a[2] - b[13] * a[4] + b[0] * a[6] - b[15] * a[9] + b[2] * a[11] - b[4] * a[13] - b[9] * a[15];
  res[7] = b[7] * a[0] - b[12] * a[2] + b[13] * a[3] + b[0] * a[7] - b[15] * a[8] - b[2] * a[12] + b[3] * a[13] - b[8] * a[15];
  res[8] = b[8] * a[0] + b[14] * a[4] + b[0] * a[8] + b[4] * a[14];
  res[9] = b[9] * a[0] + b[14] * a[3] + b[0] * a[9] + b[3] * a[14];
  res[10] = b[10] * a[0] + b[14] * a[2] + b[0] * a[10] + b[2] * a[14];
  res[11] = b[11] * a[0] + b[15] * a[4] + b[0] * a[11] - b[4] * a[15];
  res[12] = b[12] * a[0] + b[15] * a[3] + b[0] * a[12] - b[3] * a[15];
  res[13] = b[13] * a[0] + b[15] * a[2] + b[0] * a[13] - b[2] * a[15];
  res[14] = b[14] * a[0] + b[0] * a[14];
  res[15] = b[15] * a[0] + b[0] * a[15];
  return res;
};

//***********************
// PGA3D.Add : res = a + b 
// Multivector addition
//***********************
PGA3D operator + (const PGA3D& a, const PGA3D& b) {
  PGA3D res;
  res[0] = a[0] + b[0];
  res[1] = a[1] + b[1];
  res[2] = a[2] + b[2];
  res[3] = a[3] + b[3];
  res[4] = a[4] + b[4];
  res[5] = a[5] + b[5];
  res[6] = a[6] + b[6];
  res[7] = a[7] + b[7];
  res[8] = a[8] + b[8];
  res[9] = a[9] + b[9];
  res[10] = a[10] + b[10];
  res[11] = a[11] + b[11];
  res[12] = a[12] + b[12];
  res[13] = a[13] + b[13];
  res[14] = a[14] + b[14];
  res[15] = a[15] + b[15];
  return res;
};

//***********************
// PGA3D.Sub : res = a - b 
// Multivector subtraction
//***********************
PGA3D operator - (const PGA3D& a, const PGA3D& b) {
  PGA3D res;
  res[0] = a[0] - b[0];
  res[1] = a[1] - b[1];
  res[2] = a[2] - b[2];
  res[3] = a[3] - b[3];
  res[4] = a[4] - b[4];
  res[5] = a[5] - b[5];
  res[6] = a[6] - b[6];
  res[7] = a[7] - b[7];
  res[8] = a[8] - b[8];
  res[9] = a[9] - b[9];
  res[10] = a[10] - b[10];
  res[11] = a[11] - b[11];
  res[12] = a[12] - b[12];
  res[13] = a[13] - b[13];
  res[14] = a[14] - b[14];
  res[15] = a[15] - b[15];
  return res;
};

//***********************
// PGA3D.smul : res = a * b 
// scalar/multivector multiplication
//***********************
PGA3D operator * (const float& a, const PGA3D& b) {
  PGA3D res;
  res[0] = a * b[0];
  res[1] = a * b[1];
  res[2] = a * b[2];
  res[3] = a * b[3];
  res[4] = a * b[4];
  res[5] = a * b[5];
  res[6] = a * b[6];
  res[7] = a * b[7];
  res[8] = a * b[8];
  res[9] = a * b[9];
  res[10] = a * b[10];
  res[11] = a * b[11];
  res[12] = a * b[12];
  res[13] = a * b[13];
  res[14] = a * b[14];
  res[15] = a * b[15];
  return res;
};

//***********************
// PGA3D.muls : res = a * b 
// multivector/scalar multiplication
//***********************
PGA3D operator * (const PGA3D& a, const float& b) {
  PGA3D res;
  res[0] = a[0] * b;
  res[1] = a[1] * b;
  res[2] = a[2] * b;
  res[3] = a[3] * b;
  res[4] = a[4] * b;
  res[5] = a[5] * b;
  res[6] = a[6] * b;
  res[7] = a[7] * b;
  res[8] = a[8] * b;
  res[9] = a[9] * b;
  res[10] = a[10] * b;
  res[11] = a[11] * b;
  res[12] = a[12] * b;
  res[13] = a[13] * b;
  res[14] = a[14] * b;
  res[15] = a[15] * b;
  return res;
};

//***********************
// PGA3D.sadd : res = a + b 
// scalar/multivector addition
//***********************
PGA3D operator + (const float& a, const PGA3D& b) {
  PGA3D res;
  res[0] = a + b[0];
  res[1] = b[1];
  res[2] = b[2];
  res[3] = b[3];
  res[4] = b[4];
  res[5] = b[5];
  res[6] = b[6];
  res[7] = b[7];
  res[8] = b[8];
  res[9] = b[9];
  res[10] = b[10];
  res[11] = b[11];
  res[12] = b[12];
  res[13] = b[13];
  res[14] = b[14];
  res[15] = b[15];
  return res;
};

//***********************
// PGA3D.adds : res = a + b 
// multivector/scalar addition
//***********************
PGA3D operator + (const PGA3D& a, const float& b) {
  PGA3D res;
  res[0] = a[0] + b;
  res[1] = a[1];
  res[2] = a[2];
  res[3] = a[3];
  res[4] = a[4];
  res[5] = a[5];
  res[6] = a[6];
  res[7] = a[7];
  res[8] = a[8];
  res[9] = a[9];
  res[10] = a[10];
  res[11] = a[11];
  res[12] = a[12];
  res[13] = a[13];
  res[14] = a[14];
  res[15] = a[15];
  return res;
};

//***********************
// PGA3D.ssub : res = a - b 
// scalar/multivector subtraction
//***********************
PGA3D operator - (const float& a, const PGA3D& b) {
  PGA3D res;
  res[0] = a - b[0];
  res[1] = -b[1];
  res[2] = -b[2];
  res[3] = -b[3];
  res[4] = -b[4];
  res[5] = -b[5];
  res[6] = -b[6];
  res[7] = -b[7];
  res[8] = -b[8];
  res[9] = -b[9];
  res[10] = -b[10];
  res[11] = -b[11];
  res[12] = -b[12];
  res[13] = -b[13];
  res[14] = -b[14];
  res[15] = -b[15];
  return res;
};

//***********************
// PGA3D.subs : res = a - b 
// multivector/scalar subtraction
//***********************
PGA3D operator - (const PGA3D& a, const float& b) {
  PGA3D res;
  res[0] = a[0] - b;
  res[1] = a[1];
  res[2] = a[2];
  res[3] = a[3];
  res[4] = a[4];
  res[5] = a[5];
  res[6] = a[6];
  res[7] = a[7];
  res[8] = a[8];
  res[9] = a[9];
  res[10] = a[10];
  res[11] = a[11];
  res[12] = a[12];
  res[13] = a[13];
  res[14] = a[14];
  res[15] = a[15];
  return res;
};


float PGA3D::norm() { return sqrt(std::abs(((*this) * Conjugate()).mvec[0])); }
float PGA3D::inorm() { return (!(*this)).norm(); }
PGA3D PGA3D::normalized() { return (*this) * (1 / norm()); }

nvmath::mat4f PGA3D::as_mat4x4() {
  const float* b = &(this->mvec[4]);
  const float* c = &(this->mvec[8]);

  // float o[] = {
  //   POW(b[0]) + POW(b[2]) - POW(b[3]) - POW(b[2]),
  //   2.f * (b[1] * b[2] - b[3] * b[0]),
  //   2.f * (b[2] * b[0] + b[1] * b[3]),
  //   0.f,

  //   2.f * (b[0] * b[3] + b[2] * b[1]),
  //   -POW(b[1]) - POW(b[3]) + POW(b[0]) + POW(b[2]),
  //   2.f * (b[2] * b[3] - b[0] * b[1]),
  //   0.f,

  //   2.f * (-b[0] * b[2] + b[1] * b[3]),
  //   2.f * (b[1] * b[0] + b[2] * b[3]),
  //   (-POW(b[2]) + POW(b[0]) + POW(b[3]) - POW(b[1])),
  //   0,

  //   2.f * (b[2] * c[3] - b[0] * c[1] - b[3] * c[2] - b[1] * c[0]),
  //   2.f * (b[3] * c[1] - b[1] * c[3] - b[0] * c[2] - b[2] * c[0]),
  //   2.f * (b[1] * c[2] - b[2] * c[1] - b[0] * c[3] - b[3] * c[0]),
  //   POW(b[0]) + POW(b[1]) + POW(b[2]) + POW(b[3]),
  // };


  float o[] = {
    POW(b[0]) + POW(b[2]) - POW(b[3]) - POW(b[2]),
    2.f * (b[1] * b[2] - b[3] * b[0]),
    2.f * (b[2] * b[0] + b[1] * b[3]),
    0.f,

    2.f * (b[0] * b[3] + b[2] * b[1]),
    -POW(b[1]) - POW(b[3]) + POW(b[0]) + POW(b[2]),
    2.f * (b[2] * b[3] - b[0] * b[1]),
    0.f,

    2.f * (-b[0] * b[2] + b[1] * b[3]),
    2.f * (b[1] * b[0] + b[2] * b[3]),
    (-POW(b[2]) + POW(b[0]) + POW(b[3]) - POW(b[1])),
    0,

    2.f * (b[2] * c[3] - b[0] * c[1] - b[3] * c[2] - b[1] * c[0]) ,
    2.f * (b[3] * c[1] - b[1] * c[3] - b[0] * c[2] - b[2] * c[0]),
    2.f * (b[1] * c[2] - b[2] * c[1] - b[0] * c[3] - b[3] * c[0]),
    POW(b[0]) + POW(b[1]) + POW(b[2]) + POW(b[3]),
  };
  return nvmath::mat4f(o);
}


// A rotor (Euclidean line) and translator (Ideal line)
PGA3D rotor(float angle, PGA3D line) {
  auto l
  auto a = cos(angle / 2.0f) + (sin(angle / 2.0f) * line.normalized());
  return a;
  
}
PGA3D translator(float dist, PGA3D line) { return 1.0f + dist / 2.0f * line; }

// A plane is defined using its homogenous equation ax + by + cz + d = 0
PGA3D plane(float a, float b, float c, float d) { return a * e1 + b * e2 + c * e3 + d * e0; }


// A point is just a homogeneous point, euclidean coordinates plus the origin
PGA3D point(float x, float y, float z) { return e123 + x * e032 + y * e013 + z * e021; }