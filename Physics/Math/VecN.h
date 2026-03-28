/**
 * @file VecN.h
 * @brief N-dimensional vector for generic math and solver calculations.
 *
 * Essential for the Constraint Solver. Handles dynamic arrays of floats used
 * in Jacobian matrices and Velocity/Mass vectors.
 */

#ifndef VECN_H
#define VECN_H

struct VecN {
    int N;
    float* data;

    VecN();
    explicit VecN(int N);
    VecN(const VecN& v);
    VecN(VecN&& v) noexcept;
    ~VecN();

    void Zero();
    int Size() const;

    float Dot(const VecN& v) const;

    VecN& operator=(const VecN& v);
    VecN& operator=(VecN&& v) noexcept; 
    VecN  operator+(const VecN& v) const;
    VecN  operator-(const VecN& v) const;
    VecN  operator*(float n) const;
    VecN& operator+=(const VecN& v);
    VecN& operator-=(const VecN& v);
    VecN& operator*=(float n);

    float  operator[](int index) const;
    float& operator[](int index);
};

/** @brief Scalar multiplication (n * v). */
inline VecN operator*(float n, const VecN& v) {
    return v * n;
}

#endif