/**
 * @file MatMN.h
 * @brief M×N matrix for constraint solving and generalized mathematics.
 *
 * Represents a matrix with M rows and N columns. Used heavily in the physics
 * solver to compute Jacobians and solve linear systems via Gauss-Seidel.
 */

#ifndef MATMN_H
#define MATMN_H

#include "VecN.h"

struct MatMN {
    int M;      // Row count
    int N;      // Column count
    VecN* rows; // Array of M row vectors, each of size N

    // --- Lifecycle ---
    MatMN();
    MatMN(int M, int N);

    MatMN(const MatMN& m);
    MatMN(MatMN&& m) noexcept;

    ~MatMN();

    void Zero();
    MatMN Transpose() const;

    MatMN& operator=(const MatMN& m);
    MatMN& operator=(MatMN&& m) noexcept;
    VecN  operator*(const VecN& v)  const; // Matrix-vector product
    MatMN operator*(const MatMN& m) const; // Matrix-matrix product

    /**
     * @brief Solves Ax = b using the iterative Gauss-Seidel method.
     */
    static VecN SolveGaussSeidel(const MatMN& A, const VecN& b);
};

#endif