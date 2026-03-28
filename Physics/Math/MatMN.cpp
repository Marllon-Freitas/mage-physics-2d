#include "MatMN.h"
#include "../Core/PhysicsConstants.h"

#include <cassert>
#include <cmath>

MatMN::MatMN() : M(0), N(0), rows(nullptr) {}

MatMN::MatMN(int M, int N) : M(M), N(N) {
    rows = new VecN[M];
    for (int i = 0; i < M; ++i) {
        rows[i] = VecN(N);
    }
}

MatMN::MatMN(const MatMN& m) : M(m.M), N(m.N) {
    rows = new VecN[M];
    for (int i = 0; i < M; ++i) {
        rows[i] = m.rows[i];
    }
}

MatMN::MatMN(MatMN&& m) noexcept : M(m.M), N(m.N), rows(m.rows) {
    m.M = 0;
    m.N = 0;
    m.rows = nullptr;
}

MatMN::~MatMN() {
    delete[] rows;
}

void MatMN::Zero() {
    for (int i = 0; i < M; ++i) {
        rows[i].Zero();
    }
}

MatMN MatMN::Transpose() const {
    MatMN result(N, M);
    for (int i = 0; i < M; ++i) {
        for (int j = 0; j < N; ++j) {
            result.rows[j][i] = rows[i][j];
        }
    }
    return result;
}

MatMN& MatMN::operator=(const MatMN& m) {
    if (this == &m) return *this;

    // Reallocate only if dimensions changed
    if (M != m.M || N != m.N) {
        delete[] rows;
        M = m.M;
        N = m.N;
        rows = new VecN[M];
    }

    for (int i = 0; i < M; ++i) {
        rows[i] = m.rows[i];
    }
    return *this;
}

MatMN& MatMN::operator=(MatMN&& m) noexcept {
    if (this == &m) return *this;

    delete[] rows;
    M = m.M;
    N = m.N;
    rows = m.rows;

    m.M = 0;
    m.N = 0;
    m.rows = nullptr;
    return *this;
}

VecN MatMN::operator*(const VecN& v) const {
    assert(v.N == N && "Vector size must match Matrix column count");
    VecN result(M);
    for (int i = 0; i < M; ++i) {
        result[i] = rows[i].Dot(v);
    }
    return result;
}

MatMN MatMN::operator*(const MatMN& m) const {
    assert(N == m.M && "Matrix A columns must match Matrix B rows");

    MatMN m_T = m.Transpose();
    MatMN result(M, m.N);

    for (int i = 0; i < M; ++i) {
        for (int j = 0; j < m.N; ++j) {
            result.rows[i][j] = rows[i].Dot(m_T.rows[j]);
        }
    }
    return result;
}

// --- Solver ---
VecN MatMN::SolveGaussSeidel(const MatMN& A, const VecN& b) {
    const int n = b.N;
    VecN X(n);
    X.Zero();

    // Iterative approach to find x where Ax = b
    for (int iterations = 0; iterations < n; ++iterations) {
        for (int i = 0; i < n; ++i) {
            float diag = A.rows[i][i];

            if (std::fabs(diag) > mage::physics::SOLVER_EPSILON) {
                // formula: x_i = (b_i - sum_{j!=i} a_{ij} x_j) / a_{ii}
                float dx = (b[i] - A.rows[i].Dot(X)) / diag;

                // Fast NaN (Not a Number) check
                if (dx == dx) {
                    X[i] += dx;
                }
            }
        }
    }

    return X;
}