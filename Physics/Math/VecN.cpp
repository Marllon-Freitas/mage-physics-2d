#include "VecN.h"
#include <cassert>
#include <algorithm>

VecN::VecN() : N(0), data(nullptr) {}

VecN::VecN(int N) : N(N) {
    data = new float[N]();
}

VecN::VecN(const VecN& v) : N(v.N) {
    data = new float[N];
    std::copy(v.data, v.data + N, data);
}

VecN::VecN(VecN&& v) noexcept : N(v.N), data(v.data) {
    v.N = 0;
    v.data = nullptr;
}

VecN::~VecN() {
    delete[] data;
}

int VecN::Size() const {
    return N;
}

void VecN::Zero() {
    std::fill(data, data + N, 0.0f);
}

float VecN::Dot(const VecN& v) const {
    assert(N == v.N && "VecN::Dot - incompatible sizes");
    float sum = 0.0f;
    for (int i = 0; i < N; ++i) {
        sum += data[i] * v.data[i];
    }
    return sum;
}

// --- Assignment ---
VecN& VecN::operator=(const VecN& v) {
    if (this == &v) return *this;

    if (N != v.N) {
        delete[] data;
        N = v.N;
        data = new float[N];
    }
    std::copy(v.data, v.data + N, data);
    return *this;
}

VecN& VecN::operator=(VecN&& v) noexcept {
    if (this == &v) return *this;

    delete[] data;
    N = v.N;
    data = v.data;

    v.N = 0;
    v.data = nullptr;
    return *this;
}

// --- Arithmetic ---
VecN VecN::operator+(const VecN& v) const {
    assert(N == v.N && "VecN::operator+ - incompatible sizes");
    VecN result(N);
    for (int i = 0; i < N; ++i) result.data[i] = data[i] + v.data[i];
    return result;
}

VecN VecN::operator-(const VecN& v) const {
    assert(N == v.N && "VecN::operator- - incompatible sizes");
    VecN result(N);
    for (int i = 0; i < N; ++i) result.data[i] = data[i] - v.data[i];
    return result;
}

VecN VecN::operator*(float n) const {
    VecN result(N);
    for (int i = 0; i < N; ++i) result.data[i] = data[i] * n;
    return result;
}

VecN& VecN::operator+=(const VecN& v) {
    assert(N == v.N && "VecN::operator+= - incompatible sizes");
    for (int i = 0; i < N; ++i) data[i] += v.data[i];
    return *this;
}

VecN& VecN::operator-=(const VecN& v) {
    assert(N == v.N && "VecN::operator-= - incompatible sizes");
    for (int i = 0; i < N; ++i) data[i] -= v.data[i];
    return *this;
}

VecN& VecN::operator*=(float n) {
    for (int i = 0; i < N; ++i) data[i] *= n;
    return *this;
}

// --- Access ---
float VecN::operator[](int index) const {
    assert(index >= 0 && index < N && "VecN index out of bounds");
    return data[index];
}

float& VecN::operator[](int index) {
    assert(index >= 0 && index < N && "VecN index out of bounds");
    return data[index];
}