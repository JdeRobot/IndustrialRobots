/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Neobotix GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Neobotix nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef INCLUDE_MATH_MATRIX_H_
#define INCLUDE_MATH_MATRIX_H_

#include <cmath>
#include <cstdint>
#include <ostream>
#include <array>
#include <stdexcept>
#include <initializer_list>


template<typename T>
class MatrixX;

struct NO_INIT {};

template<typename T, size_t Rows, size_t Cols>
class Matrix;

template<typename T, size_t Rows, size_t Cols>
Matrix<T, Rows, Cols> inverse(const Matrix<T, Rows, Cols>& mat);

template<typename T>
Matrix<T, 1, 1> inverse(const Matrix<T, 1, 1>& m);

template<typename T>
Matrix<T, 2, 2> inverse(const Matrix<T, 2, 2>& m);

template<typename T>
Matrix<T, 3, 3> inverse(const Matrix<T, 3, 3>& m);

template<typename T>
Matrix<T, 4, 4> inverse(const Matrix<T, 4, 4>& m);


template<typename T, size_t Rows, size_t Cols>
class Matrix {
public:
	typedef T Scalar;

	/*
	 * Default constructor initializes with all zeros.
	 */
	Matrix() : data({}) {}

	/*
	 * Special constructor, does not initialize data.
	 */
	Matrix(NO_INIT) {}

	/*
	 * Generic copy constructor.
	 */
	template<typename S>
	Matrix(const Matrix<S, Rows, Cols>& mat) {
		for(size_t i = 0; i < size(); ++i) {
			data[i] = mat.data[i];
		}
	}

	/*
	 * Generic copy constructor for a MatrixX.
	 */
	template<typename S>
	Matrix(const MatrixX<S>& mat);

	/*
	 * Initialize with column major list (ie. human readable form).
	 */
	Matrix(const std::initializer_list<T>& list) {
		if(list.size() != Rows * Cols) {
			throw std::logic_error("list.size() != Rows * Cols");
		}
		size_t i = 0;
		for(const T& v : list) {
			(*this)(i / Cols, i % Cols) = v;
			i++;
		}
	}

	/*
	 * Returns identity matrix.
	 */
	static Matrix Identity() {
		if(Rows != Cols) {
			throw std::logic_error("Rows != Cols");
		}
		Matrix res;
		for(size_t i = 0; i < Rows; ++i) {
			res(i, i) = 1;
		}
		return res;
	}

	T* get_data() {
		return &data[0];
	}

	const T* get_data() const {
		return &data[0];
	}

	T& operator()(size_t i, size_t j) {
		return data[j * Rows + i];
	}

	const T& operator()(size_t i, size_t j) const {
		return data[j * Rows + i];
	}

	T& operator[](size_t i) {
		return data[i];
	}

	const T& operator[](size_t i) const {
		return data[i];
	}

	Matrix<T, Cols, Rows> transpose() const {
		Matrix<T, Cols, Rows> res;
		for(size_t j = 0; j < Cols; ++j) {
			for(size_t i = 0; i < Rows; ++i) {
				res(j, i) = (*this)(i, j);
			}
		}
		return res;
	}

	T squared_norm() const {
		T sum = 0;
		for(T v : data) {
			sum += v*v;
		}
		return sum;
	}

	T norm() const {
		return std::sqrt(squared_norm());
	}

	size_t rows() const {
		return Rows;
	}

	size_t cols() const {
		return Cols;
	}

	size_t size() const {
		return Rows * Cols;
	}

	Matrix inverse() const {
		if(Rows != Cols) {
			throw std::logic_error("Rows != Cols");
		}
		return ::inverse<T>(*this);
	}

	template<size_t N, size_t M = 1>
	Matrix<T, N, M> get(size_t i_0 = 0, size_t j_0 = 0) const {
		Matrix<T, N, M> res;
		for(size_t j = 0; j < M; ++j) {
			for(size_t i = 0; i < N; ++i) {
				res(i, j) = (*this)(i + i_0, j + j_0);
			}
		}
		return res;
	}

	Matrix<T, Rows+1, 1> extend() const {
		if(Cols != 1) {
			throw std::logic_error("Cols != 1");
		}
		Matrix<T, Rows+1, 1> res;
		for(size_t i = 0; i < Rows; ++i) {
			res[i] = data[i];
		}
		res[Rows] = 1;
		return res;
	}

	Matrix<T, Rows-1, 1> project() const {
		if(Cols != 1) {
			throw std::logic_error("Cols != 1");
		}
		auto res = get<Rows-1>();
		if(data[Rows-1] != 1) {
			res *= T(1) / data[Rows-1];
		}
		return res;
	}

	void normalize() {
		(*this) *= (T(1) / norm());
	}

	Matrix normalized() const {
		return (*this) * (T(1) / norm());
	}

	T dot(const Matrix& B) const {
		T res = 0;
		for(size_t i = 0; i < size(); ++i) {
			res += data[i] * B[i];
		}
		return res;
	}

	template<size_t N>
	Matrix<T, Rows, N> operator*(const Matrix<T, Cols, N>& B) const {
		Matrix<T, Rows, N> C;
		for(size_t i = 0; i < Rows; ++i) {
			for(size_t j = 0; j < N; ++j) {
				for(size_t k = 0; k < Cols; ++k) {
					C(i, j) += (*this)(i, k) * B(k, j);
				}
			}
		}
		return C;
	}

	template<size_t N>
	Matrix& operator*=(const Matrix<T, Cols, N>& B) {
		*this = *this * B;
		return *this;
	}

	Matrix& operator*=(const T& value) {
		for(size_t i = 0; i < Rows * Cols; ++i) {
			(*this)[i] *= value;
		}
		return *this;
	}

	Matrix operator*(const T& value) const {
		Matrix C = *this;
		C *= value;
		return C;
	}

	Matrix& operator/=(const T& value) {
		for(size_t i = 0; i < Rows * Cols; ++i) {
			(*this)[i] /= value;
		}
		return *this;
	}

	Matrix operator/(const T& value) const {
		Matrix C = *this;
		C /= value;
		return C;
	}

	Matrix& operator+=(const Matrix& B) {
		for(size_t i = 0; i < Rows * Cols; ++i) {
			(*this)[i] += B[i];
		}
		return *this;
	}

	Matrix operator+(const Matrix& B) const {
		Matrix C = *this;
		C += B;
		return C;
	}

	Matrix& operator-=(const Matrix& B) {
		for(size_t i = 0; i < Rows * Cols; ++i) {
			(*this)[i] -= B[i];
		}
		return *this;
	}

	Matrix operator-(const Matrix& B) const {
		Matrix C = *this;
		C -= B;
		return C;
	}

	bool operator==(const Matrix& B) const {
		return data == B.data;
	}

	bool operator!=(const Matrix& B) const {
		return data != B.data;
	}

	std::ostream& print(std::ostream& out, const std::string& name) const {
		out << name << " = [" << std::endl;
		for(size_t i = 0; i < Rows; ++i) {
			if(i > 0) {
				out << "," << std::endl;
			}
			out << "[";
			for(size_t j = 0; j < Cols; ++j) {
				if(j > 0) {
					out << ", ";
				}
				out << (*this)(i, j);
			}
			out << "]";
		}
		out << "]" << std::endl;
		return out;
	}

	std::array<T, Rows * Cols> data;

};


template<typename T, size_t Rows, size_t Cols>
Matrix<T, Rows, Cols> inverse(const Matrix<T, Rows, Cols>& mat) {
	throw std::logic_error("not implemented");
}

template<typename T>
Matrix<T, 1, 1> inverse(const Matrix<T, 1, 1>& m) {
	return m * (T(1) / m[0]);
}

template<typename T>
Matrix<T, 2, 2> inverse(const Matrix<T, 2, 2>& m) {
	const T det = (m(0,0) * m(1, 1)) - (m(0, 1) * m(1, 0));
	if(det == 0) {
		throw std::runtime_error("inverse(): determinant = 0");
	}
	Matrix<T, 2, 2> tmp;
	tmp(0, 0) = m(1, 1);
	tmp(1, 0) = -m(1, 0);
	tmp(0, 1) = -m(0, 1);
	tmp(1, 1) = m(0, 0);
	return tmp * (T(1) / det);
}

template<typename T>
Matrix<T, 3, 3> inverse(const Matrix<T, 3, 3>& m) {
	const T det = m(0, 0) * (m(1, 1) * m(2, 2) - m(2, 1) * m(1, 2)) -
				  m(0, 1) * (m(1, 0) * m(2, 2) - m(1, 2) * m(2, 0)) +
				  m(0, 2) * (m(1, 0) * m(2, 1) - m(1, 1) * m(2, 0));
	if(det == 0) {
		throw std::runtime_error("inverse(): determinant = 0");
	}
	Matrix<T, 3, 3> tmp;
	tmp(0, 0) = m(1, 1) * m(2, 2) - m(2, 1) * m(1, 2);
	tmp(0, 1) = m(0, 2) * m(2, 1) - m(0, 1) * m(2, 2);
	tmp(0, 2) = m(0, 1) * m(1, 2) - m(0, 2) * m(1, 1);
	tmp(1, 0) = m(1, 2) * m(2, 0) - m(1, 0) * m(2, 2);
	tmp(1, 1) = m(0, 0) * m(2, 2) - m(0, 2) * m(2, 0);
	tmp(1, 2) = m(1, 0) * m(0, 2) - m(0, 0) * m(1, 2);
	tmp(2, 0) = m(1, 0) * m(2, 1) - m(2, 0) * m(1, 1);
	tmp(2, 1) = m(2, 0) * m(0, 1) - m(0, 0) * m(2, 1);
	tmp(2, 2) = m(0, 0) * m(1, 1) - m(1, 0) * m(0, 1);
	return tmp * (T(1) / det);
}

template<typename T>
Matrix<T, 4, 4> inverse(const Matrix<T, 4, 4>& m) {
	Matrix<T, 4, 4> tmp;
	tmp[0] = m[5] * m[10] * m[15] - m[5] * m[11] * m[14] - m[9] * m[6] * m[15]
			+ m[9] * m[7] * m[14] + m[13] * m[6] * m[11] - m[13] * m[7] * m[10];

	tmp[4] = -m[4] * m[10] * m[15] + m[4] * m[11] * m[14] + m[8] * m[6] * m[15]
			- m[8] * m[7] * m[14] - m[12] * m[6] * m[11] + m[12] * m[7] * m[10];

	tmp[8] = m[4] * m[9] * m[15] - m[4] * m[11] * m[13] - m[8] * m[5] * m[15]
			+ m[8] * m[7] * m[13] + m[12] * m[5] * m[11] - m[12] * m[7] * m[9];

	tmp[12] = -m[4] * m[9] * m[14] + m[4] * m[10] * m[13] + m[8] * m[5] * m[14]
			- m[8] * m[6] * m[13] - m[12] * m[5] * m[10] + m[12] * m[6] * m[9];

	tmp[1] = -m[1] * m[10] * m[15] + m[1] * m[11] * m[14] + m[9] * m[2] * m[15]
			- m[9] * m[3] * m[14] - m[13] * m[2] * m[11] + m[13] * m[3] * m[10];

	tmp[5] = m[0] * m[10] * m[15] - m[0] * m[11] * m[14] - m[8] * m[2] * m[15]
			+ m[8] * m[3] * m[14] + m[12] * m[2] * m[11] - m[12] * m[3] * m[10];

	tmp[9] = -m[0] * m[9] * m[15] + m[0] * m[11] * m[13] + m[8] * m[1] * m[15]
			- m[8] * m[3] * m[13] - m[12] * m[1] * m[11] + m[12] * m[3] * m[9];

	tmp[13] = m[0] * m[9] * m[14] - m[0] * m[10] * m[13] - m[8] * m[1] * m[14]
			+ m[8] * m[2] * m[13] + m[12] * m[1] * m[10] - m[12] * m[2] * m[9];

	tmp[2] = m[1] * m[6] * m[15] - m[1] * m[7] * m[14] - m[5] * m[2] * m[15]
			+ m[5] * m[3] * m[14] + m[13] * m[2] * m[7] - m[13] * m[3] * m[6];

	tmp[6] = -m[0] * m[6] * m[15] + m[0] * m[7] * m[14] + m[4] * m[2] * m[15]
			- m[4] * m[3] * m[14] - m[12] * m[2] * m[7] + m[12] * m[3] * m[6];

	tmp[10] = m[0] * m[5] * m[15] - m[0] * m[7] * m[13] - m[4] * m[1] * m[15]
			+ m[4] * m[3] * m[13] + m[12] * m[1] * m[7] - m[12] * m[3] * m[5];

	tmp[14] = -m[0] * m[5] * m[14] + m[0] * m[6] * m[13] + m[4] * m[1] * m[14]
			- m[4] * m[2] * m[13] - m[12] * m[1] * m[6] + m[12] * m[2] * m[5];

	tmp[3] = -m[1] * m[6] * m[11] + m[1] * m[7] * m[10] + m[5] * m[2] * m[11]
			- m[5] * m[3] * m[10] - m[9] * m[2] * m[7] + m[9] * m[3] * m[6];

	tmp[7] = m[0] * m[6] * m[11] - m[0] * m[7] * m[10] - m[4] * m[2] * m[11]
			+ m[4] * m[3] * m[10] + m[8] * m[2] * m[7] - m[8] * m[3] * m[6];

	tmp[11] = -m[0] * m[5] * m[11] + m[0] * m[7] * m[9] + m[4] * m[1] * m[11]
			- m[4] * m[3] * m[9] - m[8] * m[1] * m[7] + m[8] * m[3] * m[5];

	tmp[15] = m[0] * m[5] * m[10] - m[0] * m[6] * m[9] - m[4] * m[1] * m[10]
			+ m[4] * m[2] * m[9] + m[8] * m[1] * m[6] - m[8] * m[2] * m[5];

	const T det = m[0] * tmp[0] + m[1] * tmp[4] + m[2] * tmp[8] + m[3] * tmp[12];
	if(det == 0) {
		throw std::runtime_error("inverse(): determinant = 0");
	}
	return tmp * (T(1) / det);
}


#endif /* INCLUDE_MATH_MATRIX_H_ */
