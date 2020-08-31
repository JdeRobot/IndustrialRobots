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

#ifndef INCLUDE_MATH_MATRIXX_H_
#define INCLUDE_MATH_MATRIXX_H_

#include "Matrix.h"

#include <vector>
#include <cmath>
#include <ostream>
#include <cstdint>


template<typename T>
class MatrixX {
public:
	MatrixX() {}

	MatrixX(size_t rows, size_t cols) {
		resize(rows, cols);
	}

	template<typename S>
	MatrixX(const MatrixX<S>& B) {
		resize(B.rows(), B.cols());
		for(size_t i = 0; i < size(); ++i) {
			data_[i] = B[i];
		}
	}

	~MatrixX() {
		clear();
	}

	void resize(size_t rows, size_t cols) {
		if(rows > 0xFFFFFFFF || cols > 0xFFFFFFFF) {
			throw std::invalid_argument("rows or cols out of bounds");
		}
		const size_t new_size = rows * cols;
		if(new_size != size()) {
			if(data_) {
				delete [] data_;
				data_ = 0;
			}
			if(new_size > 0) {
				data_ = new T[rows * cols];
			}
		}
		rows_ = uint32_t(rows);
		cols_ = uint32_t(cols);
	}

	void clear() {
		delete [] data_;
		rows_ = 0;
		cols_ = 0;
	}

	size_t rows() const {
		return rows_;
	}

	size_t cols() const {
		return cols_;
	}

	size_t size() const {
		return rows_ * cols_;
	}

	T* get_data() {
		return data_;
	}

	const T* get_data() const {
		return data_;
	}

	T& operator()(size_t i, size_t j) {
		return data_[j * rows_ + i];
	}

	const T& operator()(size_t i, size_t j) const {
		return data_[j * rows_ + i];
	}

	T& operator[](size_t i) {
		return data_[i];
	}

	const T& operator[](size_t i) const {
		return data_[i];
	}

	MatrixX<T> get(size_t N, size_t M, size_t i_0 = 0, size_t j_0 = 0) const {
		MatrixX<T> res(N, M);
		for(size_t j = 0; j < M; ++j) {
			for(size_t i = 0; i < N; ++i) {
				res(i, j) = (*this)(i + i_0, j + j_0);
			}
		}
		return res;
	}

	MatrixX<T> transpose() const {
		MatrixX<T> res(cols_, rows_);
		for(size_t j = 0; j < cols_; ++j) {
			for(size_t i = 0; i < rows_; ++i) {
				res(j, i) = (*this)(i, j);
			}
		}
		return res;
	}

	template<typename S>
	MatrixX<T>& operator=(const MatrixX<S>& B) {
		resize(B.rows(), B.cols());
		for(size_t i = 0; i < size(); ++i) {
			(*this)[i] = B[i];
		}
		return *this;
	}

	MatrixX<T> operator*(const MatrixX& B) const {
		if(B.rows() != cols()) {
			throw std::logic_error("invalid matrix multiplication");
		}
		MatrixX<T> C(rows(), B.cols());
		C.set_zero();
		for(size_t i = 0; i < rows(); ++i) {
			for(size_t j = 0; j < B.cols(); ++j) {
				for(size_t k = 0; k < cols(); ++k) {
					C(i, j) += (*this)(i, k) * B(k, j);
				}
			}
		}
		return C;
	}

	MatrixX<T>& operator*=(T b) {
		for(size_t i = 0; i < size(); ++i) {
			(*this)[i] *= b;
		}
		return *this;
	}

	MatrixX<T> operator*(T b) const {
		MatrixX<T> C = *this;
		C *= b;
		return C;
	}

	MatrixX<T>& operator+=(const MatrixX& B) {
		if(rows_ != B.rows() || cols_ != B.cols()) {
			throw std::logic_error("invalid matrix operation");
		}
		for(size_t i = 0; i < size(); ++i) {
			(*this)[i] += B[i];
		}
		return *this;
	}

	MatrixX<T> operator+(const MatrixX& B) const {
		if(rows_ != B.rows() || cols_ != B.cols()) {
			throw std::logic_error("invalid matrix operation");
		}
		MatrixX<T> C = *this;
		C += B;
		return C;
	}

	MatrixX<T>& operator-=(const MatrixX& B) {
		if(rows_ != B.rows() || cols_ != B.cols()) {
			throw std::logic_error("invalid matrix operation");
		}
		for(size_t i = 0; i < size(); ++i) {
			(*this)[i] -= B[i];
		}
		return *this;
	}

	MatrixX<T> operator-(const MatrixX& B) const {
		if(rows_ != B.rows() || cols_ != B.cols()) {
			throw std::logic_error("invalid matrix operation");
		}
		MatrixX<T> C = *this;
		C -= B;
		return C;
	}

	bool operator==(const MatrixX& B) const {
		if(rows_ == B.rows() && cols_ == B.cols()) {
			for(size_t i = 0; i < size(); ++i) {
				if(!((*this)[i] == B[i])) {
					return false;
				}
			}
			return true;
		}
		return false;
	}

	bool operator!=(const MatrixX& B) const {
		return !(*this == B);
	}

	T square_norm() const {
		T res = 0;
		for(size_t i = 0; i < size(); ++i) {
			res += (*this)[i] * (*this)[i];
		}
		return res;
	}

	T norm() const {
		return std::sqrt(square_norm());
	}

	T sum() const {
		T res = 0;
		for(size_t i = 0; i < size(); ++i) {
			res += (*this)[i];
		}
		return res;
	}

	void fill(T value) {
		for(size_t i = 0; i < size(); ++i) {
			(*this)[i] = value;
		}
	}

	void set_zero() {
		if(data_) {
			fill(0);
		}
	}

	std::vector<T> to_vector() const {
		std::vector<T> vec;
		vec.resize(size());
		for(size_t i = 0; i < size(); ++i) {
			vec[i] = (*this)[i];
		}
		return vec;
	}

	std::ostream& print(std::ostream& out, const std::string& name) const {
		out << name << " = [" << std::endl;
		for(size_t i = 0; i < rows_; ++i) {
			if(i > 0) {
				out << "," << std::endl;
			}
			out << "[";
			for(size_t j = 0; j < cols_; ++j) {
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

private:
	T* data_ = 0;
	uint32_t rows_ = 0;
	uint32_t cols_ = 0;

};


template<typename T, size_t Rows, size_t Cols>
template<typename S>
Matrix<T, Rows, Cols>::Matrix(const MatrixX<S>& mat) {
	if(mat.rows() != Rows || mat.cols() != Cols) {
		throw std::logic_error("matrix dimension mismatch");
	}
	for(size_t i = 0; i < size(); ++i) {
		data[i] = mat[i];
	}
}


#endif /* INCLUDE_MATH_MATRIXX_H_ */
