#ifndef CUSTOM_TYPES_H
#define CUSTOM_TYPES_H

#include <vector>
#include <memory>
#include <algorithm>
#include <math.h>
#include <sstream>

namespace ct{

template< typename T, int count >
class Vec_{
public:
	enum{depth = sizeof(T)};
	T val[count];

	Vec_(){
		std::fill((char*)val, (char*)val + sizeof(val), 0);
	}
	Vec_(const Vec_<T, count >& v){
		std::copy((char*)v.val, (char*)v.val + sizeof(val), (char*)val);
	}
	/**
	 * @brief Vec_
	 * example: Vec4f v = Vec4f(Vec3f())
	 * @param v
	 */
	Vec_(const Vec_<T, count-1 >& v){
		std::copy((char*)v.val, (char*)v.val + sizeof(val), (char*)val);
		std::fill((char*)val + (count - 1) * sizeof(T), (char*)val + sizeof(T), 0);
	}
	/**
	 * @brief Vec_
	 * example: Vec3f v = Vec3f(Vec4f())
	 * @param v
	 */
	Vec_(const Vec_<T, count+1 >& v){
		std::copy((char*)v.val, (char*)v.val + sizeof(val), (char*)val);
	}

	Vec_<T, count>& operator=(const Vec_<T, count >& v){
		std::copy((char*)v.val, (char*)v.val + sizeof(val), (char*)val);
		return *this;
	}
	Vec_<T, count>& operator=(const T& v){
		for(int i = 0; i < count; i++)
			val[i] = v;
		return *this;
	}
	/**
	 * @brief Vec_
	 * example: Vec4f v = (Vec3f())
	 * @param v
	 */
	Vec_<T, count>& operator=(const Vec_<T, count-1 >& v){
		std::copy((char*)v.val, (char*)v.val + sizeof(val), (char*)val);
		std::fill((char*)val + (count - 1) * sizeof(T), (char*)val + sizeof(T), 0);
		return *this;
	}
	/**
	 * @brief Vec_
	 * example: Vec3f v = (Vec4f())
	 * @param v
	 */
	Vec_<T, count>& operator=(const Vec_<T, count+1 >& v){
		std::copy((char*)v.val, (char*)v.val + sizeof(val), (char*)val);
		return *this;
	}

	Vec_(T a0){
		for(int i = 0; i < count; i++)
			val[i] = a0;
	}
	Vec_(T a0, T a1){
		if(count < 2)
			return;
		val[0] = a0;
		val[1] = a1;
		std::fill((char*)val + 2 * sizeof(T), (char*)val + sizeof(val), 0);
	}
	Vec_(T a0, T a1, T a2){
		if(count < 3)
			return;
		val[0] = a0;
		val[1] = a1;
		val[2] = a2;
		std::fill((char*)val + 3 * sizeof(T), (char*)val + sizeof(val), 0);
	}
	Vec_(T a0, T a1, T a2, T a3){
		if(count < 4)
			return;
		val[0] = a0;
		val[1] = a1;
		val[2] = a2;
		val[3] = a3;
		std::fill((char*)val + 4 * sizeof(T), (char*)val + sizeof(val), 0);
	}
	T& operator[] (int index){
		return val[index];
	}
	const T& operator[] (int index) const{
		return val[index];
	}

	inline Vec_<T, count>& operator+=( const Vec_<T, count>& v){
		for(int i = 0; i < count; i++){
			val[i] += v[i];
		}
		return *this;
	}
	inline Vec_<T, count>& operator-=( const Vec_<T, count>& v){
		for(int i = 0; i < count; i++){
			val[i] -= v[i];
		}
		return *this;
	}
	inline Vec_<T, count>& operator*=( const Vec_<T, count>& v){
		for(int i = 0; i < count; i++){
			val[i] *= v[i];
		}
		return *this;
	}
	inline Vec_<T, count>& operator+=(T v){
		for(int i = 0; i < count; i++){
			val[i] += v;
		}
		return *this;
	}
	inline Vec_<T, count>& operator-=(T v){
		for(int i = 0; i < count; i++){
			val[i] -= v;
		}
		return *this;
	}
	inline Vec_<T, count>& operator*=(T v){
		for(int i = 0; i < count; i++){
			val[i] *= v;
		}
		return *this;
	}
	inline Vec_<T, count>& operator/=(T v){
		for(int i = 0; i < count; i++){
			val[i] /= v;
		}
		return *this;
	}
	inline bool operator== (const Vec_<T, count>& v) const{
		const T eps = 1e-12;
		double res = 0;
		for(int i = 0; i < count; i++){
			res += abs(val[i] - v.val[i]);
		}
		return res < eps;
	}
	inline bool empty() const{
		bool notempty = false;
		for(int i = 0; i < count; i++){
			notempty |= val[i] != 0;
		}
		return !notempty;
	}
	inline Vec_<T, count> conj(){
		Vec_<T, count > res;
		for(int i = 0; i < count; i++){
			res.val[i] = -val[i];
		}
		return res;
	}

	inline T sum() const{
		T res = T(0);
		for(int i = 0; i < count; i++)
			res += val[i];
		return res;
	}
	inline T min() const{
		T res = val[0];
		for(int i = 1; i < count; i++)
			res = std::min(res, val[i]);
		return res;
	}
	inline T max() const{
		T res = val[0];
		for(int i = 1; i < count; i++)
			res = std::max(res, val[i]);
		return res;
	}

	inline T norm() const{
		T ret = T(0);
		for(int i = 0; i < count; i++){
			ret += val[i] * val[i];
		}
		return ::sqrt(ret);
	}
	inline T dot(const Vec_<T, count>& v) const{
		T ret = 0;
		for(int i = 0; i < count; i++){
			ret += val[i] * v.val[i];
		}
		return ret;

	}
	inline Vec_< T, count > cross(const Vec_< T, count > & v2){
		Vec_< T, count > res;
		if(count != 3)
			return res;
		res[0] = val[1] * v2.val[2] - val[2] * v2.val[1];
		res[1] = val[2] * v2.val[0] - val[0] * v2.val[2];
		res[2] = val[0] * v2.val[1] - val[1] * v2.val[0];
		return res;
	}

	inline T* ptr(){
		return val;
	}
	inline const T* ptr() const{
		return val;
	}

	inline int size() const{
		return count;
	}

	operator std::string() const{
		std::stringstream ss;
		ss << "[";
		for(int i = 0; i < count; i++){
			ss << val[i] << " ";
		}
		ss << "]";
		return ss.str();
	}

	template< typename C >
	operator Vec_< C, count >() const{
		Vec_< C, count > res;
		for(int i = 0; i < count; i++){
			res.val[i] = val[i];
		}
		return res;
	}

	///******************************
	static inline Vec_< T, count > zeros(){
		Vec_< T, count > res;
		for(int i = 0; i < count; i++){
			res.val[i] = 0;
		}
		return res;
	}
	static inline Vec_< T, count > ones(){
		Vec_< T, count > res;
		for(int i = 0; i < count; i++){
			res.val[i] = 1;
		}
		return res;
	}

private:
};

template<typename T, int count>
inline Vec_<T, count> operator+ (const Vec_<T, count>& v1, const Vec_<T, count>& v2)
{
	Vec_<T, count> ret;

	for(int i = 0; i < count; i++){
		ret.val[i] = v1.val[i] + v2.val[i];
	}
	return ret;
}

template<typename T, int count>
inline Vec_<T, count> operator/ (const Vec_<T, count>& v1, const Vec_<T, count>& v2)
{
	Vec_<T, count> ret;

	for(int i = 0; i < count; i++){
		ret.val[i] = v1.val[i] / v2.val[i];
	}
	return ret;
}

template<typename T, int count>
inline Vec_<T, count> operator- (const Vec_<T, count>& v1, const Vec_<T, count>& v2)
{
	Vec_<T, count> ret;

	for(int i = 0; i < count; i++){
		ret.val[i] = v1.val[i] - v2.val[i];
	}
	return ret;
}

template<typename T, int count>
inline Vec_<T, count> operator* (const Vec_<T, count>& v1, const Vec_<T, count>& v2)
{
	Vec_<T, count> ret;

	for(int i = 0; i < count; i++){
		ret.val[i] = v1.val[i] * v2.val[i];
	}
	return ret;
}

template<typename T, int count>
inline Vec_<T, count> operator+ (const Vec_<T, count>& v1, T v2)
{
	Vec_<T, count> ret;

	for(int i = 0; i < count; i++){
		ret.val[i] = v1.val[i] + v2;
	}
	return ret;
}

template<typename T, int count>
inline Vec_<T, count> operator- (const Vec_<T, count>& v1, T v2)
{
	Vec_<T, count> ret;

	for(int i = 0; i < count; i++){
		ret.val[i] = v1.val[i] - v2;
	}
	return ret;
}

template<typename T, int count>
inline Vec_<T, count> operator* (const Vec_<T, count>& v1, T v2)
{
	Vec_<T, count> ret;

	for(int i = 0; i < count; i++){
		ret.val[i] = v1.val[i] * v2;
	}
	return ret;
}

template<typename T, int count>
inline Vec_<T, count> operator/ (const Vec_<T, count>& v1, T v2)
{
	Vec_<T, count> ret;

	for(int i = 0; i < count; i++){
		ret.val[i] = v1.val[i] / v2;
	}
	return ret;
}

template<typename T, int count>
inline Vec_<T, count> max (const Vec_<T, count>& v1, T v2)
{
	Vec_<T, count > res;
	for(int i = 0; i < count; i++){
		res[i] = std::max(v1.val[i], v2);
	}
	return res;
}

template<typename T, int count>
inline Vec_<T, count> min (const Vec_<T, count>& v1, T v2)
{
	Vec_<T, count > res;
	for(int i = 0; i < count; i++){
		res[i] = std::min(v1.val[i], v2);
	}
	return res;
}

template<typename T, int count>
inline Vec_<T, count> min (const Vec_<T, count>& v1, const Vec_<T, count>& v2)
{
	Vec_<T, count > res;
	for(int i = 0; i < count; i++){
		res[i] = std::min(v1.val[i], v2.val[i]);
	}
	return res;
}

template<typename T, int count>
inline Vec_<T, count> max (const Vec_<T, count>& v1, const Vec_<T, count>& v2)
{
	Vec_<T, count > res;
	for(int i = 0; i < count; i++){
		res[i] = std::max(v1.val[i], v2.val[i]);
	}
	return res;
}

template<typename T, int count>
inline Vec_<T, count> sign(const Vec_<T, count>& v1)
{
	Vec_<T, count > res;
	for(int i = 0; i < count; i++){
		res.val[i] = v1.val[i] >= 0? 1 : -1;
	}
	return res;
}

template<typename T, int count>
inline Vec_<T, count> sqrt(const Vec_<T, count>& v1)
{
	Vec_<T, count > res;
	for(int i = 0; i < count; i++){
		res[i] = sqrt(v1.val[i]);
	}
	return res;
}

/**
 * @brief crop_angles
 * @param v1
 * @return values in [-M_PI/2, M_PI/2]
 */
template<typename T, int count>
Vec_<T, count> crop_angles(const Vec_<T, count>& v1)
{
	Vec_<T, count > res;
	for(int i = 0; i < count; i++){
		res[i] = atan2(sin(v1.val[i]), cos(v1.val[i]));
	}
	return res;
}

/**
 * @brief crop_angle
 * @param value
 * @return value in [-M_PI/2, M_PI/2]
 */
template< typename T >
inline T crop_angle(const T& value)
{
	return atan2(sin(value), cos(value));
}

template< typename T >
inline T value2range(T value, T min_range, T max_range)
{
	return std::max(min_range, std::min(max_range, value));
}

template< typename T >
inline T values2range(T value, double min_range, double max_range)
{
	T res(value);
	res = min(res, max_range);
	res = max(res, min_range);
	return res;
}


typedef Vec_<float, 2> Vec2f;
typedef Vec_<double, 2> Vec2d;

typedef Vec_<float, 3> Vec3f;
typedef Vec_<double, 3> Vec3d;
typedef Vec_<int, 3> Vec3i;
typedef Vec_<unsigned int, 3> Vec3u;

typedef Vec_<float, 4> Vec4f;
typedef Vec_<double, 4> Vec4d;
typedef Vec_<int, 4> Vec4i;
typedef Vec_<unsigned int, 4> Vec4u;

template< typename T, int count >
std::ostream& operator<< (std::ostream& stream, const Vec_<T, count >& v)
{
	std::stringstream ss;
	ss << "[";
	for(int i = 0; i < count; i++){
		ss << v.val[i] << " ";
	}
	ss << "]";
	stream << ss.str();
	return stream;
}

/////////////////////////////////////////////

typedef std::vector< unsigned char > vector_uchar;
typedef std::shared_ptr< vector_uchar > shared_uvector;

template< typename T >
class Mat_{
public:
	enum {depth = sizeof(T)};
	std::vector< T > val;
	int cols;
	int rows;

	Mat_(){
		cols = rows = 0;
	}
	Mat_(int rows, int cols){
		this->rows = rows;
		this->cols = cols;
		val.resize(rows * cols);
	}
	Mat_(const Mat_<T>& m){
		val = m.val;
		rows = m.rows;
		cols = m.cols;
	}
	Mat_(int rows, int cols, void* data){
		this->rows = rows;
		this->cols = cols;
		val.resize(rows * cols);
		std::copy(data, data + rows * cols * depth, (char*)&val[0]);
	}
	template< int count >
	Mat_(const Vec_< T, count>& v){
		rows = 1;
		cols = count;
		val.resize(rows * cols);
		std::copy((char*)v.val, (char*)v.val + sizeof(v.val), (char*)&val[0]);
	}
	///**********************
	Mat_<T> operator= (const Mat_<T>& m){
		this->rows = rows;
		this->cols = cols;
		val = m.val;
//		val.resize(rows * cols);
//		std::copy(data, data + rows * cols * depth, (char*)&val[0]);
		return *this;
	}
	template< int count >
	Mat_<T> operator= (const Vec_<T, count>& v){
		this->rows = 1;
		this->cols = count;
		val = v.val;
		return *this;
	}
	///***********************
	inline int total() const{
		return rows * cols;
	}
	///********************
	inline T* ptr(){
		return &val[0];
	}
	inline T* ptr() const{
		return &val[0];
	}
	///****************
	inline char* bytes(){
		return (char*)val[0];
	}
	inline char* bytes() const{
		return (char*)val[0];
	}
	///*********************
	inline T& at(int i0, int i1){
		return val[i0 * cols + i1];
	}
	inline T& at(int i0){
		return val[i0 * cols];
	}
	inline const T& at(int i0, int i1)const{
		return val[i0 * cols + i1];
	}
	inline const T& at(int i0)const {
		return val[i0 * cols];
	}
	///********************
	inline Mat_<T> operator *= (T v){
		for(int i = 0; i < rows * cols; i++){
			val[i] *= v;
		}
	}
	inline Mat_<T> operator += (T v){
		for(int i = 0; i < rows * cols; i++){
			val[i] += v;
		}
	}
	inline Mat_<T> operator -= (T v){
		for(int i = 0; i < rows * cols; i++){
			val[i] -= v;
		}
	}
	///**************************
	Mat_<T> t() const{
		Mat_<T> res(cols, rows);

		for(int i = 0; i < rows; i++){
			for(int j = 0; j < cols; j++){
				res.val[j * rows + i] = val[i * cols + j];
			}
		}

		return res;
	}
	///***********************
	template < int count >
	inline Vec_<T, count > toVecCol(int col = 0) const{
		Vec_< T, count > res;

		if(count != rows)
			return res;

		for(int i = 0; i < rows; i++){
			res.val[i] = val[i * cols + col];
		}
		return res;
	}
	template < int count >
	inline Vec_<T, count > toVecRow(int row = 0) const{
		Vec_< T, count > res;

		if(count != cols)
			return res;

		for(int i = 0; i < cols; i++){
			res.val[i] = val[row * cols + i];
		}
		return res;
	}
	template < int count >
	inline Vec_<T, count > toVec() const{
		Vec_< T, count > res;

		if((cols == 1 || count == rows) && (rows == 1 || count == cols))
			return res;

		for(int i = 0; i < count; i++){
			res.val[i] = val[i];
		}
		return res;
	}
	///**************************
	operator std::string() const{
		std::stringstream res;
		res << "[";
		for(int i = 0; i < rows; i++){
			for(int j = 0; j < cols; j++){
				res << val[i * cols + j] << " ";
			}
			res << ";\n";
		}
		res << "]";
		return res.str();
	}
	///**************************
	static inline Mat_< T > zeros(int rows, int cols){
		Mat_< T > res(rows, cols);
		res.val.resize(rows * cols, 0);
		return res;
	}
	static inline Mat_< T > eye(int rows, int cols){
		Mat_< T > res = zeros(rows, cols);
		for(int i = 0; i < std::min(rows, cols); ++i){
			res.val[i * cols + i] = 1;
		}
		return res;
	}

private:
};

template< typename T >
inline Mat_<T> operator* (const Mat_<T>& m1, const Mat_<T>& m2)
{
	if(m1.cols != m2.rows)
		return Mat_<T>();
	int r = m1.rows;
	int c = m2.cols;
	Mat_<T> res(r, c);

	for(int i = 0; i < m1.rows; i++){
		for(int k = 0; k < m2.cols; k++){
			T s = 0;
			for(int j = 0; j < m1.cols; j++){
				s += m1.at(i, j) * m2.at(j, k);
			}
			res.at(i, k) = s;
		}
	}

	return res;
}

template< typename T >
inline Mat_<T> operator+ (const Mat_<T>& m1, const Mat_<T>& m2)
{
	if(m1.cols != m2.cols || m1.rows != m2.rows)
		return Mat_<T>();
	Mat_<T> res(m1.rows, m1.cols);

	for(int i = 0; i < m1.rows * m1.cols; i++){
		res.val[i] = m1.val[i] + m2.val[i];
	}

	return res;
}

template< typename T >
inline Mat_<T> operator- (const Mat_<T>& m1, const Mat_<T>& m2)
{
	if(m1.cols != m2.cols || m1.rows != m2.rows)
		return Mat_<T>();
	Mat_<T> res(m1.rows, m1.cols);

	for(int i = 0; i < m1.rows * m1.cols; i++){
		res.val[i] = m1.val[i] - m2.val[i];
	}

	return res;
}

template< typename T >
inline Mat_<T> operator* (const Mat_<T>& m1, T v)
{
	Mat_<T> res(m1.rows, m1.cols);

	for(int i = 0; i < m1.rows * m1.cols; i++){
		res.val[i] = m1.val[i] * v;
	}

	return res;
}

template< typename T, int count >
inline Mat_<T> operator* (const Mat_<T>& m1, const Vec_< T, count >& v)
{
	Mat_<T> res(m1.rows, 1);

	if(m1.cols != count)
		return res;

	for(int i = 0; i < m1.rows; i++){
		T s = 0;
		for(int j = 0; j < m1.cols; j++){
			s += m1.val[i * m1.cols + j] * v.val[j];
		}
		res.val[i] = s;
	}

	return res;
}

typedef Mat_<float> Matf;
typedef Mat_<double> Matd;

/**
 * @brief get_tangage_mat
 * @param angle
 * @return
 */
template< typename T >
inline Mat_< T > get_tangage_mat(T angle)
{
	T data[9] = {
		1,	0,				0,
		0,	sin(angle),		cos(angle),
		0,	cos(angle),		-sin(angle),
	};
	return Mat_<T>(3, 3, data);
}

/**
 * @brief get_roll_mat
 * @param angle
 * @return
 */
template< typename T >
inline Mat_< T > get_roll_mat(T angle)
{
	T data[9] = {
		sin(angle),		0,		cos(angle),
		0,				1,				0,
		cos(angle),		0,		-sin(angle),
	};
	return Mat_<T>(3, 3, data);
}

/**
 * @brief get_yaw_mat
 * @param angle
 * @return
 */
template< typename T >
inline Mat_< T > get_yaw_mat(T angle)
{
	T data[9] = {
		sin(angle),		cos(angle),		0,
		cos(angle),		-sin(angle),	0,
		0,				0,				1
	};
	return Mat_<T>(3, 3, data);
}

template< typename T >
inline Mat_< T > get_eiler_mat(const Vec_< T, 3 >& angles)
{
	T yaw, tangage, bank;
	Mat_< T > m(3, 3);

	yaw		= angles[0];
	tangage	= angles[1];
	bank	= angles[2];

	m.at(0, 0) = cos(yaw) * cos(bank) - sin(yaw) * sin(tangage) * sin(bank);
	m.at(0, 1) = -cos(yaw) * sin(bank) - sin(yaw) * sin(tangage) * cos(bank);
	m.at(0, 2) = -sin(yaw) * cos(tangage);

	m.at(1, 0) = cos(tangage) * sin(bank);
	m.at(1, 1) = cos(tangage) * cos(bank);
	m.at(1, 2) = -sin(tangage);

	m.at(2, 0) = sin(yaw) * cos(bank) + cos(yaw) * sin(tangage) * sin(bank);
	m.at(2, 1) = -sin(yaw) * sin(bank) + cos(yaw) * sin(tangage) * cos(bank);
	m.at(2, 2) = cos(yaw) * cos(tangage);

	return m;
}

template< typename T >
inline Mat_< T > get_eiler_mat4(const Vec_< T, 3 >& angles)
{
	T yaw, tangage, bank;
	Mat_< T > m = Mat_< T >::eye(4, 4);

	yaw		= angles[0];
	tangage	= angles[1];
	bank	= angles[2];

	m.at(0, 0) = cos(yaw) * cos(bank) - sin(yaw) * sin(tangage) * sin(bank);
	m.at(0, 1) = -cos(yaw) * sin(bank) - sin(yaw) * sin(tangage) * cos(bank);
	m.at(0, 2) = -sin(yaw) * cos(tangage);

	m.at(1, 0) = cos(tangage) * sin(bank);
	m.at(1, 1) = cos(tangage) * cos(bank);
	m.at(1, 2) = -sin(tangage);

	m.at(2, 0) = sin(yaw) * cos(bank) + cos(yaw) * sin(tangage) * sin(bank);
	m.at(2, 1) = -sin(yaw) * sin(bank) + cos(yaw) * sin(tangage) * cos(bank);
	m.at(2, 2) = cos(yaw) * cos(tangage);

	return m;
}

/**
 * @brief get_yaw_mat2
 * @param yaw
 * @return
 */
template< typename T >
inline Mat_<T> get_yaw_mat2(T yaw)
{
	T data[9] = {
		cos(yaw),	0,		-sin(yaw),
		0.,			1,		0,
		sin(yaw),	0,		cos(yaw),
	};
	Mat_<T> res(3, 3, data);
	return res;
}

///////////////////////////

#define M_PI       3.14159265358979323846

template< typename T >
inline T angle2rad( T val)
{
	return static_cast< T > (val * M_PI / 180.);
}

template< typename T >
inline T rad2angle(T val)
{
	return static_cast< T > (val * 180. / M_PI);
}

}

#endif // CUSTOM_TYPES_H
