#include "modelroute.h"
#include <random>

using namespace std;
using namespace ct;

ModelRoute::ModelRoute()
	: m_current_index(0)
{

}

void ModelRoute::generate_route_norm(size_t count, const ct::Vec3f &mean, const ct::Vec3f &stdev)
{
#ifdef _MSC_VER
	std::tr1::mt19937 generator;
	std::tr1::normal_distribution< float > v1 = std::tr1::normal_distribution< float >(mean[0], stdev[0]);
	std::tr1::normal_distribution< float > v2 = std::tr1::normal_distribution< float >(mean[1], stdev[1]);
	std::tr1::normal_distribution< float > v3 = std::tr1::normal_distribution< float >(mean[2], stdev[2]);
#else
	std::mt19937 generator;
	std::normal_distribution< float > v1 = std::normal_distribution< float >(mean[0], stdev[0]);
	std::normal_distribution< float > v2 = std::normal_distribution< float >(mean[1], stdev[1]);
	std::normal_distribution< float > v3 = std::normal_distribution< float >(mean[2], stdev[2]);
#endif

	m_points.resize(count);

	for(size_t i = 0; i < count; i++){
		m_points[i][0] = v1(generator);
		m_points[i][1] = v2(generator);
		m_points[i][2] = v3(generator);
	}

	first();
}

void ModelRoute::generate_route_uniform(size_t count, const ct::Vec3f &a, const ct::Vec3f &b)
{
#ifdef _MSC_VER
	std::tr1::mt19937 generator;
	std::tr1::uniform_real_distribution< float > v1 = std::tr1::uniform_real_distribution< float >(a[0], b[0]);
	std::tr1::uniform_real_distribution< float > v2 = std::tr1::uniform_real_distribution< float >(a[1], b[1]);
	std::tr1::uniform_real_distribution< float > v3 = std::tr1::uniform_real_distribution< float >(a[2], b[2]);
#else
	std::mt19937 generator;
	std::uniform_real_distribution< float > v1 = std::uniform_real_distribution< float >(a[0], b[0]);
	std::uniform_real_distribution< float > v2 = std::uniform_real_distribution< float >(a[1], b[1]);
	std::uniform_real_distribution< float > v3 = std::uniform_real_distribution< float >(a[2], b[2]);
#endif
	m_points.resize(count);

	for(size_t i = 0; i < count; i++){
		m_points[i][0] = v1(generator);
		m_points[i][1] = v2(generator);
		m_points[i][2] = v3(generator);
	}

	first();
}

size_t ModelRoute::count() const
{
	return m_points.size();
}

std::vector<ct::Vec3f> &ModelRoute::points()
{
	return m_points;
}

ct::Vec3f zero_vec = ct::Vec3f::zeros();

Vec3f &ModelRoute::current_point()
{
	if(!isEnd())
		return m_points[m_current_index];
	return zero_vec;
}

const Vec3f &ModelRoute::current_point() const
{
	if(!isEnd())
		return m_points[m_current_index];
	return ct::Vec3f::zeros();
}

size_t ModelRoute::current_index() const
{
	return m_current_index;
}

float ModelRoute::distance(const ct::Vec3f &pt) const
{
	if(m_points.size() && !isEnd())
		return (pt - m_points[m_current_index]).norm();
	return 99999999999999.f;
}

Vec3f ModelRoute::direction(Vec3f &pt) const
{
	Vec3f v = current_point() - pt;
	float n = v.norm();
	if(n > 0)
		return v / n;
	return Vec3f::zeros();
}

void ModelRoute::first()
{
	m_current_index = 0;
}

bool ModelRoute::next()
{
	if(m_current_index < m_points.size())
		m_current_index++;
	else
		return false;
	return true;
}

bool ModelRoute::isEnd() const
{
	return m_current_index >= m_points.size();
}
