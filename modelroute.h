#ifndef MODELROUTE_H
#define MODELROUTE_H

#include "custom_types.h"

#include <iostream>
#include <vector>

class ModelRoute
{
public:
	ModelRoute();

	void generate_route_norm(size_t count, const ct::Vec3f &mean = ct::Vec3f(0, 0, 0), const ct::Vec3f &stdev = ct::Vec3f(1, 1, 1));
	void generate_route_uniform(size_t count, const ct::Vec3f &a = ct::Vec3f(0, 0, 0), const ct::Vec3f &b = ct::Vec3f(1, 1, 1));
	size_t count() const;

	std::vector<ct::Vec3f> &points();

	ct::Vec3f& current_point();
	const ct::Vec3f& current_point() const;
	size_t current_index() const;
	float distance(const ct::Vec3f& pt) const;
	ct::Vec3f direction(ct::Vec3f& pt) const;

	void first();
	bool next();
	bool isEnd() const;
private:
	std::vector< ct::Vec3f > m_points;

	size_t m_current_index;
};

#endif // MODELROUTE_H
