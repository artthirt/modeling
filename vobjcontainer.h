#ifndef VOBJCONTAINER_H
#define VOBJCONTAINER_H

#include <vector>
#include <deque>
#include <string>

#include <custom_types.h>

class VObj{
public:
	std::vector< std::string > names;
	std::vector< ct::Vec3f > v;
	std::vector< ct::Vec3f > vn;
	std::vector< ct::Vec3f > t;
	std::vector< std::vector< int > > fv;
	std::vector< std::vector< int > > fn;
	std::vector< std::vector< int > > ft;

	void clear();
};

class VObjContainer
{
public:
	VObjContainer();

	bool open(const std::string &fn);

	const std::deque<VObj> &vobjs() const;

private:
	std::string m_fn;
	std::deque< VObj > m_vobjs;
};

#endif // VOBJCONTAINER_H
