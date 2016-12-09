#ifndef VOBJCONTAINER_H
#define VOBJCONTAINER_H

#include <vector>
#include <deque>
#include <string>
#include <map>

#include <custom_types.h>

class Mtl{
public:
	Mtl(){
		Ns = Ni = 0;
		d = 0;
		illum = 0;
	}

	ct::Vec4f Ka;
	ct::Vec4f Kd;
	ct::Vec4f Ks;
	ct::Vec4f Ke;
	float Ns;
	float Ni;
	float d;
	float illum;
};

class VObj{
public:
	std::string mtlfile;

	std::vector< ct::Vec3d > v;
	std::vector< ct::Vec3d > vn;
	std::vector< ct::Vec3d > t;

	std::map< std::string, Mtl > mtls;

	class Faces{
	public:
		std::string usemtl;
//		std::string name;
		std::vector< std::vector< int > > fv;
		std::vector< std::vector< int > > fn;
		std::vector< std::vector< int > > ft;

		void clear();
	};

	std::map< std::string, Faces > faces;

	void clear();
	void openmtl(const std::string dir = "");
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
