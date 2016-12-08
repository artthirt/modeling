#include "vobjcontainer.h"

#include <iostream>
#include <fstream>
#include <algorithm>
#include <functional>
#include <locale>

//#include <QString>
//#include <QStringList>
//#include <QDebug>

using namespace std;
using namespace ct;

///////////////////////////////////////

typedef std::vector< std::string > stringlist;

struct __isspace: std::unary_function< bool, char >
{
	bool operator() (char val) const{
		return std::isspace(val, std::locale());
	}
};
#ifdef _MSC_VER
template<class _Elem> inline
	bool (isspace1)(_Elem _Ch)
	{	// test if character is whitespace, locale specific
	return (_USE(std::locale(), ctype<_Elem>).is(ctype_base::space, _Ch));
	}

// trim from start (in place)
static inline void ltrim(string& s)
{
	s.erase(s.begin(), std::find_if(s.begin(), s.end(),
			std::not1(std::ptr_fun<int, bool>(isspace1))));
}

// trim from end (in place)
static inline void rtrim(std::string &s) {
	s.erase(std::find_if(s.rbegin(), s.rend(),
			std::not1(std::ptr_fun<int, bool>(isspace1))).base(), s.end());
}

#else
// trim from start (in place)
static inline void ltrim(string& s)
{
	s.erase(s.begin(), std::find_if(s.begin(), s.end(),
			std::not1(std::ptr_fun<int, int>(std::isspace))));
}

// trim from end (in place)
static inline void rtrim(std::string &s) {
	s.erase(std::find_if(s.rbegin(), s.rend(),
			std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
}
#endif

static inline string trim(const string val)
{
	string res = val;
	ltrim(res);
	rtrim(res);
	//res = rtrim(val);
	return res;
}

static inline stringlist split(const std::string& val, char delim)
{
	std::istringstream is(val);
	stringlist res;
	std::string str;

	while(std::getline(is, str, delim)){
		res.push_back(str);
	}
	return res;
}

inline float toFloat(const std::string& val)
{
	double res;
	stringstream ss = stringstream(val);
	ss >> res;
	return static_cast<float>(res);
}

inline int toInt(const std::string& val)
{
	int res;
	stringstream ss = stringstream(val);
	ss >> res;
	return res;
}

///////////////////////////////////////

VObjContainer::VObjContainer()
{

}

bool VObjContainer::open(const std::string &fn)
{
	std::fstream fs;
	fs.open(fn.c_str(), ios_base::in);

	if(!fs.is_open())
		return false;

	std::string dir;

//	string s = "    s12345   ";
//	s = trim(s);

//	{
//		QString t = QString(fn.c_str());
//		int i = t.lastIndexOf("/");
//		if(i < 0){
//			i = t.lastIndexOf("\\");
//		}
//		if(i >= 0){
//			QStringRef tr = t.leftRef(i);
//			dir = tr.toString().toStdString();
//		}
//	}

	{
		size_t i = fn.find_last_of('/');
		if(i == string::npos){
			i = fn.find_last_of('\\');
		}
		if(i != string::npos){
			dir = fn.substr(0, i);
		}
	}

	string str;
	string obj_name;

	VObj obj;
//	VObj::Faces faces;

//	bool created = false;

	while(std::getline(fs, str)){
		//QString qstr = QString::fromStdString(str);
		string qstr = trim(str);

		//qstr = qstr.trimmed();
		//QStringList sl = qstr.split(" ");
		stringlist sl = split(qstr, ' ');
		if(sl.size()){

			if(sl[0] == "mtllib"){
				obj.mtlfile = sl[1];
				continue;
			}
			if(sl[0] == "o"){

//				if(created){
//					//m_vobjs.push_back(obj);
//					obj.faces.push_back(faces);
//					faces.clear();
//				}

				//obj.clear();
				cout << "name: " << sl[1].c_str() << "; vertex offset: " << obj.v.size();

				obj_name = sl[1];

				obj.faces[obj_name] = VObj::Faces();
//				created = true;
				continue;
			}
			if(sl[0] == "usemtl"){
				obj.faces[obj_name].usemtl = sl[1];
				continue;
			}
			if(sl[0] == "v"){
				obj.v.push_back(Vec3f(toFloat(sl[1]),
								toFloat(sl[2]),
								toFloat(sl[3])));
				continue;
			}
			if(sl[0] == "vn"){
				obj.vn.push_back(Vec3f(toFloat(sl[1]),
								toFloat(sl[2]),
								toFloat(sl[3])));
				continue;
			}
			if(sl[0] == "vt"){
				obj.vn.push_back(Vec3f(toFloat(sl[1]),
								toFloat(sl[2]),
								sl.size() > 3? toFloat(sl[3]) : 1.f));
				continue;
			}
			if(sl[0] == "f"){
				std::vector< int > fvi;
				std::vector< int > fti;
				std::vector< int > fni;

				for(int i = 1; i < sl.size(); ++i){
					stringlist face = split(sl[i], '/');
					if(face.size()){
						fvi.push_back(toInt(face[0]));
						if(!face[1].empty()){
							fti.push_back(toInt(face[1]));
						}
						if(!face[2].empty()){
							fni.push_back(toInt(face[2]));
						}
					}
				}
				obj.faces[obj_name].fv.push_back(fvi);
				obj.faces[obj_name].ft.push_back(fti);
				obj.faces[obj_name].fn.push_back(fni);
				continue;
			}
		}
	}

	m_vobjs.push_back(obj);

	fs.close();

//	if(created && !obj.v.empty()){
//		obj.faces.push_back(faces);
//		m_vobjs.push_back(obj);
//	}

	for(auto it = m_vobjs.begin(); it != m_vobjs.end(); it++){
		VObj& obj = *it;

		obj.openmtl(dir);

		size_t cnt = obj.v.size();
		for(auto it = obj.faces.begin(); it != obj.faces.end(); it++){
			VObj::Faces & faces = (*it).second;

			cout << "face:" << (*it).first.c_str() << faces.fn.size() << faces.ft.size() << faces.fv.size();

			for(size_t i = 0; i < faces.fv.size(); i++){


				std::vector< int >& inds = faces.fv[i];
				for(size_t j = 0; j < inds.size(); j++){
					if(inds[j] < 0)
						inds[j] = cnt + inds[j];
				}

				cnt = obj.t.size();
				std::vector< int >& indst = faces.ft[i];
				for(size_t j = 0; j < indst.size(); j++){
					if(indst[j] < 0)
						indst[j] = cnt + indst[j];
				}

				cnt = obj.vn.size();
				std::vector< int >& indsn = faces.fn[i];
				for(size_t j = 0; j < indsn.size(); j++){
					if(indsn[j] < 0)
						indsn[j] = cnt + indsn[j];
				}
			}
		}
	}

	return true;
}

const std::deque<VObj> &VObjContainer::vobjs() const
{
	return m_vobjs;
}

////////////////////////////////////////

void VObj::clear()
{
	v.clear();
	vn.clear();
	t.clear();
}

void VObj::openmtl(const string dir)
{
	if(mtlfile.empty())
		return;

	std::string fn = dir + "/" + mtlfile;

	std::fstream fs;
	fs.open(fn.c_str(), ios_base::in);

	if(!fs.is_open())
		return;

	string str;

	std::string smtl;
	Vec4f Ka, Kd, Ks, Ke;
	Ka[3] = Kd[3] = Ks[3] = Ke[3] = 1.;

	while(std::getline(fs, str)){
		string qstr = trim(str);

//		qstr = qstr.trimmed();
		stringlist sl = split(qstr, ' ');

		if(sl.empty())
			continue;
		if(sl[0] == "newmtl"){
			smtl = sl[1];
			mtls[smtl] = Mtl();
		}
		if(sl[0] == "Ns"){
			mtls[smtl].Ns = toFloat(sl[1]);
		}
		if(sl[0] == "Ni"){
			mtls[smtl].Ns = toFloat(sl[1]);
		}
		if(sl[0] == "d"){
			mtls[smtl].Ns = toFloat(sl[1]);
		}
		if(sl[0] == "illum"){
			mtls[smtl].Ns = toFloat(sl[1]);
		}
		if(sl[0] == "Ka"){
			Ka[0] = toFloat(sl[1]);
			Ka[1] = toFloat(sl[2]);
			Ka[2] = toFloat(sl[3]);
			mtls[smtl].Ka = Ka;
		}
		if(sl[0] == "Kd"){
			Kd[0] = toFloat(sl[1]);
			Kd[1] = toFloat(sl[2]);
			Kd[2] = toFloat(sl[3]);
			mtls[smtl].Kd = Kd;
		}
		if(sl[0] == "Ks"){
			Ks[0] = toFloat(sl[1]);
			Ks[1] = toFloat(sl[2]);
			Ks[2] = toFloat(sl[3]);
			mtls[smtl].Ks = Ks;
		}
		if(sl[0] == "Ke"){
			Ke[0] = toFloat(sl[1]);
			Ke[1] = toFloat(sl[2]);
			Ke[2] = toFloat(sl[3]);
			mtls[smtl].Ke = Ke;
		}
	}
}

void VObj::Faces::clear()
{
	fv.clear();
	fn.clear();
	ft.clear();
}
