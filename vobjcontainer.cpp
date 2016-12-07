#include "vobjcontainer.h"

#include <iostream>
#include <fstream>

#include <QString>
#include <QStringList>
#include <QDebug>

using namespace std;
using namespace ct;

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

	{
		QString t = QString(fn.c_str());
		int i = t.lastIndexOf("/");
		if(i < 0){
			i = t.lastIndexOf("\\");
		}
		if(i >= 0){
			QStringRef tr = t.leftRef(i);
			dir = tr.toString().toStdString();
		}
	}

	string str;

	VObj obj;
	VObj::Faces faces;

	bool created = false;

	while(std::getline(fs, str)){
		QString qstr = QString::fromStdString(str);

		qstr = qstr.trimmed();
		QStringList sl = qstr.split(" ");
		if(sl.size()){


			if(sl[0] == "mtllib"){
				obj.mtlfile = sl[1].toStdString();
				continue;
			}
			if(sl[0] == "o"){

				if(created){
					//m_vobjs.push_back(obj);
					obj.faces.push_back(faces);
					faces.clear();
				}

				//obj.clear();
				qDebug() << "name: " << sl[1] << "; vertex offset: " << obj.v.size();

				faces.name = sl[1].toStdString();
				created = true;
				continue;
			}
			if(sl[0] == "usemtl"){
				faces.usemtl = sl[1].toStdString();
				continue;
			}
			if(sl[0] == "v"){
				obj.v.push_back(Vec3f(sl[1].toFloat(),
								sl[2].toFloat(),
								sl[3].toFloat()));
				continue;
			}
			if(sl[0] == "vn"){
				obj.vn.push_back(Vec3f(sl[1].toFloat(),
								sl[2].toFloat(),
								sl[3].toFloat()));
				continue;
			}
			if(sl[0] == "f"){
				std::vector< int > fvi;
				std::vector< int > fti;
				std::vector< int > fni;

				for(int i = 1; i < sl.size(); ++i){
					QStringList face = sl[i].split('/');
					if(face.size()){
						fvi.push_back(face[0].toInt());
						if(!face[1].isEmpty()){
							fti.push_back(face[1].toInt());
						}
						if(!face[2].isEmpty()){
							fni.push_back(face[2].toInt());
						}
					}
				}
				faces.fv.push_back(fvi);
				faces.ft.push_back(fti);
				faces.fn.push_back(fni);
				continue;
			}
		}
	}

	fs.close();

	if(created && !obj.v.empty()){
		obj.faces.push_back(faces);
		m_vobjs.push_back(obj);
	}

	for(auto it = m_vobjs.begin(); it != m_vobjs.end(); it++){
		VObj& obj = *it;

		obj.openmtl(dir);

		size_t cnt = obj.v.size();
		for(auto it = obj.faces.begin(); it != obj.faces.end(); it++){
			VObj::Faces & faces = *it;

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
		QString qstr = QString::fromStdString(str);

		qstr = qstr.trimmed();
		QStringList sl = qstr.split(" ");

		if(sl[0] == "newmtl"){
			smtl = sl[1].toStdString();
			mtls[smtl] = Mtl();
		}
		if(sl[0] == "Ns"){
			mtls[smtl].Ns = sl[1].toFloat();
		}
		if(sl[0] == "Ni"){
			mtls[smtl].Ns = sl[1].toFloat();
		}
		if(sl[0] == "d"){
			mtls[smtl].Ns = sl[1].toFloat();
		}
		if(sl[0] == "illum"){
			mtls[smtl].Ns = sl[1].toFloat();
		}
		if(sl[0] == "Ka"){
			Ka[0] = sl[1].toFloat();
			Ka[1] = sl[2].toFloat();
			Ka[2] = sl[3].toFloat();
			mtls[smtl].Ka = Ka;
		}
		if(sl[0] == "Kd"){
			Kd[0] = sl[1].toFloat();
			Kd[1] = sl[2].toFloat();
			Kd[2] = sl[3].toFloat();
			mtls[smtl].Kd = Kd;
		}
		if(sl[0] == "Ks"){
			Ks[0] = sl[1].toFloat();
			Ks[1] = sl[2].toFloat();
			Ks[2] = sl[3].toFloat();
			mtls[smtl].Ks = Ks;
		}
		if(sl[0] == "Ke"){
			Ke[0] = sl[1].toFloat();
			Ke[1] = sl[2].toFloat();
			Ke[2] = sl[3].toFloat();
			mtls[smtl].Ke = Ke;
		}
	}
}

void VObj::Faces::clear()
{
	name = "";
	fv.clear();
	fn.clear();
	ft.clear();
}
