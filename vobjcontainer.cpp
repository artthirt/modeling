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

	string str;

	VObj obj;
	VObj::Faces faces;

	bool created = false;

	while(std::getline(fs, str)){
		QString qstr = QString::fromStdString(str);

		qstr = qstr.trimmed();
		QStringList sl = qstr.split(" ");
		if(sl.size()){


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
			}
			if(sl[0] == "v"){
				obj.v.push_back(Vec3f(sl[1].toFloat(),
								sl[2].toFloat(),
								sl[3].toFloat()));
			}
			if(sl[0] == "vn"){
				obj.vn.push_back(Vec3f(sl[1].toFloat(),
								sl[2].toFloat(),
								sl[3].toFloat()));
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
			}
		}
	}

	if(created && !obj.v.empty()){
		obj.faces.push_back(faces);
		m_vobjs.push_back(obj);
	}

	for(auto it = m_vobjs.begin(); it != m_vobjs.end(); it++){
		VObj& obj = *it;

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

void VObj::Faces::clear()
{
	name = "";
	fv.clear();
	fn.clear();
	ft.clear();
}
