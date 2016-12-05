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

	bool created = false;

	while(std::getline(fs, str)){
		QString qstr = QString::fromStdString(str);

		qstr = qstr.trimmed();
		QStringList sl = qstr.split(" ");
		if(sl.size()){


			if(sl[0] == "o"){

				if(created){
					//m_vobjs.push_back(obj);
				}

				//obj.clear();
				qDebug() << "name: " << sl[1] << "; vertex offset: " << obj.v.size();

				obj.names.push_back(sl[1].toStdString());
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
				std::vector < int > facev;
				std::vector < int > facen;
				std::vector < int > facet;
				for(int i = 1; i < sl.size(); ++i){
					QStringList face = sl[i].split('/');
					if(face.size()){
						facev.push_back(face[0].toInt());
						if(!face[1].isEmpty()){
							facet.push_back(face[1].toInt());
						}
						if(!face[2].isEmpty()){
							facen.push_back(face[2].toInt());
						}
					}
				}
				obj.fv.push_back(facev);
				obj.ft.push_back(facet);
				obj.fn.push_back(facen);
			}
		}
	}

	if(created && !obj.v.empty()){
		m_vobjs.push_back(obj);
	}

	for(auto it = m_vobjs.begin(); it != m_vobjs.end(); it++){
		VObj& obj = *it;

		size_t cnt = obj.v.size();
		for(size_t i = 0; i < obj.fv.size(); i++){
			std::vector< int >& inds = obj.fv[i];
			for(size_t j = 0; j < inds.size(); j++){
				if(inds[j] < 0)
					inds[j] = cnt + inds[j];
			}

			cnt = obj.t.size();
			std::vector< int >& indst = obj.ft[i];
			for(size_t j = 0; j < indst.size(); j++){
				if(indst[j] < 0)
					indst[j] = cnt + indst[j];
			}

			cnt = obj.vn.size();
			std::vector< int >& indsn = obj.fn[i];
			for(size_t j = 0; j < indsn.size(); j++){
				if(indsn[j] < 0)
					indsn[j] = cnt + indsn[j];
			}
		}
	}

	return true;
}

const std::deque<VObj> &VObjContainer::vobjs() const
{
	return m_vobjs;
}

void VObj::clear()
{
	names.clear();
	v.clear();
	vn.clear();
	t.clear();
	fv.clear();
	fn.clear();
	ft.clear();
}
