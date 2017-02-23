/* 
 *  Copyright (c) 2011  Chen Feng (cforrest (at) umich.edu)
 *    and the University of Michigan
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */

#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <vector>

#include "updator.h"
#include "VPCluster.h"
#include "VPSample.h"

std::vector< std::vector<float> *> pts;

inline void Read_Line(std::istream& in)
{
	while(in) {
		std::string str;
		std::getline(in,str,'\n');
		if(str.length()==0) continue;

		double x0,y0,x1,y1;
		sscanf(str.c_str(), 
			"%lf%*[^0-9-+.eE]%lf%*[^0-9-+.eE]%lf%*[^0-9-+.eE]%lf", &x0,&y0,&x1,&y1);
		double dx=x0-x1,dy=y0-y1;
		if( dx*dx+dy*dy<400 )
			continue;

		std::vector<float>* p = new std::vector<float>(4);
		pts.push_back(p);
		(*p)[0]=(float)x0;
		(*p)[1]=(float)y0;
		(*p)[2]=(float)x1;
		(*p)[3]=(float)y1;
	}
	std::cout<<"Read Line Done!"<<std::endl;
}

// exe infilename outputfilename
int main(int argc, const char* argv[])
{
	if(argc<3) {
		std::cout<<argv[0]<<" infilename outfilename"<<std::endl;
		return 1;
	}

	std::ifstream ifile(argv[1]);
	Read_Line(ifile);

	std::vector<unsigned int> Lables;
	std::vector<unsigned int> LableCount;
	{//2. VP cluster
		std::vector<std::vector<float> *> *mModels = 
			VPSample::run(&pts, 5000, 2, 0, 3);
		int classNum = VPCluster::run(Lables, LableCount, &pts, mModels, 2, 2);
		std::cout << "vpdetection found " << classNum<<" classes!"<<std::endl;

		//2.1. release other resource
		for(unsigned int i=0; i < mModels->size(); ++i)
			delete (*mModels)[i];
		delete mModels;
	}

	std::ofstream ofile(argv[2]);
	unsigned int len = (unsigned int)Lables.size();
	for(unsigned int i=0; i<len; ++i) {
		ofile << (*pts[i])[0] << " " << (*pts[i])[1] << " "
			<< (*pts[i])[2] << " " << (*pts[i])[3] << " " << Lables.at(i)<<std::endl;
	}
	ofile.close();

	for(unsigned int i=0; i<pts.size(); i++)
		delete pts[i];

	return 0;
}
