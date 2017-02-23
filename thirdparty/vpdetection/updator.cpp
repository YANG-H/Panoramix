#include "updator.h"
#include <iostream>
#include <math.h>
#include <ctime>

namespace Updator {
	double percent;
	time_t initialTime, currentTime;
	char* gstr=0;

	void InitializeWaitbar(char* str){
		// Initialize waitbar
		gstr = str;
		if(gstr) std::cout<<gstr;
		std::cout<<" In progress ..."<<std::endl;
		percent = 0.0;

		initialTime = time(NULL);
	}

	void UpdateWaitbar(float nValue){
		// update the waitbar
		static float nPreviousValue = 0.0f;

		if( (int)(floor(nValue * 100.f)) - (int) (floor(nPreviousValue * 100.f)) > 0){
			currentTime = time(NULL);
			if((currentTime - initialTime) == 0){
				nPreviousValue = nValue;
				return;
			}

			int estimetedTimeInSeconds = 0;
			if(nValue > 0.0)
				estimetedTimeInSeconds =(int)( 
				((float)(currentTime - initialTime)  
				* (float)((1.f-nValue) * 100.f)) /(float)((nValue) * 100.f)) ;

			int hours = (estimetedTimeInSeconds) / (3600);
			int minutes = ((estimetedTimeInSeconds) / 60) % 60;
			int seconds = (estimetedTimeInSeconds) % 60;

			percent = nValue;
			std::cout<<"\t(Estimated time = "<<hours<<" h : "
				<<minutes<<" m : "<<seconds<<" s)\t\t\r";
		}

		nPreviousValue = nValue;
	}

	void CloseWaitbar(){
		if(gstr) std::cout<<"\n"<<gstr;
		std::cout<<" Done!"<<std::endl;
	}
}
