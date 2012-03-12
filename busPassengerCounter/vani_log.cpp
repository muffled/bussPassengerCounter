#include "vani_log.h"

using namespace std;

void _LogWriter(const char* szmsg,const char* szfile,const char* szfunc,const int line)
{
    ofstream out_file(".\\log.txt",ios::app);
    time_t rawtime;
    tm* timeinfo = NULL;

    time(&rawtime);
    timeinfo = localtime(&rawtime);
	char sztime[256];
	memset(sztime,0,sizeof(sztime));
	sprintf_s(sztime,sizeof(sztime),"%d-%02d-%02d %02d:%02d:%02d  ",
			  timeinfo->tm_year + 1900,timeinfo->tm_mon+1,timeinfo->tm_mday,
			  timeinfo->tm_hour,timeinfo->tm_min,timeinfo->tm_sec);

//     out_file<<endl;
// 	out_file<<sztime<<endl;
//     out_file<<"file:"<<szfile<<endl;
//     out_file<<"function:"<<szfunc<<",line:"<<line<<endl;
//     out_file<<"message:"<<szmsg<<endl;

    out_file<<sztime<<" ";
    out_file<<szmsg<<endl;
}