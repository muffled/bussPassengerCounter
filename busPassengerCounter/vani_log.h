#ifndef _VANI_LOG_
#define _VANI_LOG_

#include <fstream>
#include <ctime>
#include <string>


#define LogWriter(msg) _LogWriter(msg,__FILE__,__FUNCTION__,__LINE__)

void _LogWriter(const char* szmsg,const char* szfile,const char* szfunc,const int line);

#endif  // _VANI_LOG_
