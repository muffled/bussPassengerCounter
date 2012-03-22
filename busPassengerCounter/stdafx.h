#pragma once

#include "opencv2.1/include/cv.h"
#include "opencv2.1/include/cxcore.h"
#include "opencv2.1/include/highgui.h"


#ifdef _DEBUG
#pragma comment(lib,"opencv2.1/lib/cv210d.lib")
#pragma comment(lib,"opencv2.1/lib/cxcore210d.lib")
#pragma comment(lib,"opencv2.1/lib/highgui210d.lib")
#else
#pragma comment(lib,"opencv2.1/lib/cv210.lib")
#pragma comment(lib,"opencv2.1/lib/cxcore210.lib")
#pragma comment(lib,"opencv2.1/lib/highgui210.lib")
#endif

#include <stack>
#include <vector>
#include <list>
#include <set>
#include <string>
#include <iostream>


typedef unsigned long ulong;