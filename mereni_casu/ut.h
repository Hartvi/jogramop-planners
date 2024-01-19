#ifndef _RUT_H__
#define _RUT_H__

#include <iostream>
#include <vector>
#include <list>
#include <string>
#include <ostream>
#include <sstream>
#include <algorithm>
#include <numeric>
#include <math.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/resource.h>
#include <typeinfo>

double getTime(struct rusage one, struct rusage two);
void getTime(struct rusage *t);


#endif

