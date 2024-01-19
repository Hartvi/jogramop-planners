
#include "ut.h"

#include <algorithm>
#include <numeric>
#include <math.h>
#include <fstream>

void getTime(struct rusage *t){
	getrusage(RUSAGE_SELF,t);
}

double getTime(struct rusage one, struct rusage two) {

	const unsigned long as = one.ru_utime.tv_sec;
	const unsigned long bs = two.ru_utime.tv_sec;
	const unsigned long aus = one.ru_utime.tv_usec;
	const unsigned long bus = two.ru_utime.tv_usec;

	return (double)((double)bs-(double)as) + 
		(double)((double)bus-(double)aus)/1000000.0;

}


