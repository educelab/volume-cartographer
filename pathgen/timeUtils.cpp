// timeUtils.cpp
#include "timeUtils.h"

#include <time.h>

const std::string currentDateTime( void )
{
	time_t now = time( 0 );
	struct tm tstruct;
	char buf[ 80 ];
	tstruct = *localtime( &now );
	// Visit https://en.cpreference.com/w/cpp/chrono/c/strftime
	// for more information about data/time format
	strftime( buf, sizeof( buf ), "%Y%m%d%H%M%S", &tstruct );

	return buf;
}
