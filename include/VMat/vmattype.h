#ifndef _VMAT_H_
#define _VMAT_H_

#include <limits>

namespace vm
{
	using Float = float;
	constexpr Float Pi = 3.14159265358979323846;
	constexpr Float LOWEST_FLOAT = (std::numeric_limits<Float>::lowest)();
	constexpr Float MAX_VALUE = (std::numeric_limits<Float>::max)();		// For fucking min/max macro defined in windows.h
}


#endif /*_VMAT_H_*/
