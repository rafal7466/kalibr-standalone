// Compatibility shim: <boost/detail/endian.hpp> was removed in Boost 1.73.
// Kalibr's portable_binary_archive still includes it.
#pragma once
#include <boost/predef/other/endian.h>
#if BOOST_ENDIAN_LITTLE_BYTE && !defined(BOOST_LITTLE_ENDIAN)
#define BOOST_LITTLE_ENDIAN
#elif BOOST_ENDIAN_BIG_BYTE && !defined(BOOST_BIG_ENDIAN)
#define BOOST_BIG_ENDIAN
#endif
