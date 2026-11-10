// Force-included on Windows (MinGW): POSIX functions used by Kalibr that the
// Windows CRT does not provide.
#pragma once
#if defined(_WIN32)
#include <cstdlib>
// incremental_calibration/statistics/Randomizer.tpp uses random()/srandom() with RAND_MAX
inline long random() { return std::rand(); }
inline void srandom(unsigned int seed) { std::srand(seed); }
#endif
