#ifndef LODEPNG_H_
#define LODEPNG_H_

#include <cstdio>
#include <string>

// Minimal stub for lodepng used by simulate. Screenshot saving is disabled.
// If you want PNG support, replace this file with the full lodepng distribution.

// Match the color type used by simulate.cc.
#define LCT_RGB 2

namespace lodepng {
inline unsigned encode(const std::string& filename, const unsigned char*,
                       unsigned, unsigned, int) {
  std::fprintf(stderr, "lodepng stub: screenshot disabled (file: %s)\n",
               filename.c_str());
  return 1;
}
}  // namespace lodepng

#endif  // LODEPNG_H_
