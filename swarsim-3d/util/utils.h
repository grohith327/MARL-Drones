#ifndef _UTILS_H
#define _UTILS_H

#include <tuple>

double GetTime();
void SeedRand();

float NormalizeAngle(float a);

double RandomUniform();
double RandomNormal();

class RandomColors {
 public:
  RandomColors();
  explicit RandomColors(float init_value);
  std::tuple<float, float, float> Next();
 private:
  float h_;
};

#endif
