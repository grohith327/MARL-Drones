#include "utils.h"

#include <math.h>
#include <stdlib.h>
#include <sys/time.h>

double GetTime() {
  struct timeval time;
  gettimeofday(&time, NULL);
  return (double)time.tv_sec + ((double)time.tv_usec)/1000000.0;
}

void SeedRand() {
  struct timeval time;
  gettimeofday(&time, NULL);
  srand(time.tv_usec);
}

float NormalizeAngle(float a) {
  while (a > (float)M_PI) a-=2.0f*(float)M_PI;
  while (a < (float)(-M_PI)) a+=2.0f*(float)M_PI;
  return a;
}

double RandomUniform() {
  return (double)rand()/(double)RAND_MAX;
  // retur  n (double)0.5;
}

double RandomNormal() {
  double x1, x2, w;
  do {
    x1 = 2.0 * RandomUniform() - 1.0;
    x2 = 2.0 * RandomUniform() - 1.0;
    w = x1*x1 + x2*x2;
  } while (w >= 1.0);
  w = sqrt((-2.0 * log(w))/w);
  return(x1*w);
}

RandomColors::RandomColors() {
  h_ = RandomUniform();
}

RandomColors::RandomColors(float init_value) {
  h_ = init_value;
}

std::tuple<float, float, float> RandomColors::Next() {
  static float s = 0.85;
  static float v = 0.95;
  static float golden_ratio_conjugate = 0.618033988749895f;

  float r, g, b;
  h_ += golden_ratio_conjugate;
  h_ -= (float)((int)h_);
  int hi = (int)(h_*6.0);
  float f = h_*6.0 - (float)hi;
  float p = v*(1.0 - s);
  float q = v*(1.0 - f*s);
  float t = v*(1.0 - (1.0 - f)*s);
  switch (hi) {
  case 0:
    r = v;
    g = t;
    b = p;
    break;
  case 1:
    r = q;
    g = v;
    b = p;
    break;
  case 2:
    r = p;
    g = v;
    b = t;
    break;
  case 3:
    r = p;
    g = q;
    b = v;
    break;
  case 4:
    r = t;
    g = p;
    b = v;
    break;
  default:
    r = v;
    g = p;
    b = q;
  }
  return std::tuple<float, float, float>(r, g, b);
}
