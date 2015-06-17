#include "IIRfilter.h"

void IIRfilter::init(float bCoef[], float aCoef[]) {
  a[0] = aCoef[0];
  a[1] = aCoef[1];
  b[0] = bCoef[0];
  b[1] = bCoef[1];
  for (int i = 0; i < 2; i++)
    b[i] = bCoef[i] / aCoef[0];  // normalise numerator coefficents
  for (int i = 0; i < 2; i++)
   a[i] = aCoef[i] / aCoef[0];  // normalise denominator coefficents 
}

IIRfilter::IIRfilter(float bCoef[], float aCoef[]) {
  init(bCoef, aCoef);
}

float IIRfilter::step(float input) {
  float output = 0.0f;                      // initialise running sum output
  x[1] = x[0];
  output += b[1] * x[1];
  x[0] = input;
  output += b[0] * x[0];
  y[1] = y[0];
  output -= a[1] * y[1];
  y[0] = output;
  return output;
}
