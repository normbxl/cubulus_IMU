#ifndef IIRfilter_h
#define IIRfilter_h


class IIRfilter {
  private:
    float a[2];
    float b[2];
    float x[2];
    float y[2];
  public:
    void init(float bCoef[], float aCoef[]);
    float step(float input);
    IIRfilter(float bCoef[], float aCoef[]);
};
#endif
