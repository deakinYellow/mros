#ifndef COLOR_TYPE_H
#define COLOR_TYPE_H

namespace mros {

typedef struct ColorT{
  double r;
  double g;
  double b;
  double a;
}ColorT;

static ColorT Color( const double r, const double g, const double b ){
  ColorT color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = 1.0;
  return color;
};

static ColorT Color( const double r, const double g, const double b, const double a ){
  ColorT color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
};


} //end of mros

#endif // COLOR_TYPE_H
