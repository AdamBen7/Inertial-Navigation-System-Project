//Created in order to calculate trig in degrees 
#include <cmath>
#ifndef TRIGD_H
#define TRIGD_H

double sind(double x){
  return sin(x*M_PI/180.0);
}

double cosd(double x){
  return cos(x*M_PI/180.0);
}

double tand(double x){
  return tan(x*M_PI/180.0);
}
#endif

