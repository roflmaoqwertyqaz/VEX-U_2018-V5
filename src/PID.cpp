#include "PID.h"

PID::PID() :
    p_val(0),
    i_val(0),
    d_val(0),
    _min(-1E100),
    _max(1E100),
    _i_min(-1E100),
    _i_max(1E100),
    i_term(0),
    initialized(false),
    lastinput(0)
{
    lasttime = pros::millis();
};

PID::PID(double p, double i, double d) :
    p_val(p),
    i_val(i),
    d_val(d),
    _min(-1E100),
    _max(1E100),
    _i_min(-1E100),
    _i_max(1E100),
    i_term(0),
    initialized(false),
    lastinput(0)
{
    lasttime = pros::millis();
};

PID::PID(double p, double i, double d, double min, double max, double i_min, double i_max, double i_offset) :
    p_val(p),
    i_val(i),
    d_val(d),
    _min(min),
    _max(max),
    _i_min(i_min),
    _i_max(i_max),
    i_term(i_offset),
    initialized(false),
    lastinput(0)
{
    lasttime = pros::millis();
};

void PID::compute(double target, double input)
{
    uint32_t currtime = pros::millis();
    double dt = (double)(currtime-lasttime)/1000.0;
    if(dt > 0.01) {
      lasttime = currtime;

      double error = target-input;

      double p_term = error*p_val;
      i_term += error*i_val*dt;
      if(i_term < _i_min) i_term = _i_min;
      else if(i_term > _i_max) i_term = _i_max;

      double d_term;
      if(initialized)
      {
          d_term = (lastinput-input)/dt*d_val;
      }
      else
      {
          d_term = 0;
          initialized = true;
      }
      lastinput = input;

      output = p_term+i_term;
      if(output < _min) output = _min;
      else if(output > _max) output = _max;
    }
};

double PID::getOutput()
{
    return output;
};

void PID::setConstants(double p, double i, double d)
{
    p_val = p;
    i_val = i;
    d_val = d;
};

void PID::setBounds(double min, double max)
{
    _min = min;
    _max = max;
};

void PID::setIBounds(double i_min, double i_max)
{
    _i_min = i_min;
    _i_max = i_max;
};
