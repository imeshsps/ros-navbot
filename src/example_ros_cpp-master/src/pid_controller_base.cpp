#include "pid_controller/pid_controller_base.h"

PidControllerBase::PidControllerBase()
  : kp_(10.0),
    ki_(2.0),
    kd_(0.0),
    ui_old_(0.0),
    error_old_(0.0),
    u_max_(std::numeric_limits<double>::infinity()),
    u_min_(-std::numeric_limits<double>::infinity()),
    td_min_(std::numeric_limits<double>::min()),
    td_max_(std::numeric_limits<double>::max()),
    is_first_pass_(true)
{

}

PidControllerBase::PidControllerBase(double kp, double ki, double kd)
  : kp_(kp),
    ki_(ki),
    kd_(kd),
    ui_old_(0.0),
    error_old_(0.0),
    u_max_(std::numeric_limits<double>::infinity()),
    u_min_(-std::numeric_limits<double>::infinity()),
    td_min_(std::numeric_limits<double>::min()),
    td_max_(std::numeric_limits<double>::max()),
    is_first_pass_(true)
{

}


PidControllerBase::~PidControllerBase()
{

}

double PidControllerBase::getKp(void)
{
  return kp_;
}

double PidControllerBase::getKi(void)
{
  return ki_;
}

double PidControllerBase::getKd(void)
{
  return kd_;
}

double PidControllerBase::getUMax(void)
{
    return u_max_;
}

double PidControllerBase::getUMin(void)
{
  return u_min_;
}

double PidControllerBase::getTdMax(void)
{
    return td_max_;
}

double PidControllerBase::getTdMin(void)
{
  return td_min_;
}


void PidControllerBase::setKp(double kp)
{
  kp_ = kp;
}

void PidControllerBase::setKi(double ki)
{
  ki_ = ki;
}

void PidControllerBase::setKd(double kd)
{
  kd_ = kd;
}

void PidControllerBase::setUMax(double u_max)
{
  u_max_ = u_max;
}

void PidControllerBase::setUMin(double u_min)
{
  u_min_ = u_min;
}

void PidControllerBase::setTdMax(double td_max)
{
  td_max_ = td_max;
}

void PidControllerBase::setTdMin(double td_min)
{
  td_min_ = td_min;
}

double PidControllerBase::compute(double ref, double meas)
{
  double error, u, up, ui, ud, td;
  timespec time_current;

  if (is_first_pass_ == true)
  {
    clock_gettime(CLOCK_REALTIME, &time_old_);
    is_first_pass_ = false;
  }

  // compute algorithm
  clock_gettime(CLOCK_REALTIME, &time_current);
  td = (time_current.tv_sec - time_old_.tv_sec) +
       (time_current.tv_nsec - time_old_.tv_nsec) / 1000000000.0;

  // check if time step is too small
  if (td < td_min_)
  {
    // to avoid division by zero, set td to the minimal allowable value
    td = td_min_;
    std::cout << "Warning - time step smaller than " << td_min_ << std::endl;
  }
  else if (td > td_max_)
  {
    // just print warning
    std::cout << "Warning - time step greater than " << td_max_ << std::endl;
  }

  error = ref - meas;

  // proportional term
  up = kp_ * error;

  // integral term
  ui = ui_old_  + ki_ * error ;

  // derivative term
  ud = kd_ * (error - error_old_);

  // total = p + i + d
  u = up + ui + ud;

  // saturation and anti-wind up
  if (u > u_max_)
      u = u_max_;
  else if (u < u_min_)
      u = u_min_;
  else
  {
      // Integral value is stored for the next step only if
      // control value is not saturated(anti-wind up)
      ui_old_ = ui;
  }

  error_old_ = error;
  time_old_ = time_current;

  return u;

}
