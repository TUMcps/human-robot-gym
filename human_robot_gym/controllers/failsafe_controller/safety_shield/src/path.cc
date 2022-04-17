#include "safety_shield/path.h"


namespace safety_shield {

Path::Path():
  pos_(0),
  vel_(0.0),
  acc_(0),
  is_current_(false)
{
  for (int i = 0; i < 6; i++){
    phases_[i] = 0;
  }
}


void Path::increment(double sample_time)
{
  double jerk = 0;
  double epsilon = 0.000001;
  int i = 0;

  while (i < 3){
    if (epsilon < phases_[i]){
      jerk = phases_[i+3];
      i = 3;
    }
    i += 1;
  }

  if (vel_ > -epsilon){
    pos_ += vel_*sample_time + acc_*sample_time*sample_time/2 + jerk*sample_time*sample_time*sample_time/6;
  }
  vel_ += acc_*sample_time + jerk*sample_time*sample_time/2;
  acc_ += jerk*sample_time;

  for (i = 0; i < 3; i++){
    phases_[i] -= sample_time;
  }
}


void Path::getFinalMotion(double& final_pos, double& final_vel, double& final_acc){
  final_pos = pos_;
  final_vel = vel_;
  final_acc = acc_;
  double l_time = 0;
  for (int i = 0; i < 3; i++){
    double dt = (phases_[i]-l_time);
    final_pos += final_vel*dt + final_acc*dt*dt/2 + phases_[i+3]*dt*dt*dt/6;
    final_vel += final_acc*dt + phases_[i+3]*dt*dt/2;
    final_acc += phases_[i+3]*dt;
    l_time = phases_[i];
  }
}
}
