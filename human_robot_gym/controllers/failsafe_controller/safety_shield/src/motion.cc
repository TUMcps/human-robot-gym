#include "safety_shield/motion.h"

namespace safety_shield {

Motion::Motion(int nb_modules):
    nb_modules_(nb_modules),
    time_(0)
{
  for (int i = 0; i < nb_modules_; i++){
    q_.push_back(0.0);
    dq_.push_back(0.0);
    ddq_.push_back(0.0);
    dddq_.push_back(0.0);
  }
}

Motion::Motion(double time, const std::vector<double> &q, double s):
    nb_modules_(q.size()),
    time_(time),
    q_(q),
    s_(s)
{
  for (int i = 0; i < nb_modules_; i++){
    dq_.push_back(0.0);
    ddq_.push_back(0.0);
    dddq_.push_back(0.0);
  }
}

Motion::Motion(double time, const std::vector<double> &q, const std::vector<double> &dq, double s):
    nb_modules_(q.size()),
    time_(time),
    q_(q),
    dq_(dq),
    s_(s)
{
  for (int i = 0; i < nb_modules_; i++){
    ddq_.push_back(0.0);
    dddq_.push_back(0.0);
  }
}

Motion::Motion(double time, const std::vector<double> &q, const std::vector<double> &dq, 
    const std::vector<double> &ddq, double s):
    nb_modules_(q.size()),
    time_(time),
    q_(q),
    dq_(dq),
    ddq_(ddq),
    s_(s)
{
  for (int i = 0; i < nb_modules_; i++){
    dddq_.push_back(0.0);
  }
}

Motion::Motion(double time, const std::vector<double> &q, const std::vector<double> &dq, 
    const std::vector<double> &ddq, const std::vector<double> &dddq, double s):
    nb_modules_(q.size()),
    time_(time),
    q_(q),
    dq_(dq),
    ddq_(ddq),
    dddq_(dddq),
    s_(s)
{}

} // namespace safety_shield