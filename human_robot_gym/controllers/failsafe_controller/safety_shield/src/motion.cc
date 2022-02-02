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

Motion::Motion(double time, const std::vector<double> &q):
    nb_modules_(q.size()),
    time_(time),
    q_(q)
{
  for (int i = 0; i < nb_modules_; i++){
    dq_.push_back(0.0);
    ddq_.push_back(0.0);
    dddq_.push_back(0.0);
  }
}

Motion::Motion(double time, const std::vector<double> &q, const std::vector<double> &dq):
    nb_modules_(q.size()),
    time_(time),
    q_(q),
    dq_(dq)
{
  for (int i = 0; i < nb_modules_; i++){
    ddq_.push_back(0.0);
    dddq_.push_back(0.0);
  }
}

Motion::Motion(double time, const std::vector<double> &q, const std::vector<double> &dq, 
    const std::vector<double> &ddq):
    nb_modules_(q.size()),
    time_(time),
    q_(q),
    dq_(dq),
    ddq_(ddq)
{
  for (int i = 0; i < nb_modules_; i++){
    dddq_.push_back(0.0);
  }
}

Motion::Motion(double time, const std::vector<double> &q, const std::vector<double> &dq, 
    const std::vector<double> &ddq, const std::vector<double> &dddq):
    nb_modules_(q.size()),
    time_(time),
    q_(q),
    dq_(dq),
    ddq_(ddq),
    dddq_(dddq)
{}

} // namespace safety_shield