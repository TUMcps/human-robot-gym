#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "safety_shield/safety_shield.h"

namespace py = pybind11;

PYBIND11_MODULE(safety_shield_py, handle) {
  handle.doc() = "This module contains the python bindings for the safety shield.";
  // Motion class
  py::class_<safety_shield::Motion>(handle, "Motion")
    .def(py::init<>())
    .def(py::init<int>(), py::arg("nb_modules"))
    .def(py::init<double, std::vector<double>, double>(), 
      py::arg("time"), py::arg("q"), py::arg("s") = 0.0)
    .def(py::init<double, std::vector<double>, std::vector<double>, double>(), 
      py::arg("time"), py::arg("q"), py::arg("dq"), py::arg("s") = 0.0)
    .def(py::init<double, std::vector<double>, std::vector<double>, std::vector<double>, double>(), 
      py::arg("time"), py::arg("q"), py::arg("dq"), py::arg("ddq"), py::arg("s") = 0.0)
    .def(py::init<double, std::vector<double>, std::vector<double>,std::vector<double>, std::vector<double>, double>(), 
      py::arg("time"), py::arg("q"), py::arg("dq"), py::arg("ddq"), py::arg("dddq"), py::arg("s") = 0.0)
    .def("isStopped", &safety_shield::Motion::isStopped, py::arg("threshold") = 1e-4)
    .def("hasSamePos", &safety_shield::Motion::hasSamePos, py::arg("motion"), py::arg("threshold") = 1e-4)
    .def("hasSameVel", &safety_shield::Motion::hasSameVel, py::arg("motion"), py::arg("threshold") = 1e-4)
    .def("hasSameAcc", &safety_shield::Motion::hasSameAcc, py::arg("motion"), py::arg("threshold") = 1e-4)
    .def("getTime", &safety_shield::Motion::getTime)
    .def("getS", &safety_shield::Motion::getS)
    .def("getAngle", &safety_shield::Motion::getAngle)
    .def("getVelocity", &safety_shield::Motion::getVelocity)
    .def("getAcceleration", &safety_shield::Motion::getAcceleration)
    .def("getJerk", &safety_shield::Motion::getJerk)
    .def("setTime", &safety_shield::Motion::setTime, py::arg("new_time"))
    .def("setS", &safety_shield::Motion::setS, py::arg("new_s"))
    .def("setAngle", &safety_shield::Motion::setAngle, py::arg("new_q"))
    .def("setVelocity", &safety_shield::Motion::setVelocity, py::arg("new_dq"))
    .def("setAcceleration", &safety_shield::Motion::setAcceleration, py::arg("new_ddq"))
    .def("setJerk", &safety_shield::Motion::setJerk, py::arg("new_dddq"))
    ;
  // Long-term trajectory class
  py::class_<safety_shield::LongTermTraj>(handle, "LongTermTraj")
    .def(py::init<>())
    .def(py::init<std::vector<safety_shield::Motion>, int, int>(), py::arg("long_term_traj"), py::arg("starting_index") = 0, py::arg("sliding_window_k") = 10)
    .def("setLongTermTrajectory", &safety_shield::LongTermTraj::setLongTermTrajectory, py::arg("long_term_traj"))
    .def("getLength", &safety_shield::LongTermTraj::getLength)
    .def("getCurrentPos", &safety_shield::LongTermTraj::getCurrentPos)
    .def("getCurrentMotion", &safety_shield::LongTermTraj::getCurrentMotion)
    .def("getNextMotion", &safety_shield::LongTermTraj::getNextMotion)
    .def("getNextMotionAtIndex", &safety_shield::LongTermTraj::getNextMotionAtIndex, py::arg("index"))
    .def("getTrajectoryIndex", &safety_shield::LongTermTraj::getTrajectoryIndex, py::arg("index"))
    .def("increasePosition", &safety_shield::LongTermTraj::increasePosition)
    .def("getMaxAccelerationWindow", &safety_shield::LongTermTraj::getMaxAccelerationWindow)
    .def("getMaxJerkWindow", &safety_shield::LongTermTraj::getMaxJerkWindow)
    .def("calculate_max_acc_jerk_window", &safety_shield::LongTermTraj::calculate_max_acc_jerk_window, py::arg("long_term_traj"), py::arg("k"))
    ;
  // Safety shield class
  py::class_<safety_shield::SafetyShield>(handle, "SafetyShield")
    .def(py::init<>())
    .def(py::init<bool, double, std::string, std::string, std::string, double, double, double, double, double, double, const std::vector<double>&>(),
      py::arg("activate_shield"),
      py::arg("sample_time"),
      py::arg("trajectory_config_file"),
      py::arg("robot_config_file"),
      py::arg("mocap_config_file"),
      py::arg("init_x"),
      py::arg("init_y"),
      py::arg("init_z"),
      py::arg("init_roll"),
      py::arg("init_pitch"),
      py::arg("init_yaw"),
      py::arg("init_qpos"))
    .def("step", &safety_shield::SafetyShield::step, py::arg("cycle_begin_time"))
    .def("newLongTermTrajectory", &safety_shield::SafetyShield::newLongTermTrajectory, py::arg("goal_motion"))
    .def("setLongTermTrajectory", &safety_shield::SafetyShield::setLongTermTrajectory, py::arg("traj"))
    .def("humanMeasurement", static_cast<void (safety_shield::SafetyShield::*)(const std::vector<std::vector<double>> human_measurement, double time)>(&safety_shield::SafetyShield::humanMeasurement), py::arg("human_measurement"), py::arg("time"))
    .def("getRobotReachCapsules", &safety_shield::SafetyShield::getRobotReachCapsules)
    .def("getHumanReachCapsules", &safety_shield::SafetyShield::getHumanReachCapsules, py::arg("type") = 1)
    .def("getSafety", &safety_shield::SafetyShield::getSafety)
  ;
  
}
