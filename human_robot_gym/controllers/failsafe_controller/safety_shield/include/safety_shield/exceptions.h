// -*- lsst-c++ -*/
/**
 * @file exceptions.h
 * @brief Defines custom exceptions
 * @version 0.1
 * @copyright MIT License. Please see package.xml for further detail.
 */

#include <exception>
#include <string>

#ifndef SS_EXCEPTION_H
#define SS_EXCEPTION_H

namespace safety_shield {
struct RobotMovementException : public std::exception
{
	const char * what () const throw ()
    {
    	return "Robot does not have zero velocity, acceleration, and jerk.";
    }
};

/**
 * @brief Exception that is thrown if a long-term trajectory is incorrect.
 * 
 */
class TrajectoryException: virtual public std::exception {
    
protected:
 /**
  * @brief The custom error message to display.
  */
 std::string error_message;
    
public:

 /** Constructor (C++ STL string, int, int).
  *  @param msg The error message
  */
 explicit 
 TrajectoryException(const std::string& msg) {
   error_message = "Incorrect trajctory detected: " + msg;
 }

 /** Destructor.
  *  Virtual to allow for subclassing.
  */
 virtual ~TrajectoryException() throw () {}

 /** Returns a pointer to the (constant) error description.
  *  @return A pointer to a const char*. The underlying memory
  *  is in possession of the Except object. Callers must
  *  not attempt to free the memory.
  */
 virtual const char* what() const throw () {
   return error_message.c_str();
 }
};
} // namespace safety_shield
#endif // SS_EXCEPTION_H
