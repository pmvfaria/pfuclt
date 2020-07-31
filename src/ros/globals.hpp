#ifndef PFUCLT_ROS_GLOBALS_HPP
#define PFUCLT_ROS_GLOBALS_HPP

namespace pfuclt::debug {
  class Error;
}


namespace globals {

extern bool debug;
extern bool publish;
extern pfuclt::debug::Error* error;

} // namespace globals

#endif // PFUCLT_ROS_GLOBALS_HPP
