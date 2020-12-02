#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

#include "./pi3hat/lib/cpp/mjbots/moteus/moteus_protocol.h"

namespace py = pybind11;

using namespace mjbots::moteus;

#define ENUM_VALUE(type_name, field_name) .value(#field_name, type_name::field_name)
#define RW(type_name, field_name) .def_readwrite(#field_name, &type_name::field_name)

PYBIND11_MODULE(moteus_protocol, m) {

  py::enum_<Multiplex>(m, "Multiplex")
      ENUM_VALUE(Multiplex, kWriteBase)
      ENUM_VALUE(Multiplex, kWriteInt8)
      ENUM_VALUE(Multiplex, kWriteInt16)
      ENUM_VALUE(Multiplex, kWriteInt32)
      ENUM_VALUE(Multiplex, kWriteFloat)

      ENUM_VALUE(Multiplex, kReadBase)
      ENUM_VALUE(Multiplex, kReadInt8)
      ENUM_VALUE(Multiplex, kReadInt16)
      ENUM_VALUE(Multiplex, kReadInt32)
      ENUM_VALUE(Multiplex, kReadFloat)

      ENUM_VALUE(Multiplex, kReplyBase)
      ENUM_VALUE(Multiplex, kReplyInt8)
      ENUM_VALUE(Multiplex, kReplyInt16)
      ENUM_VALUE(Multiplex, kReplyInt32)
      ENUM_VALUE(Multiplex, kReplyFloat)

      ENUM_VALUE(Multiplex, kWriteError)
      ENUM_VALUE(Multiplex, kReadError)

      // # Tunneled Stream #
      ENUM_VALUE(Multiplex, kClientToServer)
      ENUM_VALUE(Multiplex, kServerToClient)
      ENUM_VALUE(Multiplex, kClientPollServer)

      ENUM_VALUE(Multiplex, kNop);

  py::enum_<Resolution>(m, "Resolution")
      ENUM_VALUE(Resolution, kInt8)
      ENUM_VALUE(Resolution, kInt16)
      ENUM_VALUE(Resolution, kInt32)
      ENUM_VALUE(Resolution, kFloat)
      ENUM_VALUE(Resolution, kIgnore);

  py::enum_<Mode>(m, "Mode")
      ENUM_VALUE(Mode, kStopped)
      ENUM_VALUE(Mode, kFault)
      ENUM_VALUE(Mode, kEnabling)
      ENUM_VALUE(Mode, kCalibrating)
      ENUM_VALUE(Mode, kCalibrationComplete)
      ENUM_VALUE(Mode, kPwm)
      ENUM_VALUE(Mode, kVoltage)
      ENUM_VALUE(Mode, kVoltageFoc)
      ENUM_VALUE(Mode, kVoltageDq)
      ENUM_VALUE(Mode, kCurrent)
      ENUM_VALUE(Mode, kPosition)
      ENUM_VALUE(Mode, kPositionTimeout)
      ENUM_VALUE(Mode, kZeroVelocity)
      ENUM_VALUE(Mode, kNumModes);

  py::class_<CanFrame>(m, "CanFrame")
      .def(py::init<>())
      .def_property("data", [](CanFrame &p) -> pybind11::array_t<uint8_t> {
        auto base = pybind11::array_t<uint8_t>({64}, {sizeof(uint8_t)});
        return pybind11::array_t<uint8_t>({64}, {sizeof(uint8_t)}, &p.data[0], base);
      }, [](CanFrame &p) {})
      .def_readwrite("size", &CanFrame::size);

  py::class_<WriteCanFrame>(m, "WriteCanFrame")
      .def(py::init<CanFrame *>());

  py::class_<PositionCommand>(m, "PositionCommand")
      .def(py::init<>())
          RW(PositionCommand, position)
          RW(PositionCommand, velocity)
          RW(PositionCommand, feedforward_torque)
          RW(PositionCommand, kp_scale)
          RW(PositionCommand, kd_scale)
          RW(PositionCommand, maximum_torque)
          RW(PositionCommand, stop_position)
          RW(PositionCommand, watchdog_timeout);

  py::class_<PositionResolution>(m, "PositionResolution")
      .def(py::init<>())
          RW(PositionResolution, position)
          RW(PositionResolution, velocity)
          RW(PositionResolution, feedforward_torque)
          RW(PositionResolution, kp_scale)
          RW(PositionResolution, kd_scale)
          RW(PositionResolution, maximum_torque)
          RW(PositionResolution, stop_position)
          RW(PositionResolution, watchdog_timeout);

  m.def("EmitPositionCommand", &EmitPositionCommand);

  py::class_<QueryCommand>(m, "QueryCommand")
      .def(py::init<>())
          RW(QueryCommand, mode)
          RW(QueryCommand, position)
          RW(QueryCommand, velocity)
          RW(QueryCommand, torque)
          RW(QueryCommand, q_current)
          RW(QueryCommand, d_current)
          RW(QueryCommand, rezero_state)
          RW(QueryCommand, voltage)
          RW(QueryCommand, temperature)
          RW(QueryCommand, fault)
      .def("any_set", &QueryCommand::any_set);

  m.def("EmitQueryCommand", &EmitQueryCommand);

  m.def("SaturateInt8", &Saturate<int8_t>);
  m.def("SaturateInt16", &Saturate<int16_t>);

  py::class_<QueryResult>(m, "QueryResult")
      .def(py::init<>())
          RW(QueryResult, mode)
          RW(QueryResult, position)
          RW(QueryResult, velocity)
          RW(QueryResult, torque)
          RW(QueryResult, q_current)
          RW(QueryResult, d_current)
          RW(QueryResult, rezero_state)
          RW(QueryResult, voltage)
          RW(QueryResult, temperature)
          RW(QueryResult, fault);

  m.def("ParseQueryResult", [](CanFrame &f) {
    return ParseQueryResult(f.data, f.size);
  });
}
