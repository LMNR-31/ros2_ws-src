// main_codegen.cpp — REFATORADO
//
// Este arquivo era originalmente monolítico (>2000 linhas). O código foi
// dividido nos seguintes módulos, todos compilados como parte do executável
// `drone_node` (ver CMakeLists.txt):
//
//   Implementações:
//     src/command_queue.cpp             — CommandQueue, CommandType, CommandStatus, Command
//     src/waypoint_validation.cpp       — validate_waypoint(), validate_pose()
//     src/drone_controller_completo.cpp — DroneControllerCompleto (FSM, callbacks, helpers)
//     src/main.cpp                      — ponto de entrada: rclcpp::init / spin
//
//   Cabeçalhos públicos:
//     include/my_drone_controller/command_queue.hpp
//     include/my_drone_controller/waypoint_validation.hpp
//     include/my_drone_controller/drone_controller_completo.hpp
//
// Nenhuma implementação reside aqui — consulte os arquivos acima.
