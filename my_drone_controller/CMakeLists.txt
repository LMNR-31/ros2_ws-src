cmake_minimum_required(VERSION 3.8)
project(my_drone_controller) # Use o nome exato do seu pacote

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 1. Dependências
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(drone_control REQUIRED)

# 2. DIRETÓRIOS DE INCLUDE (Deve vir ANTES do add_executable)
include_directories(
  include
  include/lib
  src
)

# =========================================================
# 1. EXECUTÁVEL PRINCIPAL DO DRONE
# Código dividido em módulos: command_queue, waypoint_validation,
# drone_controller_completo e main.
# =========================================================
add_executable(drone_node
  src/main.cpp
  src/command_queue.cpp
  src/waypoint_validation.cpp
  src/drone_controller_completo.cpp
  src/Drone_codegen.cpp
  src/TrajectoryPlanner_codegen.cpp
  src/rt_nonfinite.cpp
)

# Adiciona dependências para o drone_node
ament_target_dependencies(drone_node
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  mavros_msgs
  rcl_interfaces
  drone_control
)

# =========================================================
# 2. OUTROS EXECUTÁVEIS (Seus testes antigos)
# =========================================================
add_executable(offboard_takeoff src/offboard_takeoff.cpp)
ament_target_dependencies(offboard_takeoff rclcpp mavros_msgs geometry_msgs)

add_executable(integrated_controller src/integrated_controller.cpp)
ament_target_dependencies(integrated_controller rclcpp mavros_msgs geometry_msgs nav_msgs)

# =========================================================
# 3. INSTALAÇÃO (Garante que o ROS 2 ache os executáveis)
# =========================================================
install(TARGETS
  drone_node
  offboard_takeoff
  integrated_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Instala o diretório de configuração para que o arquivo drone_config.yaml
# seja acessível via 'share/<package>/config/drone_config.yaml'.
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Instala os headers públicos do pacote
install(DIRECTORY include/
  DESTINATION include
)

ament_package()