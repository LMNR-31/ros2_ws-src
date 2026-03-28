#ifndef MY_DRONE_CONTROLLER__COMMAND_QUEUE_HPP_
#define MY_DRONE_CONTROLLER__COMMAND_QUEUE_HPP_

#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace drone_control {

// ============================================================
// SISTEMA DE FILA DE COMANDOS COM RASTREABILIDADE
// ============================================================

/// @brief Type of drone command being tracked.
enum class CommandType {
  ARM,
  DISARM,
  SET_MODE_OFFBOARD,
  TAKEOFF,
  HOVER,
  TRAJECTORY,
  LAND
};

/// @brief Lifecycle status of a command.
enum class CommandStatus {
  PENDING,    ///< Command sent, awaiting FCU confirmation.
  CONFIRMED,  ///< FCU confirmed successful execution.
  FAILED,     ///< FCU reported failure.
  TIMEOUT     ///< No confirmation received within the timeout window.
};

/// @brief Represents a single drone command with full audit metadata.
struct Command {
  uint64_t id{0};
  CommandType type{CommandType::ARM};
  CommandStatus status{CommandStatus::PENDING};
  std::chrono::system_clock::time_point timestamp{};
  std::chrono::system_clock::time_point confirm_time{};
  std::map<std::string, std::string> data;

  /// @brief Human-readable command type name.
  std::string type_str() const
  {
    switch (type) {
      case CommandType::ARM:               return "ARM";
      case CommandType::DISARM:            return "DISARM";
      case CommandType::SET_MODE_OFFBOARD: return "SET_MODE_OFFBOARD";
      case CommandType::TAKEOFF:           return "TAKEOFF";
      case CommandType::HOVER:             return "HOVER";
      case CommandType::TRAJECTORY:        return "TRAJECTORY";
      case CommandType::LAND:              return "LAND";
      default:                             return "DESCONHECIDO";
    }
  }

  /// @brief Human-readable command status name.
  std::string status_str() const
  {
    switch (status) {
      case CommandStatus::PENDING:   return "PENDENTE";
      case CommandStatus::CONFIRMED: return "CONFIRMADO";
      case CommandStatus::FAILED:    return "FALHO";
      case CommandStatus::TIMEOUT:   return "TIMEOUT";
      default:                       return "DESCONHECIDO";
    }
  }
};

/**
 * @brief Thread-safe queue that tracks drone commands with full history.
 *
 * Commands are enqueued with a unique ID.  Callers later confirm
 * (success/failure) the command via that ID.  A periodic timeout check
 * moves stale pending commands to TIMEOUT status.  The full history can be
 * persisted to a structured log file.
 */
class CommandQueue
{
public:
  CommandQueue();

  /// @brief Destructor — clears all pending commands and history safely.
  ~CommandQueue();

  /// @brief Enqueue a new command and return its unique ID.
  uint64_t enqueue(CommandType type,
                   const std::map<std::string, std::string> & data = {});

  /// @brief Confirm or mark as failed a pending command.
  /// @return true if the command was found and updated.
  bool confirm(uint64_t id, bool success = true);

  /**
   * @brief Check for timed-out pending commands and move them to TIMEOUT.
   *
   * The entire operation is performed under the queue mutex so that history_
   * and pending_ are always modified together atomically.
   *
   * @return IDs of commands that expired.
   */
  std::vector<uint64_t> check_timeouts(double timeout_seconds);

  /// @brief Return a snapshot copy of the command history (thread-safe).
  std::vector<Command> get_history() const;

  /// @brief Number of pending (unconfirmed) commands.
  size_t pending_count() const;

  /// @brief Persist the full command history to a structured log file.
  void save_log(const std::string & filename) const;

  /**
   * @brief Cancel all pending commands, moving them to FAILED in the history.
   *
   * Called during reset_after_landing() to flush stale pending entries left
   * from the previous flight cycle (e.g. trajectory that was interrupted mid-
   * flight). Without this cleanup, check_timeouts() would later mark those
   * entries as TIMEOUT, producing confusing log noise and (in theory) a spurious
   * timeout callback for a command ID that the controller no longer tracks.
   *
   * Implemented as "mark FAILED + move to history" so that the full audit trail
   * is preserved in the log file while the pending_ map starts clean.
   */
  void cancel_all_pending();

private:
  mutable std::mutex mutex_;
  uint64_t next_id_{1};
  std::map<uint64_t, Command> pending_;
  std::vector<Command> history_;
};

}  // namespace drone_control

#endif  // MY_DRONE_CONTROLLER__COMMAND_QUEUE_HPP_
