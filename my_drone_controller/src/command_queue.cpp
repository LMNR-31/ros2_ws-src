#include "my_drone_controller/command_queue.hpp"

#include <iostream>

namespace drone_control {

// ── Constructor / Destructor ─────────────────────────────────────────────────

CommandQueue::CommandQueue() : next_id_(1) {}

CommandQueue::~CommandQueue()
{
  std::lock_guard<std::mutex> lock(mutex_);
  pending_.clear();
  history_.clear();
}

// ── enqueue ──────────────────────────────────────────────────────────────────

uint64_t CommandQueue::enqueue(CommandType type,
                               const std::map<std::string, std::string> & data)
{
  std::lock_guard<std::mutex> lock(mutex_);
  Command cmd;
  cmd.id = next_id_++;
  cmd.type = type;
  cmd.status = CommandStatus::PENDING;
  cmd.timestamp = std::chrono::system_clock::now();
  cmd.data = data;
  pending_[cmd.id] = cmd;
  history_.push_back(cmd);
  return cmd.id;
}

// ── confirm ──────────────────────────────────────────────────────────────────

bool CommandQueue::confirm(uint64_t id, bool success)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = pending_.find(id);
  if (it == pending_.end()) {
    return false;
  }
  CommandStatus new_status = success ? CommandStatus::CONFIRMED : CommandStatus::FAILED;
  it->second.status = new_status;
  it->second.confirm_time = std::chrono::system_clock::now();
  for (auto & h : history_) {
    if (h.id == id) {
      h.status = new_status;
      h.confirm_time = it->second.confirm_time;
      break;
    }
  }
  pending_.erase(it);
  return true;
}

// ── check_timeouts ───────────────────────────────────────────────────────────

std::vector<uint64_t> CommandQueue::check_timeouts(double timeout_seconds)
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<uint64_t> timed_out;
  auto now = std::chrono::system_clock::now();
  for (auto it = pending_.begin(); it != pending_.end(); ) {
    double elapsed =
      std::chrono::duration<double>(now - it->second.timestamp).count();
    if (elapsed > timeout_seconds) {
      it->second.status = CommandStatus::TIMEOUT;
      // Update the history entry under the same lock to avoid data races
      // between check_timeouts() and save_log().
      for (auto & h : history_) {
        if (h.id == it->second.id) {
          h.status = CommandStatus::TIMEOUT;
          break;
        }
      }
      timed_out.push_back(it->second.id);
      it = pending_.erase(it);
    } else {
      ++it;
    }
  }
  return timed_out;
}

// ── get_history ──────────────────────────────────────────────────────────────

std::vector<Command> CommandQueue::get_history() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return history_;
}

// ── pending_count ────────────────────────────────────────────────────────────

size_t CommandQueue::pending_count() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return pending_.size();
}

// ── cancel_all_pending ───────────────────────────────────────────────────────

void CommandQueue::cancel_all_pending()
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto now = std::chrono::system_clock::now();
  for (auto & [id, cmd] : pending_) {
    cmd.status = CommandStatus::FAILED;
    cmd.confirm_time = now;
    // Update the matching history entry so the log file stays consistent.
    for (auto & h : history_) {
      if (h.id == id) {
        h.status = CommandStatus::FAILED;
        h.confirm_time = now;
        break;
      }
    }
  }
  pending_.clear();
}

// ── save_log ─────────────────────────────────────────────────────────────────

void CommandQueue::save_log(const std::string & filename) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::ofstream file(filename);
  if (!file.is_open()) {
    std::cerr << "[CommandQueue] ERRO: Nao foi possivel abrir arquivo de log: "
              << filename << "\n";
    return;
  }
  file << "=== HISTORICO DE COMANDOS DO DRONE ===\n";
  file << "Total: " << history_.size() << " comandos\n\n";
  for (const auto & cmd : history_) {
    std::time_t t = std::chrono::system_clock::to_time_t(cmd.timestamp);
    std::tm tm_buf{};
#ifdef _WIN32
    localtime_s(&tm_buf, &t);
#else
    localtime_r(&t, &tm_buf);
#endif
    file << std::put_time(&tm_buf, "[%Y-%m-%d %H:%M:%S]")
         << " ID=" << std::setw(4) << std::right << cmd.id
         << " | TIPO="   << std::setw(18) << std::left << cmd.type_str()
         << " | STATUS=" << std::setw(10) << std::left << cmd.status_str();
    if (!cmd.data.empty()) {
      file << " | DADOS={";
      bool first = true;
      for (const auto & kv : cmd.data) {
        if (!first) { file << ", "; }
        file << kv.first << "=" << kv.second;
        first = false;
      }
      file << "}";
    }
    if (cmd.status == CommandStatus::CONFIRMED ||
        cmd.status == CommandStatus::FAILED) {
      double elapsed = std::chrono::duration<double>(
        cmd.confirm_time - cmd.timestamp).count();
      file << " | TEMPO=" << std::fixed << std::setprecision(2) << elapsed << "s";
    }
    file << "\n";
  }
  file.close();
}

}  // namespace drone_control
