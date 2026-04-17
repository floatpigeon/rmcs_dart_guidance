#include "manager/manager_types.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <deque>
#include <iomanip>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_dart_guidance::manager {
namespace {

constexpr int kProtocolVersion = 1;
constexpr int64_t kDefaultStatePushIntervalMs = 200;
constexpr uint16_t kDefaultListenPort = 37601;
constexpr int64_t kDefaultHelloTimeoutMs = 3000;
constexpr int64_t kDefaultHeartbeatIntervalMs = 1000;
constexpr size_t kDefaultMaxMessageBytes = 64 * 1024;
constexpr std::string_view kServerName = "rmcs-dart-bridge";
constexpr std::string_view kServerVersion = "0.1.0";

int64_t current_time_ms() {
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
}

void close_fd(int& fd) {
    if (fd >= 0) {
        close(fd);
        fd = -1;
    }
}

bool set_non_blocking(int fd, std::string& error) {
    const int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0) {
        error = std::string("fcntl(F_GETFL) failed: ") + std::strerror(errno);
        return false;
    }

    if (fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
        error = std::string("fcntl(F_SETFL) failed: ") + std::strerror(errno);
        return false;
    }

    return true;
}

std::string escape_json(std::string_view value) {
    std::string escaped;
    escaped.reserve(value.size() + 8);

    for (const char ch : value) {
        switch (ch) {
        case '"': escaped += "\\\""; break;
        case '\\': escaped += "\\\\"; break;
        case '\b': escaped += "\\b"; break;
        case '\f': escaped += "\\f"; break;
        case '\n': escaped += "\\n"; break;
        case '\r': escaped += "\\r"; break;
        case '\t': escaped += "\\t"; break;
        default:
            if (static_cast<unsigned char>(ch) < 0x20) {
                std::ostringstream stream;
                stream << "\\u" << std::uppercase << std::hex << std::setw(4)
                       << std::setfill('0') << static_cast<int>(static_cast<unsigned char>(ch));
                escaped += stream.str();
            } else {
                escaped.push_back(ch);
            }
            break;
        }
    }

    return escaped;
}

std::string quoted(std::string_view value) { return "\"" + escape_json(value) + "\""; }

template <typename Number>
std::string number_json(Number value) {
    std::ostringstream stream;
    stream << value;
    return stream.str();
}

template <typename Number>
std::string optional_number_json(const std::optional<Number>& value) {
    return value.has_value() ? number_json(*value) : "null";
}

const char* failure_message(ActionFailureReason reason) {
    switch (reason) {
    case ActionFailureReason::TIMEOUT: return "动作执行超时";
    case ActionFailureReason::STALL: return "机构堵转";
    case ActionFailureReason::EXTERNAL_CANCEL: return "任务被外部取消";
    case ActionFailureReason::DEPENDENCY_FAILURE: return "依赖动作失败";
    case ActionFailureReason::NONE: return "";
    }
    return "未知错误";
}

enum class JsonType { Null, Bool, Number, String, Object, Array };

struct JsonValue {
    JsonType type{JsonType::Null};
    bool bool_value{false};
    int64_t number_value{0};
    std::string string_value;
    std::map<std::string, JsonValue> object_value;
    std::vector<JsonValue> array_value;
};

class JsonParser {
public:
    explicit JsonParser(std::string_view input)
        : input_(input) {}

    bool parse(JsonValue& value, std::string& error) {
        skip_ws();
        if (!parse_value(value, error)) {
            return false;
        }

        skip_ws();
        if (position_ != input_.size()) {
            error = "unexpected trailing characters";
            return false;
        }

        return true;
    }

private:
    bool parse_value(JsonValue& value, std::string& error) {
        skip_ws();
        if (position_ >= input_.size()) {
            error = "unexpected end of input";
            return false;
        }

        switch (input_[position_]) {
        case '{': return parse_object(value, error);
        case '[': return parse_array(value, error);
        case '"': {
            value.type = JsonType::String;
            return parse_string(value.string_value, error);
        }
        case 't': return parse_true(value, error);
        case 'f': return parse_false(value, error);
        case 'n': return parse_null(value, error);
        default: return parse_number(value, error);
        }
    }

    bool parse_object(JsonValue& value, std::string& error) {
        value = JsonValue{};
        value.type = JsonType::Object;

        ++position_;
        skip_ws();
        if (consume('}')) {
            return true;
        }

        while (true) {
            std::string key;
            if (!parse_string(key, error)) {
                return false;
            }

            skip_ws();
            if (!consume(':')) {
                error = "expected ':' after object key";
                return false;
            }

            JsonValue member;
            if (!parse_value(member, error)) {
                return false;
            }

            value.object_value.emplace(std::move(key), std::move(member));

            skip_ws();
            if (consume('}')) {
                return true;
            }

            if (!consume(',')) {
                error = "expected ',' or '}' in object";
                return false;
            }
        }
    }

    bool parse_array(JsonValue& value, std::string& error) {
        value = JsonValue{};
        value.type = JsonType::Array;

        ++position_;
        skip_ws();
        if (consume(']')) {
            return true;
        }

        while (true) {
            JsonValue element;
            if (!parse_value(element, error)) {
                return false;
            }

            value.array_value.emplace_back(std::move(element));

            skip_ws();
            if (consume(']')) {
                return true;
            }

            if (!consume(',')) {
                error = "expected ',' or ']' in array";
                return false;
            }
        }
    }

    bool parse_string(std::string& output, std::string& error) {
        if (!consume('"')) {
            error = "expected string";
            return false;
        }

        output.clear();
        while (position_ < input_.size()) {
            const char ch = input_[position_++];
            if (ch == '"') {
                return true;
            }

            if (ch != '\\') {
                output.push_back(ch);
                continue;
            }

            if (position_ >= input_.size()) {
                error = "unterminated escape sequence";
                return false;
            }

            const char escaped = input_[position_++];
            switch (escaped) {
            case '"': output.push_back('"'); break;
            case '\\': output.push_back('\\'); break;
            case '/': output.push_back('/'); break;
            case 'b': output.push_back('\b'); break;
            case 'f': output.push_back('\f'); break;
            case 'n': output.push_back('\n'); break;
            case 'r': output.push_back('\r'); break;
            case 't': output.push_back('\t'); break;
            case 'u':
                error = "unicode escapes are not supported";
                return false;
            default:
                error = "invalid escape sequence";
                return false;
            }
        }

        error = "unterminated string";
        return false;
    }

    bool parse_true(JsonValue& value, std::string& error) {
        if (!consume_literal("true")) {
            error = "invalid literal";
            return false;
        }

        value = JsonValue{};
        value.type = JsonType::Bool;
        value.bool_value = true;
        return true;
    }

    bool parse_false(JsonValue& value, std::string& error) {
        if (!consume_literal("false")) {
            error = "invalid literal";
            return false;
        }

        value = JsonValue{};
        value.type = JsonType::Bool;
        value.bool_value = false;
        return true;
    }

    bool parse_null(JsonValue& value, std::string& error) {
        if (!consume_literal("null")) {
            error = "invalid literal";
            return false;
        }

        value = JsonValue{};
        value.type = JsonType::Null;
        return true;
    }

    bool parse_number(JsonValue& value, std::string& error) {
        const size_t start = position_;
        if (peek() == '-') {
            ++position_;
        }

        if (position_ >= input_.size() || !std::isdigit(static_cast<unsigned char>(input_[position_]))) {
            error = "invalid number";
            return false;
        }

        while (position_ < input_.size()
               && std::isdigit(static_cast<unsigned char>(input_[position_]))) {
            ++position_;
        }

        if (position_ < input_.size()) {
            const char tail = input_[position_];
            if (tail == '.' || tail == 'e' || tail == 'E') {
                error = "floating point numbers are not supported";
                return false;
            }
        }

        const std::string_view token = input_.substr(start, position_ - start);
        try {
            value = JsonValue{};
            value.type = JsonType::Number;
            value.number_value = std::stoll(std::string(token));
        } catch (const std::exception&) {
            error = "number out of range";
            return false;
        }

        return true;
    }

    void skip_ws() {
        while (position_ < input_.size()
               && std::isspace(static_cast<unsigned char>(input_[position_]))) {
            ++position_;
        }
    }

    bool consume(char expected) {
        if (position_ < input_.size() && input_[position_] == expected) {
            ++position_;
            return true;
        }
        return false;
    }

    bool consume_literal(std::string_view literal) {
        if (input_.substr(position_, literal.size()) != literal) {
            return false;
        }
        position_ += literal.size();
        return true;
    }

    char peek() const { return position_ < input_.size() ? input_[position_] : '\0'; }

    std::string_view input_;
    size_t position_{0};
};

const JsonValue* find_member(const JsonValue& object, std::string_view key) {
    if (object.type != JsonType::Object) {
        return nullptr;
    }

    const auto iterator = object.object_value.find(std::string(key));
    return iterator == object.object_value.end() ? nullptr : &iterator->second;
}

bool parse_json_line(std::string_view line, JsonValue& json, std::string& error) {
    JsonParser parser(line);
    return parser.parse(json, error);
}

std::optional<std::string> string_member(const JsonValue& object, std::string_view key) {
    const JsonValue* member = find_member(object, key);
    if (member == nullptr || member->type != JsonType::String) {
        return std::nullopt;
    }
    return member->string_value;
}

std::optional<int64_t> integer_member(const JsonValue& object, std::string_view key) {
    const JsonValue* member = find_member(object, key);
    if (member == nullptr || member->type != JsonType::Number) {
        return std::nullopt;
    }
    return member->number_value;
}

const JsonValue* object_member(const JsonValue& object, std::string_view key) {
    const JsonValue* member = find_member(object, key);
    return member != nullptr && member->type == JsonType::Object ? member : nullptr;
}

const JsonValue* array_member(const JsonValue& object, std::string_view key) {
    const JsonValue* member = find_member(object, key);
    return member != nullptr && member->type == JsonType::Array ? member : nullptr;
}

std::string supported_commands_json() {
    return "[\"launch_prepare\",\"launch_cancel\",\"fire_preload\","
           "\"manual_control_enter\",\"manual_control_exit\","
           "\"belt_up\",\"belt_down\",\"yaw_left\",\"yaw_right\","
           "\"pitch_up\",\"pitch_down\",\"trigger_lock\",\"trigger_free\","
           "\"filling_lift_up\",\"filling_lift_down\",\"recover\",\"cancel\"]";
}

std::string make_envelope(
    std::string_view type, std::string_view request_id, int64_t timestamp_ms,
    std::string payload_json) {
    std::ostringstream stream;
    stream << "{\"type\":" << quoted(type) << ",\"protocol_version\":" << kProtocolVersion
           << ",\"request_id\":" << quoted(request_id)
           << ",\"timestamp_ms\":" << timestamp_ms << ",\"payload\":" << payload_json << "}\n";
    return stream.str();
}

std::string make_error_line(
    std::string_view request_id, std::string_view code, std::string_view message) {
    std::ostringstream payload;
    payload << "{\"code\":" << quoted(code) << ",\"message\":" << quoted(message)
            << ",\"details\":{}}";
    return make_envelope("error", request_id, current_time_ms(), payload.str());
}

std::string make_hello_ack_line(
    std::string_view request_id, std::string_view session_id, int64_t heartbeat_interval_ms) {
    std::ostringstream payload;
    payload << "{\"server_name\":" << quoted(kServerName)
            << ",\"server_version\":" << quoted(kServerVersion)
            << ",\"session_id\":" << quoted(session_id)
            << ",\"heartbeat_interval_ms\":" << heartbeat_interval_ms
            << ",\"state_push_interval_ms\":" << kDefaultStatePushIntervalMs
            << ",\"supported_commands\":" << supported_commands_json() << "}";
    return make_envelope("hello_ack", request_id, current_time_ms(), payload.str());
}

std::string make_heartbeat_line(std::string_view session_id) {
    std::ostringstream payload;
    payload << "{\"session_id\":" << quoted(session_id) << "}";
    return make_envelope("heartbeat", "", current_time_ms(), payload.str());
}

std::string make_echo_ack_line(std::string_view request_id, std::string_view message) {
    std::ostringstream payload;
    payload << "{\"message\":" << quoted(message) << "}";
    return make_envelope("echo_ack", request_id, current_time_ms(), payload.str());
}

std::string make_command_ack_line(std::string_view request_id, std::string_view command_name) {
    std::ostringstream payload;
    payload << "{\"accepted\":true,\"command_name\":" << quoted(command_name) << "}";
    return make_envelope("command_ack", request_id, current_time_ms(), payload.str());
}

struct ManagerSnapshot {
    std::string lifecycle_state;
    std::string current_task;
    std::string current_action;
    uint32_t fire_count{0};
    bool manual_control_active{false};
    std::string host_last_command;
    std::string host_last_command_status;
    std::optional<int64_t> host_last_command_timestamp_ms;
    std::optional<double> pitch_deg;
    std::optional<int32_t> force_channel_1;
    std::optional<int32_t> force_channel_2;
    std::vector<ManagerQueuedTaskInfo> queue;
    std::optional<ManagerLastErrorInfo> last_error;

    friend bool operator==(const ManagerSnapshot&, const ManagerSnapshot&) = default;
};

std::string make_queue_json(const std::vector<ManagerQueuedTaskInfo>& queue) {
    std::ostringstream payload;
    payload << "[";

    bool first_item = true;
    for (const auto& item : queue) {
        if (!first_item) {
            payload << ",";
        }
        first_item = false;

        payload << "{"
                << "\"task_name\":" << quoted(item.task_name) << ","
                << "\"display_name\":" << quoted(item.display_name) << ","
                << "\"status\":\"queued\""
                << "}";
    }

    payload << "]";
    return payload.str();
}

std::string make_last_error_json(const std::optional<ManagerLastErrorInfo>& last_error) {
    if (!last_error.has_value()) {
        return "null";
    }

    std::ostringstream payload;
    payload << "{"
            << "\"task_name\":" << quoted(last_error->task_name) << ","
            << "\"action_name\":" << quoted(last_error->action_name) << ","
            << "\"reason\":" << quoted(to_string(last_error->reason)) << ","
            << "\"message\":" << quoted(failure_message(last_error->reason)) << ","
            << "\"timestamp_ms\":" << last_error->timestamp_ms
            << "}";
    return payload.str();
}

std::string make_manager_state_line(const ManagerSnapshot& snapshot) {
    std::ostringstream payload;
    payload << "{\"lifecycle_state\":" << quoted(snapshot.lifecycle_state)
            << ",\"current_task\":" << quoted(snapshot.current_task)
            << ",\"current_action\":" << quoted(snapshot.current_action)
            << ",\"fire_count\":" << snapshot.fire_count
            << ",\"manual_control_active\":"
            << (snapshot.manual_control_active ? "true" : "false")
            << ",\"host_last_command\":" << quoted(snapshot.host_last_command)
            << ",\"host_last_command_status\":" << quoted(snapshot.host_last_command_status)
            << ",\"host_last_command_timestamp_ms\":"
            << optional_number_json(snapshot.host_last_command_timestamp_ms)
            << ",\"pitch_deg\":" << optional_number_json(snapshot.pitch_deg)
            << ",\"force_channel_1\":" << optional_number_json(snapshot.force_channel_1)
            << ",\"force_channel_2\":" << optional_number_json(snapshot.force_channel_2)
            << ",\"queue\":" << make_queue_json(snapshot.queue)
            << ",\"last_error\":" << make_last_error_json(snapshot.last_error) << "}";
    return make_envelope("manager_state", "", current_time_ms(), payload.str());
}

struct InboundFrame {
    enum class Kind { Line, Oversize };

    Kind kind{Kind::Line};
    std::string line;
};

struct PendingWrite {
    std::string line;
    size_t bytes_sent{0};
    bool close_after_send{false};
};

class HostBridgeRuntime {
public:
    void configure(
        std::string listen_host, uint16_t listen_port, int64_t hello_timeout_ms,
        int64_t heartbeat_interval_ms, size_t max_message_bytes) {
        listen_host_ = std::move(listen_host);
        listen_port_ = listen_port;
        hello_timeout_ms_ = hello_timeout_ms;
        heartbeat_interval_ms_ = heartbeat_interval_ms;
        max_message_bytes_ = max_message_bytes;
    }

    void start(const rclcpp::Logger& logger) {
        if (listen_fd_ >= 0) {
            return;
        }

        listen_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (listen_fd_ < 0) {
            throw std::runtime_error(
                std::string("Failed to create host bridge socket: ") + std::strerror(errno));
        }

        int reuse_addr = 1;
        if (setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse_addr, sizeof(reuse_addr)) < 0) {
            const std::string error = std::string("setsockopt(SO_REUSEADDR) failed: ")
                                    + std::strerror(errno);
            close_fd(listen_fd_);
            throw std::runtime_error(error);
        }

        std::string error;
        if (!set_non_blocking(listen_fd_, error)) {
            close_fd(listen_fd_);
            throw std::runtime_error(error);
        }

        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_port = htons(listen_port_);
        if (inet_pton(AF_INET, listen_host_.c_str(), &address.sin_addr) != 1) {
            close_fd(listen_fd_);
            throw std::runtime_error("Invalid tcp_listen_host, only IPv4 address is supported");
        }

        if (bind(listen_fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
            const std::string bind_error =
                std::string("Failed to bind host bridge socket: ") + std::strerror(errno);
            close_fd(listen_fd_);
            throw std::runtime_error(bind_error);
        }

        if (listen(listen_fd_, 4) < 0) {
            const std::string listen_error =
                std::string("Failed to listen on host bridge socket: ") + std::strerror(errno);
            close_fd(listen_fd_);
            throw std::runtime_error(listen_error);
        }

        RCLCPP_INFO(
            logger, "[HostBridge] listening on %s:%u", listen_host_.c_str(),
            static_cast<unsigned>(listen_port_));
    }

    void stop() {
        close_fd(pending_client_fd_);
        close_fd(client_fd_);
        close_fd(listen_fd_);
        inbound_frames_.clear();
        outbound_lines_.clear();
        recv_buffer_.clear();
        replacement_notice_pending_ = false;
        hello_received_ = false;
        hello_ack_pending_ = false;
        initial_state_pending_ = false;
        hello_request_id_.clear();
        session_id_.clear();
        pending_session_id_.clear();
    }

    void poll_accept_and_receive(const rclcpp::Logger& logger) {
        if (listen_fd_ < 0) {
            return;
        }

        accept_new_clients(logger);
        receive_active_client(logger);
    }

    void flush_outbound(const rclcpp::Logger& logger) {
        while (client_fd_ >= 0 && !outbound_lines_.empty()) {
            PendingWrite& pending = outbound_lines_.front();
            const char* buffer = pending.line.data() + pending.bytes_sent;
            const size_t bytes_left = pending.line.size() - pending.bytes_sent;

            const ssize_t sent = send(client_fd_, buffer, bytes_left, MSG_NOSIGNAL);
            if (sent < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    break;
                }

                RCLCPP_WARN(
                    logger, "[HostBridge] send failed on session '%s': %s", session_id_.c_str(),
                    std::strerror(errno));
                close_active_client(logger);
                break;
            }

            if (sent == 0) {
                break;
            }

            pending.bytes_sent += static_cast<size_t>(sent);
            last_sent_at_ms_ = current_time_ms();
            if (pending.bytes_sent < pending.line.size()) {
                break;
            }

            const bool close_after_send = pending.close_after_send;
            outbound_lines_.pop_front();
            if (close_after_send) {
                close_active_client(logger);
                break;
            }
        }
    }

    [[nodiscard]] bool has_active_client() const { return client_fd_ >= 0; }
    [[nodiscard]] bool hello_received() const { return hello_received_; }
    [[nodiscard]] bool hello_ack_pending() const { return hello_ack_pending_; }
    [[nodiscard]] bool initial_state_pending() const { return initial_state_pending_; }
    [[nodiscard]] bool snapshot_dirty() const { return snapshot_dirty_; }
    [[nodiscard]] bool has_inbound_frames() const { return !inbound_frames_.empty(); }
    [[nodiscard]] bool replacement_notice_pending() const { return replacement_notice_pending_; }

    void clear_hello_ack_pending() { hello_ack_pending_ = false; }
    void clear_initial_state_pending() { initial_state_pending_ = false; }
    void clear_replacement_notice() { replacement_notice_pending_ = false; }
    void clear_outbound() { outbound_lines_.clear(); }

    [[nodiscard]] int64_t heartbeat_interval_ms() const { return heartbeat_interval_ms_; }
    [[nodiscard]] int64_t hello_timeout_ms() const { return hello_timeout_ms_; }
    [[nodiscard]] size_t max_message_bytes() const { return max_message_bytes_; }
    [[nodiscard]] int64_t last_sent_at_ms() const { return last_sent_at_ms_; }
    [[nodiscard]] int64_t client_connected_at_ms() const { return client_connected_at_ms_; }
    [[nodiscard]] const std::string& session_id() const { return session_id_; }
    [[nodiscard]] const std::string& hello_request_id() const { return hello_request_id_; }
    [[nodiscard]] const ManagerSnapshot& latest_snapshot() const { return latest_snapshot_; }

    void accept_hello(std::string request_id) {
        hello_received_ = true;
        hello_ack_pending_ = true;
        initial_state_pending_ = true;
        hello_request_id_ = std::move(request_id);
    }

    void update_snapshot(ManagerSnapshot snapshot) {
        latest_snapshot_ = std::move(snapshot);
        if (latest_snapshot_ != last_sent_snapshot_) {
            snapshot_dirty_ = true;
        }
    }

    void mark_snapshot_sent() {
        last_sent_snapshot_ = latest_snapshot_;
        snapshot_dirty_ = false;
    }

    bool hello_timed_out(int64_t now_ms) const {
        return client_fd_ >= 0 && !hello_received_
            && (now_ms - client_connected_at_ms_) >= hello_timeout_ms_;
    }

    bool should_send_heartbeat(int64_t now_ms) const {
        return client_fd_ >= 0 && hello_received_
            && (now_ms - last_sent_at_ms_) >= heartbeat_interval_ms_;
    }

    InboundFrame pop_inbound_frame() {
        InboundFrame frame = std::move(inbound_frames_.front());
        inbound_frames_.pop_front();
        return frame;
    }

    void queue_line(std::string line, bool close_after_send = false) {
        if (client_fd_ < 0) {
            return;
        }
        outbound_lines_.push_back(PendingWrite{std::move(line), 0, close_after_send});
    }

private:
    void accept_new_clients(const rclcpp::Logger& logger) {
        while (true) {
            sockaddr_in client_address{};
            socklen_t client_address_size = sizeof(client_address);
            int accepted_fd =
                accept(listen_fd_, reinterpret_cast<sockaddr*>(&client_address), &client_address_size);
            if (accepted_fd < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    break;
                }

                RCLCPP_WARN(logger, "[HostBridge] accept failed: %s", std::strerror(errno));
                break;
            }

            std::string error;
            if (!set_non_blocking(accepted_fd, error)) {
                RCLCPP_WARN(logger, "[HostBridge] %s", error.c_str());
                close_fd(accepted_fd);
                continue;
            }

            if (client_fd_ < 0) {
                activate_client(accepted_fd, next_session_id(), logger);
                continue;
            }

            if (pending_client_fd_ >= 0) {
                close_fd(pending_client_fd_);
            }

            pending_client_fd_ = accepted_fd;
            pending_session_id_ = next_session_id();
            replacement_notice_pending_ = true;
            RCLCPP_INFO(
                logger,
                "[HostBridge] new client accepted, current session '%s' will be replaced",
                session_id_.c_str());
            break;
        }
    }

    void receive_active_client(const rclcpp::Logger& logger) {
        if (client_fd_ < 0) {
            return;
        }

        while (true) {
            char buffer[4096];
            const ssize_t received = recv(client_fd_, buffer, sizeof(buffer), 0);
            if (received < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    break;
                }

                RCLCPP_WARN(
                    logger, "[HostBridge] recv failed on session '%s': %s", session_id_.c_str(),
                    std::strerror(errno));
                close_active_client(logger);
                break;
            }

            if (received == 0) {
                RCLCPP_INFO(logger, "[HostBridge] session '%s' disconnected", session_id_.c_str());
                close_active_client(logger);
                break;
            }

            last_received_at_ms_ = current_time_ms();
            recv_buffer_.append(buffer, static_cast<size_t>(received));
            if (recv_buffer_.size() > max_message_bytes_ && recv_buffer_.find('\n') == std::string::npos) {
                inbound_frames_.push_back(InboundFrame{InboundFrame::Kind::Oversize, {}});
                recv_buffer_.clear();
                break;
            }

            extract_lines();
        }
    }

    void extract_lines() {
        while (true) {
            const size_t newline = recv_buffer_.find('\n');
            if (newline == std::string::npos) {
                break;
            }

            std::string line = recv_buffer_.substr(0, newline);
            recv_buffer_.erase(0, newline + 1);
            if (!line.empty() && line.back() == '\r') {
                line.pop_back();
            }

            if (line.size() > max_message_bytes_) {
                inbound_frames_.push_back(InboundFrame{InboundFrame::Kind::Oversize, {}});
                continue;
            }

            inbound_frames_.push_back(InboundFrame{InboundFrame::Kind::Line, std::move(line)});
        }
    }

    void close_active_client(const rclcpp::Logger& logger) {
        close_fd(client_fd_);
        inbound_frames_.clear();
        outbound_lines_.clear();
        recv_buffer_.clear();
        hello_received_ = false;
        hello_ack_pending_ = false;
        initial_state_pending_ = false;
        hello_request_id_.clear();
        session_id_.clear();
        last_sent_at_ms_ = 0;
        client_connected_at_ms_ = 0;
        replacement_notice_pending_ = false;
        promote_pending_client(logger);
    }

    void promote_pending_client(const rclcpp::Logger& logger) {
        if (pending_client_fd_ < 0) {
            return;
        }

        activate_client(pending_client_fd_, std::move(pending_session_id_), logger);
        pending_client_fd_ = -1;
        pending_session_id_.clear();
    }

    void activate_client(int fd, std::string session_id, const rclcpp::Logger& logger) {
        client_fd_ = fd;
        session_id_ = std::move(session_id);
        client_connected_at_ms_ = current_time_ms();
        last_sent_at_ms_ = 0;
        last_received_at_ms_ = client_connected_at_ms_;
        hello_received_ = false;
        hello_ack_pending_ = false;
        initial_state_pending_ = false;
        hello_request_id_.clear();
        recv_buffer_.clear();
        inbound_frames_.clear();
        outbound_lines_.clear();

        RCLCPP_INFO(
            logger, "[HostBridge] session '%s' connected on %s:%u", session_id_.c_str(),
            listen_host_.c_str(), static_cast<unsigned>(listen_port_));
    }

    std::string next_session_id() { return "sess-" + std::to_string(++session_counter_); }

    std::string listen_host_{"127.0.0.1"};
    uint16_t listen_port_{kDefaultListenPort};
    int64_t hello_timeout_ms_{kDefaultHelloTimeoutMs};
    int64_t heartbeat_interval_ms_{kDefaultHeartbeatIntervalMs};
    size_t max_message_bytes_{kDefaultMaxMessageBytes};

    int listen_fd_{-1};
    int client_fd_{-1};
    int pending_client_fd_{-1};

    uint64_t session_counter_{0};
    std::string session_id_;
    std::string pending_session_id_;
    std::string hello_request_id_;

    int64_t client_connected_at_ms_{0};
    int64_t last_received_at_ms_{0};
    int64_t last_sent_at_ms_{0};

    bool replacement_notice_pending_{false};
    bool hello_received_{false};
    bool hello_ack_pending_{false};
    bool initial_state_pending_{false};
    bool snapshot_dirty_{false};

    std::string recv_buffer_;
    std::deque<InboundFrame> inbound_frames_;
    std::deque<PendingWrite> outbound_lines_;

    ManagerSnapshot latest_snapshot_{};
    ManagerSnapshot last_sent_snapshot_{};
};

class HostCommand : public rmcs_executor::Component {
public:
    explicit HostCommand(std::shared_ptr<HostBridgeRuntime> runtime)
        : runtime_(std::move(runtime))
        , logger_(rclcpp::get_logger(get_component_name())) {
        register_output("/dart/manager/host_command", host_command_output_, std::string{});
        register_output(
            "/dart/manager/host_manual/belt_direction", host_manual_belt_direction_output_,
            int32_t{0});
        register_output(
            "/dart/manager/host_manual/lift_direction", host_manual_lift_direction_output_,
            int32_t{0});
        register_output(
            "/dart/manager/host_manual/yaw_direction", host_manual_yaw_direction_output_,
            int32_t{0});
        register_output(
            "/dart/manager/host_manual/pitch_direction", host_manual_pitch_direction_output_,
            int32_t{0});
        register_output(
            "/dart/manager/host_manual/trigger_command", host_manual_trigger_command_output_,
            rmcs_msgs::DartServoCommand::WAIT);
        register_input(
            "/dart/manager/debug/manual_control_active", manual_control_active_input_, false);
    }

    void before_updating() override {
        if (!manual_control_active_input_.ready()) {
            manual_control_active_input_.make_and_bind_directly(false);
            RCLCPP_WARN(
                logger_,
                "Failed to fetch \"/dart/manager/debug/manual_control_active\". Set to false.");
        }
    }

    void update() override {
        *host_command_output_ = std::string{};

        runtime_->poll_accept_and_receive(logger_);

        if (runtime_->replacement_notice_pending()) {
            runtime_->clear_replacement_notice();
            runtime_->clear_outbound();
            runtime_->queue_line(
                make_error_line("", "session_replaced", "Session replaced by a newer client"),
                true);
        }

        if (runtime_->hello_timed_out(current_time_ms())) {
            runtime_->clear_outbound();
            runtime_->queue_line(
                make_error_line("", "bad_request", "hello must be sent within 3000 ms"), true);
        }

        while (runtime_->has_inbound_frames()) {
            const InboundFrame frame = runtime_->pop_inbound_frame();
            if (frame.kind == InboundFrame::Kind::Oversize) {
                runtime_->queue_line(
                    make_error_line("", "bad_request", "Message exceeds 64 KiB limit"));
                continue;
            }

            process_line(frame.line);
            if (runtime_->hello_ack_pending()) {
                break;
            }
        }

        queue_manual_exit_on_disconnect();
        publish_manual_control_outputs();
        emit_next_manager_command();
    }

private:
    struct AxisState {
        bool negative_pressed{false};
        bool positive_pressed{false};

        int32_t direction() const {
            if (positive_pressed == negative_pressed) {
                return 0;
            }
            return positive_pressed ? 1 : -1;
        }

        void reset() {
            negative_pressed = false;
            positive_pressed = false;
        }
    };

    void process_line(const std::string& line) {
        JsonValue json;
        std::string parse_error;
        if (!parse_json_line(line, json, parse_error)) {
            runtime_->queue_line(make_error_line("", "invalid_json", parse_error));
            return;
        }

        if (json.type != JsonType::Object) {
            runtime_->queue_line(make_error_line("", "bad_request", "Envelope must be a JSON object"));
            return;
        }

        const std::optional<std::string> type = string_member(json, "type");
        const std::optional<int64_t> protocol_version = integer_member(json, "protocol_version");
        const std::optional<std::string> request_id = string_member(json, "request_id");
        const JsonValue* payload = object_member(json, "payload");

        if (!type || !protocol_version || !request_id || payload == nullptr) {
            runtime_->queue_line(
                make_error_line("", "bad_request", "Missing or invalid envelope fields"));
            return;
        }

        if (*protocol_version != kProtocolVersion) {
            runtime_->queue_line(
                make_error_line(
                    *request_id, "protocol_version_mismatch", "Only protocol_version=1 is supported"));
            return;
        }

        if (!runtime_->hello_received()) {
            if (*type != "hello") {
                runtime_->queue_line(
                    make_error_line(*request_id, "bad_request", "hello must be the first message"));
                return;
            }

            handle_hello(*request_id, *payload);
            return;
        }

        if (*type == "echo") {
            handle_echo(*request_id, *payload);
            return;
        }

        if (*type == "command") {
            handle_command(*request_id, *payload);
            return;
        }

        runtime_->queue_line(make_error_line(*request_id, "unsupported_type", "Unsupported message type"));
    }

    void handle_hello(const std::string& request_id, const JsonValue& payload) {
        const std::optional<std::string> client_name = string_member(payload, "client_name");
        const std::optional<std::string> client_version = string_member(payload, "client_version");
        const JsonValue* capabilities = array_member(payload, "capabilities");

        if (request_id.empty()) {
            runtime_->queue_line(make_error_line("", "bad_request", "hello.request_id must be non-empty"));
            return;
        }

        if (!client_name || !client_version || capabilities == nullptr) {
            runtime_->queue_line(
                make_error_line(request_id, "bad_request", "hello payload is missing required fields"));
            return;
        }

        runtime_->accept_hello(request_id);
        RCLCPP_INFO(
            logger_, "[HostCommand] hello accepted from client '%s' version '%s'",
            client_name->c_str(), client_version->c_str());
    }

    void handle_echo(const std::string& request_id, const JsonValue& payload) {
        const std::optional<std::string> message = string_member(payload, "message");
        if (!message) {
            runtime_->queue_line(
                make_error_line(request_id, "bad_request", "echo payload requires message"));
            return;
        }

        runtime_->queue_line(make_echo_ack_line(request_id, *message));
    }

    void handle_command(const std::string& request_id, const JsonValue& payload) {
        const std::optional<std::string> name = string_member(payload, "name");
        const JsonValue* args = object_member(payload, "args");
        if (!name || args == nullptr) {
            runtime_->queue_line(
                make_error_line(request_id, "bad_request", "command payload requires name and args"));
            return;
        }

        if (*name == "manual_control_enter") {
            if (manual_control_active()) {
                runtime_->queue_line(
                    make_error_line(request_id, "invalid_state", "manual control is already active"));
                return;
            }

            reset_manual_control_state();
            pending_manager_commands_.push_back(*name);
            runtime_->queue_line(make_command_ack_line(request_id, *name));
            return;
        }

        if (*name == "manual_control_exit") {
            if (!manual_control_active()) {
                runtime_->queue_line(
                    make_error_line(request_id, "invalid_state", "manual control is not active"));
                return;
            }

            reset_manual_control_state();
            pending_manager_commands_.push_back(*name);
            runtime_->queue_line(make_command_ack_line(request_id, *name));
            return;
        }

        if (is_manual_button_command(*name)) {
            if (!manual_control_active()) {
                runtime_->queue_line(
                    make_error_line(
                        request_id, "invalid_state", "manual control must be active first"));
                return;
            }

            const std::optional<std::string> phase = string_member(*args, "phase");
            if (!phase || (*phase != "press" && *phase != "release")) {
                runtime_->queue_line(
                    make_error_line(
                        request_id, "bad_request", "manual control command requires phase=press|release"));
                return;
            }

            apply_manual_button_command(*name, *phase);
            runtime_->queue_line(make_command_ack_line(request_id, *name));
            return;
        }

        if (is_automatic_task_command(*name) && manual_control_active()) {
            runtime_->queue_line(
                make_error_line(
                    request_id, "invalid_state",
                    "automatic task commands are blocked while manual control is active"));
            return;
        }

        if (!is_supported_manager_command(*name)) {
            runtime_->queue_line(
                make_error_line(
                    request_id, "invalid_command",
                    std::string("Unsupported command: ") + *name));
            return;
        }

        pending_manager_commands_.push_back(*name);
        runtime_->queue_line(make_command_ack_line(request_id, *name));
    }

    bool manual_control_active() const {
        return manual_control_active_input_.ready() && *manual_control_active_input_;
    }

    static bool is_automatic_task_command(std::string_view name) {
        return name == "launch_prepare" || name == "launch_cancel" || name == "fire_preload";
    }

    static bool is_supported_manager_command(std::string_view name) {
        return is_automatic_task_command(name) || name == "recover" || name == "cancel";
    }

    static bool is_manual_button_command(std::string_view name) {
        return name == "belt_up" || name == "belt_down" || name == "yaw_left"
            || name == "yaw_right" || name == "pitch_up" || name == "pitch_down"
            || name == "trigger_lock" || name == "trigger_free"
            || name == "filling_lift_up" || name == "filling_lift_down";
    }

    void apply_manual_button_command(const std::string& name, const std::string& phase) {
        const bool pressed = phase == "press";

        if (name == "belt_up") {
            belt_axis_.positive_pressed = pressed;
            return;
        }

        if (name == "belt_down") {
            belt_axis_.negative_pressed = pressed;
            return;
        }

        if (name == "yaw_right") {
            yaw_axis_.positive_pressed = pressed;
            return;
        }

        if (name == "yaw_left") {
            yaw_axis_.negative_pressed = pressed;
            return;
        }

        if (name == "pitch_up") {
            pitch_axis_.positive_pressed = pressed;
            return;
        }

        if (name == "pitch_down") {
            pitch_axis_.negative_pressed = pressed;
            return;
        }

        if (name == "filling_lift_up") {
            lift_axis_.positive_pressed = pressed;
            return;
        }

        if (name == "filling_lift_down") {
            lift_axis_.negative_pressed = pressed;
            return;
        }

        if (name == "trigger_lock" && pressed) {
            trigger_command_ = rmcs_msgs::DartServoCommand::LOCK;
            return;
        }

        if (name == "trigger_free" && pressed) {
            trigger_command_ = rmcs_msgs::DartServoCommand::FREE;
        }
    }

    void reset_manual_control_state() {
        belt_axis_.reset();
        lift_axis_.reset();
        yaw_axis_.reset();
        pitch_axis_.reset();
        trigger_command_ = rmcs_msgs::DartServoCommand::WAIT;
    }

    void queue_manual_exit_on_disconnect() {
        if (!manual_control_active()) {
            disconnect_exit_queued_ = false;
            return;
        }

        if (runtime_->hello_received()) {
            disconnect_exit_queued_ = false;
            return;
        }

        if (disconnect_exit_queued_) {
            return;
        }

        reset_manual_control_state();
        pending_manager_commands_.push_back("manual_control_exit");
        disconnect_exit_queued_ = true;
    }

    void publish_manual_control_outputs() {
        const bool active = manual_control_active();
        *host_manual_belt_direction_output_ = active ? belt_axis_.direction() : 0;
        *host_manual_lift_direction_output_ = active ? lift_axis_.direction() : 0;
        *host_manual_yaw_direction_output_ = active ? yaw_axis_.direction() : 0;
        *host_manual_pitch_direction_output_ = active ? pitch_axis_.direction() : 0;
        *host_manual_trigger_command_output_ =
            active ? trigger_command_ : rmcs_msgs::DartServoCommand::WAIT;
    }

    void emit_next_manager_command() {
        if (hold_output_gap_) {
            hold_output_gap_ = false;
            return;
        }

        if (pending_manager_commands_.empty()) {
            return;
        }

        *host_command_output_ = pending_manager_commands_.front();
        pending_manager_commands_.pop_front();
        hold_output_gap_ = true;
    }

    std::shared_ptr<HostBridgeRuntime> runtime_;
    rclcpp::Logger logger_;
    OutputInterface<std::string> host_command_output_;
    OutputInterface<int32_t> host_manual_belt_direction_output_;
    OutputInterface<int32_t> host_manual_lift_direction_output_;
    OutputInterface<int32_t> host_manual_yaw_direction_output_;
    OutputInterface<int32_t> host_manual_pitch_direction_output_;
    OutputInterface<rmcs_msgs::DartServoCommand> host_manual_trigger_command_output_;
    InputInterface<bool> manual_control_active_input_;
    std::deque<std::string> pending_manager_commands_;
    AxisState belt_axis_;
    AxisState lift_axis_;
    AxisState yaw_axis_;
    AxisState pitch_axis_;
    rmcs_msgs::DartServoCommand trigger_command_{rmcs_msgs::DartServoCommand::WAIT};
    bool disconnect_exit_queued_{false};
    bool hold_output_gap_{false};
};

class HostStatusOutput : public rmcs_executor::Component {
public:
    explicit HostStatusOutput(std::shared_ptr<HostBridgeRuntime> runtime)
        : runtime_(std::move(runtime))
        , logger_(rclcpp::get_logger(get_component_name())) {
        register_input("/dart/manager/debug/lifecycle_state", lifecycle_state_input_);
        register_input("/dart/manager/debug/current_task", current_task_input_);
        register_input("/dart/manager/debug/current_action", current_action_input_);
        register_input("/dart/manager/debug/manual_control_active", manual_control_active_input_);
        register_input("/dart/manager/debug/host_last_command", host_last_command_input_);
        register_input(
            "/dart/manager/debug/host_last_command_status", host_last_command_status_input_);
        register_input(
            "/dart/manager/debug/host_last_command_timestamp_ms",
            host_last_command_timestamp_input_);
        register_input("/dart/manager/fire_count", fire_count_input_);
        register_input("/dart/manager/debug/queue", queue_input_);
        register_input("/dart/manager/debug/last_error", last_error_input_);
        register_input("/imu/catapult_pitch_angle", pitch_deg_input_, false);
        register_input("/force_sensor/channel_1/weight", force_channel_1_input_, false);
        register_input("/force_sensor/channel_2/weight", force_channel_2_input_, false);
    }

    void update() override {
        const std::optional<int64_t> host_last_command_timestamp_ms =
            !host_last_command_input_->empty() && *host_last_command_timestamp_input_ > 0
                ? std::optional<int64_t>{*host_last_command_timestamp_input_}
                : std::nullopt;
        const std::optional<double> pitch_deg =
            pitch_deg_input_.ready() ? std::optional<double>{*pitch_deg_input_} : std::nullopt;
        const std::optional<int32_t> force_channel_1 = force_channel_1_input_.ready()
                                                     ? std::optional<int32_t>{*force_channel_1_input_}
                                                     : std::nullopt;
        const std::optional<int32_t> force_channel_2 = force_channel_2_input_.ready()
                                                     ? std::optional<int32_t>{*force_channel_2_input_}
                                                     : std::nullopt;

        runtime_->update_snapshot(ManagerSnapshot{
            *lifecycle_state_input_,
            *current_task_input_,
            *current_action_input_,
            *fire_count_input_,
            *manual_control_active_input_,
            *host_last_command_input_,
            *host_last_command_status_input_,
            host_last_command_timestamp_ms,
            pitch_deg,
            force_channel_1,
            force_channel_2,
            *queue_input_,
            *last_error_input_,
        });

        if (runtime_->hello_ack_pending()) {
            runtime_->queue_line(
                make_hello_ack_line(
                    runtime_->hello_request_id(), runtime_->session_id(),
                    runtime_->heartbeat_interval_ms()));
            runtime_->clear_hello_ack_pending();
        }

        if (runtime_->hello_received()) {
            if (runtime_->initial_state_pending() || runtime_->snapshot_dirty()) {
                runtime_->queue_line(make_manager_state_line(runtime_->latest_snapshot()));
                runtime_->mark_snapshot_sent();
                runtime_->clear_initial_state_pending();
            } else if (runtime_->should_send_heartbeat(current_time_ms())) {
                runtime_->queue_line(make_heartbeat_line(runtime_->session_id()));
            }
        }

        runtime_->flush_outbound(logger_);
    }

private:
    std::shared_ptr<HostBridgeRuntime> runtime_;
    rclcpp::Logger logger_;
    InputInterface<std::string> lifecycle_state_input_;
    InputInterface<std::string> current_task_input_;
    InputInterface<std::string> current_action_input_;
    InputInterface<bool> manual_control_active_input_;
    InputInterface<std::string> host_last_command_input_;
    InputInterface<std::string> host_last_command_status_input_;
    InputInterface<int64_t> host_last_command_timestamp_input_;
    InputInterface<uint32_t> fire_count_input_;
    InputInterface<std::vector<ManagerQueuedTaskInfo>> queue_input_;
    InputInterface<std::optional<ManagerLastErrorInfo>> last_error_input_;
    InputInterface<double> pitch_deg_input_;
    InputInterface<int32_t> force_channel_1_input_;
    InputInterface<int32_t> force_channel_2_input_;
};

class HostBridge
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    HostBridge()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , runtime_(std::make_shared<HostBridgeRuntime>())
        , logger_(get_logger()) {
        create_partner_component<HostCommand>("host_command", runtime_);
        create_partner_component<HostStatusOutput>("host_status_output", runtime_);
    }

    ~HostBridge() override { runtime_->stop(); }

    void before_updating() override {
        const std::string listen_host = get_parameter_or("tcp_listen_host", std::string("127.0.0.1"));
        const auto listen_port = static_cast<uint16_t>(get_parameter_or("tcp_listen_port", static_cast<int>(kDefaultListenPort)));
        const auto hello_timeout_ms =
            static_cast<int64_t>(get_parameter_or("tcp_hello_timeout_ms", static_cast<int>(kDefaultHelloTimeoutMs)));
        const auto heartbeat_interval_ms = static_cast<int64_t>(
            get_parameter_or("tcp_heartbeat_interval_ms", static_cast<int>(kDefaultHeartbeatIntervalMs)));
        const auto max_message_bytes =
            static_cast<size_t>(get_parameter_or("tcp_max_message_bytes", static_cast<int>(kDefaultMaxMessageBytes)));

        runtime_->configure(
            listen_host, listen_port, hello_timeout_ms, heartbeat_interval_ms, max_message_bytes);
        runtime_->start(logger_);
    }

    void update() override {}

private:
    std::shared_ptr<HostBridgeRuntime> runtime_;
    rclcpp::Logger logger_;
};

} // namespace
} // namespace rmcs_dart_guidance::manager

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_dart_guidance::manager::HostBridge, rmcs_executor::Component)
