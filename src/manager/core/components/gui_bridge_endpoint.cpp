#include "manager/core/components/gui_bridge_ports.hpp"
#include "manager/gui_bridge_json_utils.hpp"
#include "manager/gui_bridge_runtime.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <openssl/evp.h>
#include <openssl/sha.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <regex>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_dart_guidance::manager {

namespace {

constexpr uint16_t kDefaultBridgePort = 18081;
constexpr std::string_view kDefaultBridgeAddress = "127.0.0.1";
constexpr std::string_view kWebSocketGuid = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

std::optional<std::string> match_json_string(const std::string& payload, const char* key) {
    const std::regex pattern(
        "\"" + std::string(key) + R"("\s*:\s*"([^"]*))", std::regex::ECMAScript);
    std::smatch match;
    if (!std::regex_search(payload, match, pattern) || match.size() != 2) {
        return std::nullopt;
    }

    return match[1].str();
}

std::string websocket_accept_key(const std::string& client_key) {
    const std::string source = client_key + std::string{kWebSocketGuid};
    unsigned char digest[SHA_DIGEST_LENGTH];
    SHA1(reinterpret_cast<const unsigned char*>(source.data()), source.size(), digest);

    std::vector<unsigned char> encoded(EVP_ENCODE_LENGTH(SHA_DIGEST_LENGTH));
    const int encoded_length = EVP_EncodeBlock(encoded.data(), digest, SHA_DIGEST_LENGTH);
    return std::string(reinterpret_cast<const char*>(encoded.data()), encoded_length);
}

bool recv_exact(int fd, void* buffer, std::size_t size) {
    auto* data = static_cast<unsigned char*>(buffer);
    std::size_t total = 0;

    while (total < size) {
        const ssize_t received = ::recv(fd, data + total, size - total, 0);
        if (received <= 0) {
            return false;
        }

        total += static_cast<std::size_t>(received);
    }

    return true;
}

bool send_all(int fd, const void* buffer, std::size_t size) {
    const auto* data = static_cast<const unsigned char*>(buffer);
    std::size_t total = 0;

    while (total < size) {
        const ssize_t sent = ::send(fd, data + total, size - total, 0);
        if (sent <= 0) {
            return false;
        }

        total += static_cast<std::size_t>(sent);
    }

    return true;
}

} // namespace

class GuiBridgeEndpoint
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    GuiBridgeEndpoint()
        : Node(get_component_name())
        , runtime_(std::make_shared<GuiBridgeRuntime>()) {
        bridge_enabled_ = declare_parameter<bool>("enabled", true);
        bridge_address_ =
            declare_parameter<std::string>("listen_address", std::string{kDefaultBridgeAddress});
        fail_on_bind_error_ = declare_parameter<bool>("fail_on_bind_error", false);

        const int64_t listen_port =
            declare_parameter<int64_t>("listen_port", static_cast<int64_t>(kDefaultBridgePort));
        if (listen_port <= 0 || listen_port > std::numeric_limits<uint16_t>::max()) {
            throw std::runtime_error(
                "Invalid gui bridge listen_port: " + std::to_string(listen_port));
        }
        bridge_port_ = static_cast<uint16_t>(listen_port);

        create_partner_component<GuiCommandBridge>("gui_command_bridge", runtime_);
        create_partner_component<GuiBridgeStatePort>("gui_bridge_state_port", runtime_);
    }

    ~GuiBridgeEndpoint() override {
        running_ = false;
        close_socket(server_fd_);
        close_client_socket();

        if (server_thread_.joinable()) {
            server_thread_.join();
        }
    }

    void before_updating() override {
        if (server_start_attempted_) {
            return;
        }

        server_start_attempted_ = true;
        if (!bridge_enabled_) {
            RCLCPP_INFO(get_logger(), "[GuiBridgeEndpoint] disabled by parameter");
            return;
        }

        try {
            start_server();
            server_started_ = true;
            RCLCPP_INFO(
                get_logger(), "[GuiBridgeEndpoint] listening on ws://%s:%u/ws",
                bridge_address_.c_str(), static_cast<unsigned>(bridge_port_));
        } catch (const std::system_error& exception) {
            if (exception.code().value() != EADDRINUSE || fail_on_bind_error_) {
                RCLCPP_ERROR(
                    get_logger(), "[GuiBridgeEndpoint] failed to start ws://%s:%u/ws: %s",
                    bridge_address_.c_str(), static_cast<unsigned>(bridge_port_), exception.what());
                throw;
            }

            RCLCPP_ERROR(
                get_logger(),
                "[GuiBridgeEndpoint] failed to start ws://%s:%u/ws: %s. GUI bridge will stay "
                "disabled for this executor.",
                bridge_address_.c_str(), static_cast<unsigned>(bridge_port_), exception.what());
        } catch (const std::exception& exception) {
            RCLCPP_ERROR(
                get_logger(), "[GuiBridgeEndpoint] failed to start ws://%s:%u/ws: %s",
                bridge_address_.c_str(), static_cast<unsigned>(bridge_port_), exception.what());
            throw;
        }
    }

    void update() override {
        std::deque<std::string> pending_events;
        std::string snapshot_to_send;

        {
            std::scoped_lock lock(runtime_->state_mutex);
            pending_events.swap(runtime_->pending_events);

            if (runtime_->snapshot_version != last_sent_snapshot_version_) {
                snapshot_to_send = runtime_->latest_snapshot_json;
                last_sent_snapshot_version_ = runtime_->snapshot_version;
            }
        }

        for (const auto& event_json : pending_events) {
            if (!event_json.empty()) {
                send_text(event_json);
            }
        }

        if (!snapshot_to_send.empty()) {
            send_text(snapshot_to_send);
        }
    }

private:
    [[noreturn]] void throw_socket_error(const char* operation, int err) const {
        throw std::system_error(
            err, std::generic_category(),
            std::string("Failed to ") + operation + " gui bridge socket (" + bridge_address_ + ":"
                + std::to_string(bridge_port_) + ")");
    }

    void start_server() {
        server_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd_ < 0) {
            throw_socket_error("create", errno);
        }

        int enable = 1;
        if (::setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable)) < 0) {
            const int err = errno;
            close_socket(server_fd_);
            server_fd_ = -1;
            throw_socket_error("configure", err);
        }

        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_port = htons(bridge_port_);
        if (::inet_pton(AF_INET, bridge_address_.c_str(), &address.sin_addr) != 1) {
            close_socket(server_fd_);
            server_fd_ = -1;
            throw std::runtime_error("Failed to parse gui bridge address: " + bridge_address_);
        }

        if (::bind(server_fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
            const int err = errno;
            close_socket(server_fd_);
            server_fd_ = -1;
            throw_socket_error("bind", err);
        }

        if (::listen(server_fd_, 1) < 0) {
            const int err = errno;
            close_socket(server_fd_);
            server_fd_ = -1;
            throw_socket_error("listen on", err);
        }

        server_thread_ = std::thread([this]() { server_loop(); });
    }

    void server_loop() {
        while (running_ && rclcpp::ok()) {
            sockaddr_in client_addr{};
            socklen_t client_addr_len = sizeof(client_addr);
            const int accepted_fd =
                ::accept(server_fd_, reinterpret_cast<sockaddr*>(&client_addr), &client_addr_len);
            if (accepted_fd < 0) {
                if (running_ && errno != EBADF && errno != EINVAL) {
                    RCLCPP_WARN(
                        get_logger(), "[GuiBridgeEndpoint] accept failed: %s",
                        std::strerror(errno));
                }
                continue;
            }

            if (!perform_handshake(accepted_fd)) {
                close_socket(accepted_fd);
                continue;
            }

            {
                std::scoped_lock lock(client_mutex_);
                close_socket(client_fd_);
                client_fd_ = accepted_fd;
            }

            send_latest_snapshot();
            read_client_loop(accepted_fd);
        }
    }

    static bool perform_handshake(int client_fd) {
        std::string request;
        request.reserve(2048);

        char buffer[1024];
        while (request.find("\r\n\r\n") == std::string::npos) {
            const ssize_t bytes = ::recv(client_fd, buffer, sizeof(buffer), 0);
            if (bytes <= 0) {
                return false;
            }

            request.append(buffer, static_cast<std::size_t>(bytes));
            if (request.size() > 8192) {
                return false;
            }
        }

        const std::regex key_pattern(
            R"(Sec-WebSocket-Key:\s*([^\r\n]+)\r\n)", std::regex::ECMAScript);
        std::smatch match;
        if (!std::regex_search(request, match, key_pattern) || match.size() != 2) {
            return false;
        }

        const std::string accept_key = websocket_accept_key(match[1].str());
        const std::string response = "HTTP/1.1 101 Switching Protocols\r\n"
                                     "Upgrade: websocket\r\n"
                                     "Connection: Upgrade\r\n"
                                     "Sec-WebSocket-Accept: "
                                   + accept_key + "\r\n\r\n";

        return send_all(client_fd, response.data(), response.size());
    }

    void read_client_loop(int client_fd) {
        while (running_ && rclcpp::ok()) {
            pollfd descriptor{};
            descriptor.fd = client_fd;
            descriptor.events = POLLIN;

            const int poll_result = ::poll(&descriptor, 1, 200);
            if (poll_result == 0) {
                continue;
            }
            if (poll_result < 0) {
                break;
            }
            if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
                break;
            }
            if ((descriptor.revents & POLLIN) == 0) {
                continue;
            }

            auto payload = read_text_frame(client_fd);
            if (!payload.has_value()) {
                break;
            }

            handle_message(*payload);
        }

        std::scoped_lock lock(client_mutex_);
        if (client_fd_ == client_fd) {
            close_socket(client_fd_);
            client_fd_ = -1;
        } else {
            close_socket(client_fd);
        }
    }

    std::optional<std::string> read_text_frame(int client_fd) {
        unsigned char header[2];
        if (!recv_exact(client_fd, header, sizeof(header))) {
            return std::nullopt;
        }

        const unsigned char opcode = header[0] & 0x0F;
        const bool masked = (header[1] & 0x80U) != 0U;
        uint64_t payload_length = header[1] & 0x7FU;

        if (payload_length == 126) {
            unsigned char extended[2];
            if (!recv_exact(client_fd, extended, sizeof(extended))) {
                return std::nullopt;
            }
            payload_length =
                (static_cast<uint64_t>(extended[0]) << 8U) | static_cast<uint64_t>(extended[1]);
        } else if (payload_length == 127) {
            unsigned char extended[8];
            if (!recv_exact(client_fd, extended, sizeof(extended))) {
                return std::nullopt;
            }

            payload_length = 0;
            for (const unsigned char byte : extended) {
                payload_length = (payload_length << 8U) | static_cast<uint64_t>(byte);
            }
        }

        unsigned char masking_key[4]{};
        if (masked && !recv_exact(client_fd, masking_key, sizeof(masking_key))) {
            return std::nullopt;
        }

        std::string payload(payload_length, '\0');
        if (payload_length > 0 && !recv_exact(client_fd, payload.data(), payload.size())) {
            return std::nullopt;
        }

        if (masked) {
            for (uint64_t index = 0; index < payload_length; ++index) {
                payload[static_cast<std::size_t>(index)] ^=
                    static_cast<char>(masking_key[index % 4U]);
            }
        }

        if (opcode == 0x8) {
            return std::nullopt;
        }

        if (opcode == 0x9) {
            send_control_frame(0xA, payload);
            return std::string{};
        }

        if (opcode != 0x1) {
            return std::string{};
        }

        return payload;
    }

    void handle_message(const std::string& payload) {
        if (payload.empty()) {
            return;
        }

        const auto type = match_json_string(payload, "type");
        const std::string request_id = match_json_string(payload, "request_id").value_or("");
        const auto command = match_json_string(payload, "command");

        if (!type.has_value() || *type != "task.submit" || !command.has_value()) {
            send_ack(request_id, false, "invalid_message");
            return;
        }

        static const std::set<std::string> kSupportedCommands{
            "slider_init",    "launch_prepare", "launch_cancel", "fire_preload",
            "manual_control", "cancel",         "recover",
        };

        if (!kSupportedCommands.contains(*command)) {
            send_ack(request_id, false, "unsupported_command");
            return;
        }

        {
            std::scoped_lock lock(runtime_->command_mutex);
            runtime_->pending_gui_commands.push_back(*command);
        }
        send_ack(request_id, true, "");
    }

    void send_ack(const std::string& request_id, bool accepted, const std::string& reason) {
        send_text(
            "{\"type\":\"command.ack\",\"request_id\":\"" + escape_json_string(request_id)
            + "\",\"accepted\":" + std::string{accepted ? "true" : "false"} + ",\"reason\":\""
            + escape_json_string(reason) + "\"}");
    }

    void send_latest_snapshot() {
        std::string snapshot;
        {
            std::scoped_lock lock(runtime_->state_mutex);
            snapshot = runtime_->latest_snapshot_json;
            last_sent_snapshot_version_ = runtime_->snapshot_version;
        }

        if (!snapshot.empty()) {
            send_text(snapshot);
        }
    }

    void send_text(const std::string& payload) {
        std::scoped_lock lock(client_mutex_);
        if (client_fd_ < 0) {
            return;
        }

        if (!send_frame_locked(client_fd_, 0x1, payload)) {
            close_socket(client_fd_);
            client_fd_ = -1;
        }
    }

    void send_control_frame(unsigned char opcode, const std::string& payload) {
        std::scoped_lock lock(client_mutex_);
        if (client_fd_ < 0) {
            return;
        }

        if (!send_frame_locked(client_fd_, opcode, payload)) {
            close_socket(client_fd_);
            client_fd_ = -1;
        }
    }

    static bool send_frame_locked(int client_fd, unsigned char opcode, const std::string& payload) {
        std::array<unsigned char, 10> header{};
        std::size_t header_size = 0;

        header[header_size++] = static_cast<unsigned char>(0x80U | (opcode & 0x0FU));
        const uint64_t payload_size = payload.size();
        if (payload_size <= 125) {
            header[header_size++] = static_cast<unsigned char>(payload_size);
        } else if (payload_size <= 0xFFFF) {
            header[header_size++] = 126;
            header[header_size++] = static_cast<unsigned char>((payload_size >> 8U) & 0xFFU);
            header[header_size++] = static_cast<unsigned char>(payload_size & 0xFFU);
        } else {
            header[header_size++] = 127;
            for (int shift = 56; shift >= 0; shift -= 8) {
                header[header_size++] =
                    static_cast<unsigned char>((payload_size >> shift) & 0xFFU);
            }
        }

        if (!send_all(client_fd, header.data(), header_size)) {
            return false;
        }

        return payload.empty() || send_all(client_fd, payload.data(), payload.size());
    }

    void close_client_socket() {
        std::scoped_lock lock(client_mutex_);
        close_socket(client_fd_);
        client_fd_ = -1;
    }

    static void close_socket(int fd) {
        if (fd >= 0) {
            ::shutdown(fd, SHUT_RDWR);
            ::close(fd);
        }
    }

    std::shared_ptr<GuiBridgeRuntime> runtime_;

    std::thread server_thread_;
    std::atomic<bool> running_{true};
    bool bridge_enabled_{true};
    bool fail_on_bind_error_{false};
    std::string bridge_address_{std::string{kDefaultBridgeAddress}};
    uint16_t bridge_port_{kDefaultBridgePort};
    bool server_start_attempted_{false};
    bool server_started_{false};

    std::mutex client_mutex_;
    int server_fd_{-1};
    int client_fd_{-1};

    uint64_t last_sent_snapshot_version_{0};
};

} // namespace rmcs_dart_guidance::manager

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_dart_guidance::manager::GuiBridgeEndpoint, rmcs_executor::Component)
