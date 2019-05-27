#pragma once

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <array>
#include <cassert>
#include <cstring>
#include <functional>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <utility>
#include <vector>
#include <fstream>

namespace sockvis {

enum {
    UDP_BUFFER_SIZE_LIMIT = 65507,
};

class udp_client {
public:
    udp_client(const char s_addr[], int s_port) {
        assert(sock_raii_.sockfd > 0);
        bzero(reinterpret_cast<char*>(&s_addr_), sizeof(s_addr_));
        s_addr_.sin_family = AF_INET;
        s_addr_.sin_port   = htons(s_port);
        assert(inet_aton(s_addr, &s_addr_.sin_addr) > 0);
    }

    bool send(const char* buf, int buf_len) const {
        if (buf_len > UDP_BUFFER_SIZE_LIMIT)
            std::cerr << "sending an UDP packet exceeds " << UDP_BUFFER_SIZE_LIMIT << " bytes\n";
        int ret = ::sendto(sock_raii_.sockfd, buf, buf_len, 0,
                reinterpret_cast<const sockaddr*>(&s_addr_), sizeof(s_addr_));
        return (ret == buf_len);
    }

private:
    struct udp_raii {
        udp_raii() { sockfd = socket(AF_INET, SOCK_DGRAM, 0); }
        ~udp_raii() { ::close(sockfd); }
        int sockfd;
    } sock_raii_;

    /* server address */
    sockaddr_in s_addr_;
};

template<typename quat_type, typename pos_type>
void send_pose(
        const udp_client& udp,
        const std::string& operation,
        const std::string& name,
        const std::string& color,
        const int id,
        const quat_type& q,
        const pos_type& p) {
    std::stringstream ss;
    ss << operation << " p " << name << " " << color << " " << id << " "
       << std::scientific << std::setprecision(4)
       << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " "
       << p.x() << " " << p.y() << " " << p.z() << " <";
    std::string buf = ss.str();

    while (not udp.send(buf.data(), buf.size()));
}

template<typename point_type>
void send_points(
        const udp_client& udp,
        const std::string& operation,
        const std::string& name,
        const std::string& color,
        const std::vector<point_type>& points) {
    std::stringstream ss;
    ss << operation << " l " << name << " " << color << " ";
    std::string header = ss.str();
    for (auto it = points.begin(); it != points.end();) {
        std::string buf = header;
        for (; it != points.end(); ++it) {
            std::stringstream ss;
            ss << std::scientific << std::setprecision(4)
               << it->x() << " " << it->y() << " " << it->z() << " ";
            ss.seekg(0, std::ios::end);
            int buf_size = ss.tellg();
            if (buf.size() + buf_size > UDP_BUFFER_SIZE_LIMIT - 1)
                break;
            buf += ss.str();
        }
        buf.push_back('<');
        while (not udp.send(buf.data(), buf.size()));
    }
}

template<typename pos_type>
void send_vector(
        const udp_client& udp,
        const std::string& operation,
        const std::string& name,
        const std::string& color,
        const pos_type& p0,
        const pos_type& p1) {
    std::stringstream ss;
    ss << operation << " v " << name << " " << color << " "
       << std::scientific << std::setprecision(4)
       << p0.x() << " " << p0.y() << " " << p0.z() << " "
       << p1.x() << " " << p1.y() << " " << p1.z() << " <";
    std::string buf = ss.str();
    while (not udp.send(buf.data(), buf.size()));
}

inline void send_message(
        const udp_client& udp,
        const std::string& operation,
        const std::string& message) {
    std::stringstream ss;
    ss << operation << " m " << message << " <";
    std::string buf = ss.str();
    while (not udp.send(buf.data(), buf.size()));
}

}  /* namespace sockvis */
