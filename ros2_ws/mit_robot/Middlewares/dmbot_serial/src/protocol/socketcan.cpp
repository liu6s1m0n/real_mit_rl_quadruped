/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

//
// Created by qiayuan on 3/3/21.
//
#include "dmbot_serial/protocol/socketcan.h"
#include "dmbot_serial/protocol/terminal_colors.h"
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <iostream>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <utility>
#include <unistd.h>
#include <linux/can.h>
#include <linux/can/raw.h>

namespace damiao
{
/* ref:
 * https://github.com/JCube001/socketcan-demo
 * http://blog.mbedded.ninja/programming/operating-systems/linux/how-to-use-socketcan-with-c-in-linux
 * https://github.com/linux-can/can-utils/blob/master/candump.c
 */

SocketCAN::~SocketCAN()
{
  close();
}

void SocketCAN::logThrottledError() const
{
  const auto now = std::chrono::duration_cast<std::chrono::seconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  auto previous = last_error_log_s_.load(std::memory_order_relaxed);
  if ((previous == 0 || now - previous >= 5) &&
    last_error_log_s_.compare_exchange_strong(previous, now, std::memory_order_relaxed))
  {
    std::cerr << terminalColor(TerminalColor::Yellow, stderr)
              << "Unable to write CAN frame on " << interface_request_.ifr_name
              << terminalColorReset(stderr) << std::endl;
  }
}


bool SocketCAN::open(
  const std::string & interface,
  std::function<void(const canfd_frame & frame)> handler,
  int thread_priority)
{
  if (isOpen() || interface.empty() || interface.size() >= IFNAMSIZ) {return false;}
  reception_handler_ = std::move(handler);
  sock_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (sock_fd_ == -1) {
    //ROS_ERROR("Error: Unable to create a CAN socket");
    std::cerr << terminalColor(TerminalColor::Red, stderr) <<
      "[ERROR] Error: Unable to create a CAN socket" << terminalColorReset(stderr) << std::endl;
    return false;
  }
  int enable_canfd = 1;
  if (setsockopt(
      sock_fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd,
      sizeof(enable_canfd)) != 0)
  {
    std::cerr << terminalColor(TerminalColor::Red, stderr) <<
      "[ERROR] Failed to enable CAN FD support on socket" << terminalColorReset(stderr) <<
      std::endl;
    ::close(sock_fd_);
    sock_fd_ = -1;
    return false;
  }
  interface_request_ = {};
  std::strncpy(interface_request_.ifr_name, interface.c_str(), IFNAMSIZ - 1);
  // Get the index of the network interface
  if (ioctl(sock_fd_, SIOCGIFINDEX, &interface_request_) == -1) {
    //ROS_ERROR("Unable to select CAN interface %s: I/O control error", name);
    std::cerr << terminalColor(TerminalColor::Red, stderr) <<
      "[ERROR] Unable to select CAN interface " << interface << ": I/O control error" <<
      terminalColorReset(stderr) << std::endl;
    // Invalidate unusable socket
    ::close(sock_fd_);
    sock_fd_ = -1;
    return false;
  }
  // Bind the socket to the network interface
  address_.can_family = AF_CAN;// 指定协议族
  address_.can_ifindex = interface_request_.ifr_ifindex; // 设备索引
  int rc = bind(sock_fd_, reinterpret_cast<struct sockaddr *>(&address_), sizeof(address_));
  if (rc == -1) {
    //ROS_ERROR("Failed to bind socket to %s network interface", name);
    std::cerr << terminalColor(TerminalColor::Red, stderr) <<
      "[ERROR] Failed to bind socket to " << interface << " network interface" <<
      terminalColorReset(stderr) << std::endl;
    ::close(sock_fd_);
    sock_fd_ = -1;
    return false;
  }
  // Start a separate, event-driven thread for frame reception
  if (startReceiverThread(thread_priority)) {return true;}
  ::close(sock_fd_);
  sock_fd_ = -1;
  return false;
}

void SocketCAN::close()
{
  terminate_receiver_thread_.store(true);
  if (receiver_thread_started_) {
    static_cast<void>(pthread_join(receiver_thread_id_, nullptr));
    receiver_thread_started_ = false;
  }
  if (isOpen()) {
    ::close(sock_fd_);
    sock_fd_ = -1;
  }
  reception_handler_ = {};
}

bool SocketCAN::isOpen() const
{
  return sock_fd_ != -1;
}

bool SocketCAN::write(const can_frame * frame) const
{
  if (!isOpen()) {
    //ROS_ERROR_THROTTLE(5., "Unable to write: Socket %s not open", interface_request_.ifr_name);
    logThrottledError();
    return false;
  }
  const ssize_t written = ::write(sock_fd_, frame, sizeof(can_frame));
  if (written != static_cast<ssize_t>(sizeof(can_frame))) {
    //ROS_DEBUG_THROTTLE(5., "Unable to write: The %s tx buffer may be full", interface_request_.ifr_name);
    logThrottledError();
    return false;
  }
  return true;
}

bool SocketCAN::write2(const canfd_frame * frame) const
{
  if (!isOpen()) {
    //ROS_ERROR_THROTTLE(5., "Unable to write: Socket %s not open", interface_request_.ifr_name);
    logThrottledError();
    return false;
  }
  const ssize_t written = ::write(sock_fd_, frame, sizeof(canfd_frame));
  if (written != static_cast<ssize_t>(sizeof(canfd_frame))) {
    //ROS_DEBUG_THROTTLE(5., "Unable to write: The %s tx buffer may be full", interface_request_.ifr_name);
    logThrottledError();
    return false;
  }
  return true;
}

void * SocketCAN::receiverThread(void * instance) noexcept
{
  auto * socket = static_cast<SocketCAN *>(instance);
  fd_set descriptors;
  canfd_frame rx_frame{};
  while (!socket->terminate_receiver_thread_.load()) {
    timeval timeout{};
    timeout.tv_usec = 100000;
    FD_ZERO(&descriptors);
    FD_SET(socket->sock_fd_, &descriptors);
    const int selected = select(
      socket->sock_fd_ + 1, &descriptors, nullptr, nullptr, &timeout);
    if (selected < 0) {
      if (errno == EINTR) {continue;}
      break;
    }
    if (selected == 0 || !FD_ISSET(socket->sock_fd_, &descriptors)) {continue;}
    const ssize_t len = read(socket->sock_fd_, &rx_frame, CANFD_MTU);
    if (len != CAN_MTU && len != CANFD_MTU) {continue;}
    try {
      if (socket->reception_handler_) {socket->reception_handler_(rx_frame);}
    } catch (...) {
      std::cerr << terminalColor(TerminalColor::Red, stderr)
                << "[ERROR] SocketCAN receive callback threw an exception"
                << terminalColorReset(stderr) << std::endl;
      break;
    }
  }
  return nullptr;
}

bool SocketCAN::startReceiverThread(int thread_priority)
{
  terminate_receiver_thread_.store(false);
  const int rc = pthread_create(&receiver_thread_id_, nullptr, &SocketCAN::receiverThread, this);
  if (rc != 0) {
    //ROS_ERROR("Unable to start receiver thread");
    std::cerr << terminalColor(TerminalColor::Red, stderr) <<
      "[ERROR] Unable to start receiver thread" << terminalColorReset(stderr) << std::endl;
    return false;
  }
  receiver_thread_started_ = true;
  //ROS_INFO("Successfully started receiver thread with ID %lu", receiver_thread_id_);
  std::cout << terminalColor(TerminalColor::White, stdout) <<
    "[INFO] Successfully started receiver thread with ID " << receiver_thread_id_ <<
    terminalColorReset(stdout) << std::endl;
  if (thread_priority > 0) {
    const int maximum = sched_get_priority_max(SCHED_FIFO);
    sched_param schedule{};
    schedule.sched_priority = maximum > 0 ? std::min(thread_priority, maximum) : 0;
    if (schedule.sched_priority == 0 ||
      pthread_setschedparam(receiver_thread_id_, SCHED_FIFO, &schedule) != 0)
    {
      std::cerr << terminalColor(TerminalColor::Yellow, stderr)
                << "[WARN] Unable to set SocketCAN receiver real-time priority"
                << terminalColorReset(stderr) << std::endl;
    }
  }
  return true;
}

}  // namespace damiao
