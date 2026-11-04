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
// 创建者：qiayuan，创建时间：2021 年 3 月 3 日。
//

#pragma once

#include <linux/can.h>
#include <net/if.h>
// 多线程支持
#include <pthread.h>
#include <atomic>
#include <cstdint>
#include <functional>
#include <string>

namespace damiao
{

  class SocketCAN
  {
public:
    SocketCAN() = default;
    ~SocketCAN();
    SocketCAN(const SocketCAN &) = delete;
    SocketCAN & operator = (const SocketCAN &) = delete;

    /** \brief 打开并绑定套接字。
     *
     * \param interface 总线名称（例如：can0）。
     * \param handler 从 CAN 总线接收到数据帧时调用的回调函数。
     * \param thread_priority 接收线程的实时优先级；小于等于 0 表示不设置实时优先级。
     *
     * \returns 如果套接字成功打开并绑定，则返回 \c true。
     */
    bool open(
      const std::string & interface,
      std::function < void(const canfd_frame & frame) > handler,
      int thread_priority);
    /** \brief 关闭并解除绑定套接字。
     *
     */
    void close();
    /** \brief 返回套接字当前是否处于打开状态。
     *
     * \returns 如果套接字已打开，则返回 \c true。
     */
    bool isOpen() const;
    /** \brief 将指定的数据帧发送到总线。
     *
     * \param frame 指向待发送数据帧的指针。
     */
    bool write(const can_frame * frame) const;
    /** \brief 将 CAN FD 数据帧发送到总线。
     *
     * \param frame 指向待发送 CAN FD 数据帧的指针。
     */
    bool write2(const canfd_frame * frame) const;

private:
    static void * receiverThread(void * instance) noexcept;
    bool startReceiverThread(int thread_priority);
    void logThrottledError() const;

    ifreq interface_request_ {};
    sockaddr_can address_ {};
    pthread_t receiver_thread_id_ {};
    int sock_fd_ = -1;
    std::atomic_bool terminate_receiver_thread_ {false};
    bool receiver_thread_started_ = false;
    std::function < void(const canfd_frame & frame) > reception_handler_;
    mutable std::atomic < std::int64_t > last_error_log_s_ {0};
  };

}  // namespace damiao
