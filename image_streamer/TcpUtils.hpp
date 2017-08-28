/****************************************************************************
 *   Copyright (c) 2017 Stephen Chaves. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ATLFlight nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMIED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#ifndef TCP_UTILS_HPP_
#define TCP_UTILS_HPP_

#include <arpa/inet.h>
#include <errno.h>
#include <iostream>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>


struct ImageHeader
{
  uint8_t message_id;
  uint8_t flag;
  int32_t frame_id;
  int64_t timestamp_ns;
  uint16_t num_cols;
  uint16_t num_rows;
  uint8_t opts[4];
  uint16_t checksum;
};


class TcpServer
{
 public:
  int CreateSocket (const char *ip_addr, int port_num);
  int BindSocket (void);
  int CheckSocket (void);
  int ConnectClient (void);
  int RecvMessage (void);
  int SendMessage (const uint8_t *msg, size_t msg_len,
      int msg_id, uint16_t num_cols, uint16_t num_rows,
      uint8_t *opts, int32_t frame, int64_t timestamp);
  int SendMessage (const uint16_t *msg, size_t msg_len,
      int msg_id, uint16_t num_cols, uint16_t num_rows,
      uint8_t *opts, int32_t frame, int64_t timestamp);
  int SendMessage (const float *msg, size_t msg_len,
      int msg_id, uint16_t num_cols, uint16_t num_rows,
      uint8_t *opts, int32_t frame, int64_t timestamp);
  ~TcpServer (void);

 private:
  int sock_;
  int data_sock_;
  size_t pack_len_;
  struct sockaddr_in serv_addr_;
  socklen_t serv_addr_size_;
  fd_set master_;
  fd_set readfds_;
};

#endif // TCP_UTILS_HPP_
