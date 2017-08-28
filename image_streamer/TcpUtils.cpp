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
#include "TcpUtils.hpp"

int TcpServer::CreateSocket (const char *ip_addr, int port_num)
{
  sock_ = socket(AF_INET, SOCK_STREAM, 0);
  if (sock_ < 0) {
    printf("ERROR: Could not create socket\n");
    return -1;
  }

  printf("Created socket\n");

  memset((void *)&serv_addr_, 0, sizeof(serv_addr_));
  serv_addr_.sin_family = AF_INET;
  serv_addr_.sin_addr.s_addr = inet_addr(ip_addr);
  serv_addr_.sin_port = htons(port_num);
  serv_addr_size_ = sizeof(serv_addr_);

  return 0;
}

int TcpServer::BindSocket (void)
{
  int sockoptval = 1;
  if (setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, &sockoptval, sizeof(int)) < 0)
  {
    printf("ERROR: Error with setsockopt\n");
    return -1;
  }

  int res = bind(sock_, (struct sockaddr *)&serv_addr_, sizeof(serv_addr_));
  if (res < 0) {
    printf("ERROR: Could not bind address to socket\n");
    return -1;
  }
  printf("Bind successful\n");

  return 0;
}

int TcpServer::CheckSocket (void)
{
  int sockoptval = 1;
  socklen_t len = sizeof(sockoptval);
  if (getsockopt(data_sock_, SOL_SOCKET, SO_ERROR, &sockoptval, &len) < 0)
  {
    printf("ERROR: Error with getsockopt\n");
    return -1;
  }

  if (sockoptval == 0)
  {
    return 0;
  }

  printf("ERROR: Socket connection error\n");
  return -1;
}

int TcpServer::ConnectClient (void)
{
  int res = listen(sock_, 5);
  if (res < 0) {
    printf("ERROR: Socket listen failed\n");
    return -1;
  }
  printf("Listen successful, waiting for connection...\n");

  struct sockaddr_in client_addr;
  socklen_t client_addr_size = sizeof(client_addr);
  data_sock_ = accept(sock_, (struct sockaddr *)&client_addr, &client_addr_size);
  if (data_sock_ < 0) {
    printf("ERROR: Connection accept failed [%d]\n", errno);
    return -1;
  }
  FD_ZERO(&master_);
  FD_SET(data_sock_, &master_);
  printf("Connection established\n");

  return 0;
}

int TcpServer::RecvMessage (void)
{
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 1000;
  readfds_ = master_;
  if (select(data_sock_+1, &readfds_, NULL, NULL, &tv) < 0)
  {
    printf("ERROR: Error with select\n");
    return -1;
  }
  if (FD_ISSET(data_sock_, &readfds_))
  {
    uint8_t recv_buf;
    int recvlen = recv(data_sock_, &recv_buf, 1, 0);
    if (recvlen < 0) {
      recv_buf = 0;
      return -1;
    }
    return recv_buf;
  }

  return 0;
}

int TcpServer::SendMessage (const uint8_t* msg, size_t msg_len,
    int msg_id, uint16_t num_cols, uint16_t num_rows, uint8_t* opts,
    int32_t frame, int64_t timestamp)
{
  ImageHeader* msg_hdr = new ImageHeader();
  msg_hdr->message_id = msg_id;
  msg_hdr->flag = 0;
  msg_hdr->frame_id = frame;
  msg_hdr->timestamp_ns = timestamp;
  msg_hdr->num_cols = num_cols;
  msg_hdr->num_rows = num_rows;
  memcpy(msg_hdr->opts, opts, 4*sizeof(uint8_t));
  msg_hdr->checksum = 0; // placeholder for checksum (not implemented)

  uint8_t *packet = new uint8_t[24];
  memcpy(packet, &msg_hdr->message_id, 1);
  memcpy(packet+1, &msg_hdr->flag, 1);
  memcpy(packet+2, &msg_hdr->frame_id, 4);
  memcpy(packet+6, &msg_hdr->timestamp_ns, 8);
  memcpy(packet+14, &msg_hdr->num_cols, 2);
  memcpy(packet+16, &msg_hdr->num_rows, 2);
  memcpy(packet+18, &msg_hdr->opts, 4);
  memcpy(packet+22, &msg_hdr->checksum, 2);

  // First send the header
  int res = send(data_sock_, packet, 24, 0);
  if (res < 0)
  {
    printf("ERROR: Error in sending image header\n");
    return -1;
  }

  // Now send the image data
  res = send(data_sock_, msg, msg_len, 0);
  if (res < 0)
  {
    printf("ERROR: Error in sending image\n");
    return -1;
  }

  delete[] packet;
  delete[] msg_hdr;

  return 0;
}

int TcpServer::SendMessage(const uint16_t* msg, size_t msg_len,
    int msg_id, uint16_t num_cols, uint16_t num_rows, uint8_t* opts,
    int32_t frame, int64_t timestamp)
{
  int m = sizeof(uint16_t)/sizeof(uint8_t);

  ImageHeader* msg_hdr = new ImageHeader();
  msg_hdr->message_id = msg_id;
  msg_hdr->flag = 0;
  msg_hdr->frame_id = frame;
  msg_hdr->timestamp_ns = timestamp;
  msg_hdr->num_cols = num_cols;
  msg_hdr->num_rows = num_rows;
  memcpy(msg_hdr->opts, opts, 4*sizeof(uint8_t));
  msg_hdr->checksum = 0;

  uint8_t *packet = new uint8_t[24];
  memcpy(packet, &msg_hdr->message_id, 1);
  memcpy(packet+1, &msg_hdr->flag, 1);
  memcpy(packet+2, &msg_hdr->frame_id, 4);
  memcpy(packet+6, &msg_hdr->timestamp_ns, 8);
  memcpy(packet+14, &msg_hdr->num_cols, 2);
  memcpy(packet+16, &msg_hdr->num_rows, 2);
  memcpy(packet+18, &msg_hdr->opts, 4);
  memcpy(packet+22, &msg_hdr->checksum, 2);

  int n = num_cols*num_rows;
  for (int i = 0; i < m; ++i)
  {
    // First send the header
    int res = send(data_sock_, packet, 24, 0);
    if (res < 0)
    {
      printf("ERROR: Error in sending image header\n");
      return -1;
    }

    // Now send the image data
    res = send(data_sock_, msg + (i*(n/m)), msg_len/m, 0);
    if (res < 0)
    {
      printf("ERROR: Error in sending image\n");
      return -1;
    }
  }

  delete[] packet;
  delete[] msg_hdr;

  return 0;
}

int TcpServer::SendMessage (const float* msg, size_t msg_len,
    int msg_id, uint16_t num_cols, uint16_t num_rows, uint8_t* opts,
    int32_t frame, int64_t timestamp)
{
  int m = sizeof(float)/sizeof(uint8_t);

  ImageHeader* msg_hdr = new ImageHeader();
  msg_hdr->message_id = msg_id;
  msg_hdr->flag = 0;
  msg_hdr->frame_id = frame;
  msg_hdr->timestamp_ns = timestamp;
  msg_hdr->num_cols = num_cols;
  msg_hdr->num_rows = num_rows;
  memcpy(msg_hdr->opts, opts, 4*sizeof(uint8_t));
  msg_hdr->checksum = 0;

  uint8_t *packet = new uint8_t[24];
  memcpy(packet, &msg_hdr->message_id, 1);
  memcpy(packet+1, &msg_hdr->flag, 1);
  memcpy(packet+2, &msg_hdr->frame_id, 4);
  memcpy(packet+6, &msg_hdr->timestamp_ns, 8);
  memcpy(packet+14, &msg_hdr->num_cols, 2);
  memcpy(packet+16, &msg_hdr->num_rows, 2);
  memcpy(packet+18, &msg_hdr->opts, 4);
  memcpy(packet+22, &msg_hdr->checksum, 2);

  int n = num_cols*num_rows;
  for (int i = 0; i < m; ++i)
  {
    // First send the header
    int res = send(data_sock_, packet, 24, 0);
    if (res < 0)
    {
      printf("ERROR: Error in sending image header\n");
      return -1;
    }

    // Now send the image data
    res = send(data_sock_, msg + (i*(n/m)), msg_len/m, 0);
    if (res < 0)
    {
      printf("ERROR: Error in sending image\n");
      return -1;
    }
  }

  delete[] packet;
  delete[] msg_hdr;

  return 0;
}

/** Closes socket
*/
TcpServer::~TcpServer (void)
{
  close(sock_);
  close(data_sock_);
  printf("Closing sockets\n");
}
