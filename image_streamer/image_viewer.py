#! /usr/bin/env python
"""
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
 """
import sys
import os
from PyQt4.QtCore import *
from PyQt4.QtGui import *
import struct
import socket
import time
import argparse


class CameraDisplay(QLabel):
    def __init__(self, parent = None):
        super(CameraDisplay, self).__init__(parent)

    def update_frame(self, image):
        self.setPixmap(QPixmap.fromImage(image))


class ImageViewerGui(QWidget):
    camera_window_signal = pyqtSignal(QImage)

    def __init__(self, parent = None):
        super(ImageViewerGui, self).__init__(parent)

        # Image dislay
        self.camera_window = CameraDisplay()
        self.camera_window_signal.connect(self.camera_window.update_frame)
        self.img_format = QImage.Format_Indexed8
        self.img_color_table = [qRgb(v,v,v) for v in xrange(256)]
        self.save_now = False
        self.save_cntr = 0

        # Save button
        save_btn = QPushButton("Save image")
        save_btn.clicked.connect(lambda: self.handle_save_btn())

        # Gain and exposure buttons
        gain_label = QLabel("Gain")
        gain_label.setAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
        gain_down_btn = QPushButton("<--")
        gain_down_btn.clicked.connect(lambda: self.send_command(189))
        gain_up_btn = QPushButton("-->")
        gain_up_btn.clicked.connect(lambda: self.send_command(190))
        exposure_label = QLabel("Exposure")
        exposure_label.setAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
        exposure_down_btn = QPushButton("<--")
        exposure_down_btn.clicked.connect(lambda: self.send_command(191))
        exposure_up_btn = QPushButton("-->")
        exposure_up_btn.clicked.connect(lambda: self.send_command(192))

        # Set window layout
        grid = QGridLayout()
        grid.setSpacing(10)

        grid.addWidget(self.camera_window, 0, 9, 5, 5)
        grid.addWidget(save_btn, 0, 0, 1, 5)
        grid.addWidget(gain_label, 2, 1, 1, 3)
        grid.addWidget(gain_down_btn, 2, 0, 1, 1)
        grid.addWidget(gain_up_btn, 2, 4, 1, 1)
        grid.addWidget(exposure_label, 3, 1, 1, 3)
        grid.addWidget(exposure_down_btn, 3, 0, 1, 1)
        grid.addWidget(exposure_up_btn, 3, 4, 1, 1)
        self.setLayout(grid)

        self.resize(500, 300)
        self.setWindowTitle("Image Viewer GUI")
        self.show()

        self.stream_connected = False

        # Create a dummy image
        img = QImage(300, 300, self.img_format)
        img.fill(qRgb(0,0,0))
        img.setColorTable(self.img_color_table)
        self.camera_window_signal.emit(img)



    def wait_for_connection(self):

        # Wait for the connection over TCP/IP
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.ipaddr, 5556))
            self.packet_buf = []
            print "Connection initialized to", self.ipaddr

            self.stream_connected = True
            self.initialize_stream()

        except:
            QTimer.singleShot(100, self.wait_for_connection)


    def unpack_header(self):

        hdr = ''.join(self.packet_buf[:24])
        self.header = {}
        self.header['msg_id'] = struct.unpack('B', hdr[0])[0]
        self.header['flag'] = struct.unpack('B', hdr[1])[0]
        self.header['frame_id'] = struct.unpack('i', hdr[2:6])[0]
        self.header['timestamp_ns'] = struct.unpack('q', hdr[6:14])[0]
        self.header['num_cols'] = struct.unpack('H', hdr[14:16])[0]
        self.header['num_rows'] = struct.unpack('H', hdr[16:18])[0]
        self.header['opts'] = struct.unpack('B'*4, hdr[18:22])
        self.header['checksum'] = struct.unpack('H', hdr[22:24])[0]


    def initialize_stream(self):

        while len(self.packet_buf) < 24:
            data = self.sock.recv(24)
            self.packet_buf.extend(data)

        self.unpack_header()
        self.packet_size = 24 + self.header['num_cols'] * self.header['num_rows']

        if (self.header['msg_id'] == 2):
            self.resize(self.header['num_cols'], self.header['num_rows'])
        elif (self.header['msg_id'] == 20) or (self.header['msg_id'] == 21):
            self.resize(2*self.header['num_cols'], self.header['num_rows'])


    def send_command(self, message):
        self.sock.send(struct.pack('B', message))


    def handle_save_btn(self):
        self.save_now = True
        print "Save image!", str(self.save_cntr).zfill(3)

    def run(self):
        try:
            # This is the image receive loop
            # Continue to read in images and display until user quits

            if (not self.stream_connected):
               raise UserWarning("stream not connected")

            while len(self.packet_buf) < self.packet_size:
                data = self.sock.recv(self.packet_size)
                self.packet_buf.extend(data)

            self.unpack_header()

            # Monocular image
            if (self.header['msg_id'] == 2):
                frame = ''.join(self.packet_buf[24:self.packet_size])
                img = QImage(frame, self.header['num_cols'], \
                        self.header['num_rows'], \
                        self.header['num_cols'], self.img_format)
                img.setColorTable(self.img_color_table)
                self.camera_window_signal.emit(img)

                if (self.save_now):
                    img_name = "image_" + str(self.save_cntr).zfill(3) + ".png"
                    img.save(img_name)
                    self.save_now = False
                    self.save_cntr += 1

            # Get left image for stereo display
            elif (self.header['msg_id'] == 20):
                self.left_frame = ''.join(self.packet_buf[24:self.packet_size])
                self.left_time = self.header['timestamp_ns']

            # Get right image and process both stereo frames
            elif (self.header['msg_id'] == 21):
                self.right_frame = ''.join(self.packet_buf[24:self.packet_size])
                self.right_time = self.header['timestamp_ns']

                if (self.left_time == self.right_time):
                    frame = ''
                    start = 0
                    end = self.header['num_cols']
                    for r in xrange(self.header['num_rows']):
                        lr = self.left_frame[start:end]
                        rr = self.right_frame[start:end]
                        frame = ''.join([frame, lr, rr])
                        start = end
                        end += self.header['num_cols']

                    img = QImage(frame, 2*self.header['num_cols'], \
                            self.header['num_rows'], \
                            2*self.header['num_cols'], self.img_format)
                    img.setColorTable(self.img_color_table)
                    self.camera_window_signal.emit(img)

                    if (self.save_now):
                        img_name_left = "left_" + str(self.save_cntr).zfill(3) + ".png"
                        img_left = QImage(self.left_frame, \
                            self.header['num_cols'], self.header['num_rows'], \
                            self.img_format)
                        img_left.setColorTable(self.img_color_table)
                        img_left.save(img_name_left)

                        img_name_right = "right_" + str(self.save_cntr).zfill(3) + ".png"
                        img_right = QImage(self.right_frame, \
                            self.header['num_cols'], self.header['num_rows'], \
                            self.img_format)
                        img_right.setColorTable(self.img_color_table)
                        img_right.save(img_name_right)

                        self.save_now = False
                        self.save_cntr += 1

            # Clear the packet buffer to read the next image
            while (len(self.packet_buf) >= self.packet_size):
                self.packet_buf = self.packet_buf[self.packet_size:]

        except UserWarning, e:
            pass

        finally:
            QTimer.singleShot(10, self.run)


def main():
    # Do stuff here
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', '--ipaddress', type=str, help="IP address of the vehicle", required=True)
    args = parser.parse_args()

    app = QApplication(sys.argv)
    ivg = ImageViewerGui()
    ivg.ipaddr = args.ipaddress
    ivg.wait_for_connection()
    ivg.run()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
