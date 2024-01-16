#!/usr/bin/env python3

'''
Copyright (c) 2016, Nadya Ampilogova
All rights reserved.
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

# Script for simulation
# Launch gazebo world prior to run this script

from __future__ import print_function
import subprocess
import sys
import uuid
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class TakePhoto:
    def __init__(self,save_path):

        self.bridge = CvBridge()
        self.image_received = False
        self.save_path = save_path  # Adiciona o caminho da pasta

        # Connect image topic
        img_topic = "/camera/rgb/image_raw"
        self.image_sub = rospy.Subscriber(img_topic, Image, self.callback)

        # Allow up to one second to connection
        rospy.sleep(1)

    def callback(self, data):

        # Convert image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.image_received = True
        self.image = cv_image

    def take_picture(self, img_title):
        if self.image_received:
            # Salva a imagem no caminho configurado
            full_path = f"{self.save_path}/{img_title}"
            cv2.imwrite(full_path, self.image)
            return full_path  # Retorna o caminho completo onde a imagem foi salva
        else:
            return None

if __name__ == '__main__':

    # Initialize
    rospy.init_node('take_photo', anonymous=False)
    #camera = TakePhoto()

    # Take a photo
    #save_path = rospy.get_param('~save_path', 'pasta/image')  # Caminho padrão: 'pasta/image'
    camera = TakePhoto(save_path='/home/nunomsfs/catkin_ws/src/Trabalho_final_PSR/perception_robutler/images')
    # Use '_image_title' parameter from command line
    # Default value is 'photo.jpg'
    #uuid_str = str(uuid.uuid4())
    #name= "picture_"+uuid_str+".jpg"
    #bashCommand = 'rosrun perception_robutler take_picture.py _image_title:=name '
    #check_process = subprocess.Popen(bashCommand.split())
    img_title = rospy.get_param('~image_title', 'photo.jpg')
    # use the command: python take_photo.py _image_title:="new_title.jpg" 
    # to set the param

    if camera.take_picture(img_title):
        rospy.loginfo("Saved image " + img_title)
    else:
        rospy.loginfo("No images received")

    # Sleep to give the last log messages time to be sent
    rospy.sleep(1)