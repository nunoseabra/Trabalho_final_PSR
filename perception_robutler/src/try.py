import subprocess
import uuid

import rospy


uuid_str = str(uuid.uuid4())
uuid_str=uuid_str[0:6]


name= "picture_"+uuid_str+".jpg"

bashCommand = 'rosparam set /take_photo/image_title ' + name 
picture_process = subprocess.Popen(bashCommand.split())

img_title = rospy.get_param('~image_title', name)

print(img_title)