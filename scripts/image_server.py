#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from string_srvs.srv import GetImage, GetImageRequest, GetImageResponse


class ImageSubs(object):
    def __init__(self):
        self.last_img = None
        self.sub = rospy.Subscriber('/webcam/image_raw',
                                    Image,
                                    self.img_cb,
                                    queue_size=1)
        self.serv = rospy.Service('/get_webcam_image',
                                  GetImage,
                                  self.get_img_cb)

    def img_cb(self, img):
        self.last_img = img

    def get_img_cb(self, req):
        # if req.topic == '/webcam/image_raw'
        return GetImageResponse(img=self.last_img)


if __name__ == '__main__':
    rospy.init_node('test_img_server')
    ImgSub = ImageSubs()
    rospy.spin()
