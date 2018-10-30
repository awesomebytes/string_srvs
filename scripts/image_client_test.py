#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from string_srvs.srv import GetImage, GetImageRequest, GetImageResponse
import time

if __name__ == '__main__':
    rospy.init_node('asdfasfd')

    srv = rospy.ServiceProxy('/get_webcam_image', GetImage, persistent=True)
    srv.wait_for_service()

    # check which is faster
    loop_ini_t = time.time()
    r = rospy.Rate(30)
    best_wait_time = 9999.9
    worst_wait_time = 0.0
    best_srv_persistent_time = 9999.9
    worst_srv_persistent_time = 0.0
    best_srv_new_time = 9999.9
    worst_srv_new_time = 0.0
    while not rospy.is_shutdown() and time.time() - loop_ini_t < 5.0:
        ini_t = time.time()
        img = rospy.wait_for_message('/webcam/image_raw', Image)
        fin_t = time.time()
        total_t = fin_t - ini_t
        if total_t > worst_wait_time:
            worst_wait_time = total_t
        if total_t < best_wait_time:
            best_wait_time = total_t
        print("wait_for_message took: " + str(total_t))
        # wait_for_message took: 0.0846312046051
        # wait_for_message took: 0.0473592281342

        # srv call is way faster
        ini_t = time.time()
        img = srv.call(GetImageRequest())
        fin_t = time.time()
        total_t = fin_t - ini_t
        if total_t > worst_srv_persistent_time:
            worst_srv_persistent_time = total_t
        if total_t < best_srv_persistent_time:
            best_srv_persistent_time = total_t
        print("srv.call took: " + str(total_t))
        # srv.call took: 0.0073869228363
        # srv.call took: 0.0153398513794

        # no real overhead
        ini_t = time.time()
        srv2 = rospy.ServiceProxy('/get_webcam_image', GetImage)
        img = srv2.call(GetImageRequest())
        srv2.close()
        fin_t = time.time()
        total_t = fin_t - ini_t
        if total_t > worst_srv_new_time:
            worst_srv_new_time = total_t
        if total_t < best_srv_new_time:
            best_srv_new_time = total_t
        print("srv.call with making proxy took: " + str(total_t))
        # srv.call with making proxy took: 0.00325012207031
        # srv.call with making proxy took: 0.0139148235321

        r.sleep()

    print("Summary on delay to get an image:")
    print("Best wait: " + str(best_wait_time))
    print("Worst wait: " + str(worst_wait_time))
    print("Best persistent srv: " + str(best_srv_persistent_time))
    print("Worst persistent srv: " + str(worst_srv_persistent_time))
    print("Best new srv: " + str(best_srv_new_time))
    print("Worst new srv: " + str(worst_srv_new_time))

    # This represents how long we need to wait to get an image
    # Best wait: 0.0247027873993
    # Worst wait: 0.11227107048
    # Best persistent srv: 0.00110411643982
    # Worst persistent srv: 0.00875806808472
    # Best new srv: 0.00319695472717
    # Worst new srv: 0.0427930355072

    # Now lets try what is the delay of the image
    global best_subs_time_dif
    global worst_subs_time_dif
    best_subs_time_dif = 9999.9
    worst_subs_time_dif = 0.0

    def subs_cb(img):
        tnow = rospy.get_time()
        t = img.header.stamp.to_sec()
        t_dif = tnow - t
        global best_subs_time_dif
        global worst_subs_time_dif
        if t_dif > worst_subs_time_dif:
            worst_subs_time_dif = t_dif
        if t_dif < best_subs_time_dif:
            best_subs_time_dif = t_dif
        print("Subscriber time dif: " + str(t_dif))

    subs = rospy.Subscriber('/webcam/image_raw',
                            Image,
                            subs_cb,
                            queue_size=1)
    rospy.sleep(10.0)
    subs.unregister()

    looping_t = time.time()
    best_wait_time_dif = 9999.9
    worst_wait_time_dif = 0.0
    best_srv_time_dif = 9999.9
    worst_srv_time_dif = 0.0
    while not rospy.is_shutdown() and time.time() - looping_t < 10.0:
        img = rospy.wait_for_message('/webcam/image_raw',
                                     Image)
        tnow = rospy.get_time()
        t = img.header.stamp.to_sec()
        t_dif = tnow - t
        if t_dif > worst_wait_time_dif:
            worst_wait_time_dif = t_dif
        if t_dif < best_wait_time_dif:
            best_wait_time_dif = t_dif
        print("wait_for_msg time dif: " + str(t_dif))

        img = srv.call(GetImageRequest())
        tnow = rospy.get_time()
        t = img.img.header.stamp.to_sec()
        t_dif = tnow - t
        if t_dif > worst_srv_time_dif:
            worst_srv_time_dif = t_dif
        if t_dif < best_srv_time_dif:
            best_srv_time_dif = t_dif
        print("srv persist call time dif: " + str(t_dif))

    print("Summary on timestamp oldness")
    print("Best subs: " + str(best_subs_time_dif))
    print("Worst subs: " + str(worst_subs_time_dif))
    print("Best wait: " + str(best_wait_time_dif))
    print("Worst wait: " + str(worst_wait_time_dif))
    print("Best persistent srv: " + str(best_srv_time_dif))
    print("Worst persistent srv: " + str(worst_srv_time_dif))


    # Summary on timestamp oldness
    # Best subs: 0.00172400474548
    # Worst subs: 0.0129487514496
    # Best wait: 0.00565791130066
    # Worst wait: 0.0300512313843
    # Best persistent srv: 0.00703191757202
    # Worst persistent srv: 0.032989025116

    # Final experiment results

    # To get an image
    # You'll get it from
    # subs: 0.0 seconds later
    # wait: 0.02-0.11 seconds later
    # persistent srv: 0.001-0.008 seconds later
    # new srv: 0.003-0.042 seconds later

    # From the moment you request an image
    # You'll get it from
    # Subs: 0.001-0.012 in the past
    # wait: 0.005-0.03 in the past
    # srv: 0.007-0.03 in the past


    # So for cases we need to react ultra-fast
    # and on every image... USE SUBSCRIBER

    # For cases we want an image from time to time
    # USE SERVICE (persistent if possible)

    # There is no use-case for wait_for_message
