import rospy

from pupil_ros.driver import PupilAsyncDriver

if __name__ == '__main__':
    rospy.init_node('invisible_driver')
    drv = PupilAsyncDriver()
    drv.spin()