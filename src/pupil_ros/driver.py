import rospy
from std_srvs.srv import Trigger, TriggerResponse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from pupil_labs.realtime_api.simple import discover_one_device


class PupilAsyncDriver(object):
    def __init__(self):
        self._discover_device()
        self._generate_event_srvs()#

        self.bridge = CvBridge()
        self.scene_pub = rospy.Publisher("~scene_image", Image, queue_size=5)
        self.rate = rospy.get_param("~rate", 30)

    def __del__(self):
        self.device.close()

    def _discover_device(self):
        rospy.loginfo("Searching for Pupil device...")
        self.device = discover_one_device(max_search_duration_seconds=10.0)
        if self.device is None: 
            raise RuntimeError("Could not find Pupil device.")
        
        rospy.loginfo("Found Pupil device: %s", self.device.serial_number_glasses)

    def _generate_event_srvs(self):
        self.events_srvs = [
            {"srv_name": "start_session", "event_name": "session.begin"},
            {"srv_name": "stop_session", "event_name": "session.end"},
            {"srv_name": "start_intervention", "event_name": "intervention.begin"},
            {"srv_name": "stop_intervention", "event_name": "intervention.end"}
        ]

        for srv_dict in self.events_srvs:
            srv_dict["srv"] = rospy.Service(srv_dict["srv_name"], Trigger, lambda req, msg: self._event_cbk(req, msg, srv_dict["event_name"]))

    def _event_cbk(self, req, msg):
        self.device.send_event(msg)
        return TriggerResponse(success=True)
    
    def _start_recording(self, req, msg):
        rid = self.device.recording_start()
        rospy.loginfo("Started recording with id: %s", rid)
        return TriggerResponse(success=True)

    def _stop_recording(self, req, msg):
        self.device.recording_stop_and_save()
        rospy.loginfo("Stopped recording.")
        return TriggerResponse(success=True)

    def spin(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            tstp = rospy.Time.now()
            frame, gaze = self.device.receive_matched_scene_video_frame_and_gaze()
            if frame is not None:
                scene_msg = self.bridge.cv2_to_imgmsg(frame.bgr_pixels, encoding="bgr8")
                scene_msg.header.stamp = tstp
                self.scene_pub.publish(scene_msg)

            # if gaze is not None:

            rate.sleep()