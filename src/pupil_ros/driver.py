import rospy
from std_srvs.srv import Trigger, TriggerResponse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import matplotlib.pyplot as plt

from pupil_labs.realtime_api.simple import discover_one_device, Device


class PupilAsyncDriver(object):
    def __init__(self):
        self.phone_ip = rospy.get_param("~phone_ip", "10.0.0.0")
        self._discover_device()
        self._generate_event_srvs()

        self.bridge = CvBridge()
        self.scene_pub = rospy.Publisher("~scene_image", Image, queue_size=5)
        self.scene_gaze_pub = rospy.Publisher("~scene_image_with_gaze", Image, queue_size=5)
        self.rate = rospy.get_param("~rate", 30)
        self.stream_img = rospy.get_param("~stream_img", False)

        self.start_rec_srv = rospy.Service("~start_recording", Trigger, self._start_recording)
        self.stop_rec_srv = rospy.Service("~stop_recording", Trigger, self._stop_recording)

        rospy.loginfo("Pupil driver initialised.")

    def __del__(self):
        self.device.close()

    def _discover_device(self):
        if self.phone_ip != "10.0.0.0":
            rospy.loginfo("Searching for Pupil device with phone ip: %s", self.phone_ip)
            self.device = Device(address=self.phone_ip, port="8080")
        else:
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
            {"srv_name": "stop_intervention", "event_name": "intervention.end"},
            {"srv_name": "start_takeover", "event_name": "takeover.begin"},
            {"srv_name": "stop_takeover", "event_name": "takeover.end"},
            {"srv_name": "start_movement", "event_name": "move.begin"},
            {"srv_name": "end_movement", "event_name": "move.end"},
            {"srv_name": "report_failure", "event_name": "failure"},
            {"srv_name": "report_success", "event_name": "success"},
        ]

        for srv_dict in self.events_srvs:
            srv_dict["srv"] = rospy.Service("~" + srv_dict["srv_name"], Trigger, lambda req, ev_name=srv_dict["event_name"]: self._event_cbk(ev_name))

    def _event_cbk(self, msg):
        rospy.loginfo("Sending event: %s", msg)
        self.device.send_event(msg)
        return TriggerResponse(success=True)
    
    def _start_recording(self, req):
        rid = self.device.recording_start()
        rospy.loginfo("Started recording with id: %s", rid)
        return TriggerResponse(success=True)

    def _stop_recording(self, req):
        self.device.recording_stop_and_save()
        rospy.loginfo("Stopped recording.")
        return TriggerResponse(success=True)

    def spin(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if self.stream_img:
                tstp = rospy.Time.now()
                frame, gaze = self.device.receive_matched_scene_video_frame_and_gaze()
                if frame is not None:

                    frame_bgr = frame.bgr_pixels
                    scene_msg = self.bridge.cv2_to_imgmsg(frame_bgr, encoding="bgr8")
                    scene_msg.header.stamp = tstp
                    self.scene_pub.publish(scene_msg)

                    # Add gaze circle
                    cv2.circle(frame_bgr, (int(gaze.x), int(gaze.y)), 10, (0, 0, 255), 2)
                    scene_with_gaze_msg = self.bridge.cv2_to_imgmsg(frame_bgr, encoding="bgr8")
                    scene_with_gaze_msg.header.stamp = tstp
                    self.scene_gaze_pub.publish(scene_with_gaze_msg)

                else:
                    rospy.logwarn("No frame received.")

            rate.sleep()