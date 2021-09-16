import rosbag
import rospy
import time
import os
from datetime import datetime


class RosbagRecorder(object):
    """ Subscribe to topics and record to rosbags

    Provide a simple python interface to recording rosbags. Using rosbag as a
    ros command line tool from a sub process is sub optimal, because it's
    especially difficult and ugly to manage the nodes lifetime in a separate
    thread.
    """
    def __init__(self, topic_dict):
        """ Subscribe to each topic given in topic_dict

        topic_dict is of the form {'topic_name': ros_type}
        """
        self.topic_callbacks = []
        self.msg_buffer = {}
        for t_name, t_type in topic_dict.items():
            self.topic_callbacks.append(rospy.Subscriber(
                t_name, t_type, self.buffer, callback_args=t_name, queue_size=10))
            self.msg_buffer[t_name] = []

        self.recording = False

    def buffer(self, msg, topic_name):
        """ Buffer incoming messages if in recording mode

        Store tuples of (msg_data, time_stamp) for each topic name.
        """
        if self.recording:
            self.msg_buffer[topic_name].append((msg, rospy.Time.now()))

    def start_recording(self, wait_for_data=False):
        """ Activate recording to internal buffers

        If wait_for_data == True, this function will block until the first
        message is received.  This can be used to record for an exact period of
        time, by using this as a trigger in the main thread.
        """
        if self.recording:
            return

        self.recording = True  # Activate callback
        if wait_for_data:
            while not any(self.msg_buffer.values()):
                time.sleep(0.001)
                rospy.loginfo_throttle(1, 'RosbagRecorder: Waiting for data ...')

    def stop_recording(self, folder, prefix="", stamped=False):
        """ Stop the recording process

        This will stop the recording and write the rosbag with a time stamp to
        the given folder. Creates the folder if it doesn't exist. Existing files
        are overwritten.
        """
        if not self.recording:
            return

        self.recording = False
        if not os.path.exists(folder):
            os.makedirs(folder)
        if folder[-1] != '/':
            folder += '/'
        stamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S.%f')[:-3] if stamped else ''
        with rosbag.Bag('{}{}{}.bag'.format(folder, prefix, stamp), 'w') as bag:
            for topic_name, data in self.msg_buffer.items():
                for msg, stamp in data:
                    bag.write(topic_name, msg, stamp)
                self.msg_buffer[topic_name] = []  # reset buffer
