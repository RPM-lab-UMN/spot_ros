from spot_msgs.msg import TaskState

import rospy
import threading

class PersistentPublisherThread(threading.Thread):
    def __init__(self, pub, freq, msg, *args, **kwargs):
        self.pub, self.freq, self.msg = pub, freq, msg
        super().__init__(*args, **kwargs)
        self._stop_event = threading.Event()

    def stop(self):
        self._stop_event.set()

    def _persist(self):
        return not self._stop_event.is_set()

    def run(self):
        rate = rospy.Rate(self.freq)
        while not rospy.is_shutdown() and self._persist():
            self.pub.publish(self.msg)
            rate.sleep()


class TaskStatePublisher:
    
    def __init__(self, topic_name='task_state', frequency=None):
        '''
        Creates an object that can publish task state persistently.
        Args:
            frequency (float): Frequency at which to publish task state. If None,
                then task state is only published when the task state changes.
        '''
        self._f = frequency
        self._thread = None
        self._pub = rospy.Publisher(topic_name, TaskState, queue_size=1)
        rospy.loginfo(f"TaskStatePublisher: Publishing to {topic_name} with frequency {frequency}")

    def _kill_thread(self):
        if self._thread is not None:
            self._thread.stop()
            self._thread.join()
            self._thread = None

    def __call__(self, name, pose, status, terminal=False):
        '''
        Starts a persistent publisher 
        Args:
            name (str): Name of the task.
            pose (list): Pose of the task.
            status (str): 'running', 'success', or 'failure'.
            terminal (bool): Whether the task is terminal.
        '''
        msg = TaskState()
        msg.task_name, msg.target_pose, msg.status = name, pose, status
        self._kill_thread()
        if terminal:
            self._pub.publish(msg)
        else:
            self._thread = PersistentPublisherThread(self._pub, self._f, msg)
            self._thread.start()
        
    def terminal(self, name, pose, state):
        self(name, pose, state, terminal=True)

    def stop(self):
        self._kill_thread()

    def __del__(self):
        self._kill_thread()


    