# robot_adapter.py

import threading

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from nav2_msgs.action import NavigateToPose
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped

from mission_manager.config import ROBOT_AVERAGE_SPEED, POSESTAMPED_FRAME_ID

class RobotAdapter:
    node : Node
    
    def __init__(self, parent_node : Node, robot_id):
        self.node = parent_node
        self.logger = parent_node.get_logger()
        self.robot_id = robot_id
        self.namespace = f"/{robot_id}"
        
        self._client_cb_group = ReentrantCallbackGroup()
        
        action_topic = f"{self.namespace}/navigate_to_pose"
        self.client = ActionClient(
            node=self.node, 
            action_type=NavigateToPose, 
            action_name=action_topic, 
            callback_group=self._client_cb_group
        )
        
        plan_topic = f"{self.namespace}/compute_path_to_pose"
        self.plan_client = ActionClient(
            node=self.node, 
            action_type=ComputePathToPose,
            action_name=plan_topic, 
            callback_group=self._client_cb_group
        )
        
        
        self._goal_response_pending = False
        self._current_handle = None
        
        self.current_goal_pose = None
        self.estimated_time_remaining = None
        self.navigation_time = None
        self.distance_remaining = None
        
        self.lock = threading.RLock()
    
    def send_goal(self, pose, on_success, on_failure):
        with self.lock:
            if self.is_busy():
                raise Exception("Cannot send goal to a robot that is already busy.")
            
            if not self.client.wait_for_server(timeout_sec=1.0):
                self.logger.error(f"Robot {self.robot_id} action server not available.")
                on_failure()
                return
            
            self._goal_response_pending = True
            self.current_goal_pose = pose
            goal_msg = NavigateToPose.Goal()
            ps = PoseStamped()
            ps.header.frame_id = POSESTAMPED_FRAME_ID
            ps.header.stamp = self.node.get_clock().now().to_msg()
            ps.pose = pose
            goal_msg.pose = ps
            
            send_goal_future = self.client.send_goal_async(goal_msg, feedback_callback=self._feedback_callback)
            send_goal_future.add_done_callback(
                lambda f: self._goal_response_callback(f, on_success, on_failure)
            )
        
    def _goal_response_callback(self, future, on_success, on_failure):
        with self.lock:
            self._goal_response_pending = False
            try:
                goal_handle = future.result()
            except Exception as e:
                self.logger.error(f"Robot {self.robot_id}: Service call failed: {e}")
                self.current_goal_pose = None
                on_failure()
                return
            
            if not goal_handle.accepted:
                self.logger.error(f"Robot {self.robot_id}: Goal rejected.")
                self.current_goal_pose = None
                on_failure()
                return
        
            self.logger.info(f"Robot {self.robot_id}: Goal accepted, moving...")
            self._current_handle = goal_handle
            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(
                lambda f: self._get_result_callback(f, on_success, on_failure)
            )
        
    def _get_result_callback(self, future, on_success, on_failure):
        with self.lock:
            result = future.result()
            self.current_goal_pose = None
            self._current_handle = None
            
            match result.status:
                case GoalStatus.STATUS_SUCCEEDED:
                    self.logger.info(f"Robot {self.robot_id}: Goal reached.")
                    on_success()
                case GoalStatus.STATUS_ABORTED:
                    self.logger.error(f"Robot {self.robot_id}: Goal aborted.")
                    on_failure()
                case GoalStatus.STATUS_CANCELED:
                    self.logger.error(f"Robot {self.robot_id}: Goal canceled.")
                    on_failure()
                case _:
                    self.logger.error(f"Robot {self.robot_id}: Unknown result status : {result.status}.")
                    on_failure()
        
    def _feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.estimated_time_remaining = feedback.estimated_time_remaining.sec + feedback.estimated_time_remaining.nanosec * 1e-9
        self.navigation_time = feedback.navigation_time.sec + feedback.navigation_time.nanosec * 1e-9
        self.distance_remaining = feedback.distance_remaining
        
    def is_busy(self):
        with self.lock:
            return self._goal_response_pending or self._current_handle is not None
    
    def is_navigating(self):
        with self.lock:
            return self._current_handle is not None
    
    def cancel_goal(self):
        with self.lock:
            if self._current_handle is not None:
                self._current_handle.cancel_goal_async()
                self.current_goal_pose = None

    def destroy(self):
        with self.lock:
            self.current_goal_pose = None
            self.client.destroy()
            
    def get_current_goal(self):
        with self.lock:
            if self.is_busy():
                return self.current_goal_pose
            return None 
        

    def make_plan_async(self, start_pose_msg, goal_pose_msg):
        if not hasattr(start_pose_msg, 'header'):
            start_pose_msg = self._to_pose_stamped(start_pose_msg)
        if not hasattr(goal_pose_msg, 'header'):
            goal_pose_msg = self._to_pose_stamped(goal_pose_msg)
            

        goal_msg = ComputePathToPose.Goal()
        goal_msg.start = start_pose_msg
        goal_msg.goal = goal_pose_msg
        goal_msg.use_start = True

        server_ready = self.plan_client.wait_for_server(timeout_sec=0.5)
        if not server_ready:
            self.logger.error(f"ComputePath server not available for robot {self.robot_id}")
            return None
        # Asynchronous sending
        # Send a future
        return self.plan_client.send_goal_async(goal_msg)
    
    def _to_pose_stamped(self, pose_msg):
        ps = PoseStamped()
        ps.header.frame_id = POSESTAMPED_FRAME_ID
        ps.header.stamp = self.node.get_clock().now().to_msg()
        # Si c'est déjà une PoseStamped, on prend .pose, sinon on prend l'objet tel quel
        if hasattr(pose_msg, 'pose'):
             ps.pose = pose_msg.pose
        else:
             ps.pose = pose_msg
        return ps