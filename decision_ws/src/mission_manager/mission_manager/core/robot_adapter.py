# robot_adapter.py

import threading
from typing import Any, Callable, Optional, Union

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from nav2_msgs.action import NavigateToPose
from nav2_msgs.action import ComputePathToPose
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped

from mission_manager.config import POSESTAMPED_FRAME_ID

class RobotAdapter:
    """
    Interface between the Mission Manager and a specific robot's navigation stack.

    This class encapsulates the ROS 2 Action Clients required to send navigation goals
    and compute paths for a single robot. It manages the lifecycle of a navigation
    action (sending, feedback, result) and provides thread-safe status checks.

    Attributes:
        robot_id (str): Unique identifier of the robot.
        namespace (str): ROS namespace of the robot (e.g., "/robot_1").
        current_goal_pose (Optional[Pose]): The target pose currently being pursued.
        estimated_time_remaining (Optional[float]): Time remaining in seconds (from feedback).
        navigation_time (Optional[float]): Time elapsed since start in seconds (from feedback).
        distance_remaining (Optional[float]): Distance to goal in meters (from feedback).
    """
    node: Node
    
    def __init__(self, parent_node: Node, robot_id: str) -> None:
        """
        Initializes the RobotAdapter.

        Args:
            parent_node (Node): The ROS 2 node managing this adapter.
            robot_id (str): The unique identifier of the robot to control.
        """
        self.node = parent_node
        self.logger = parent_node.get_logger()
        self.robot_id = robot_id
        self.namespace = f"/{robot_id}"
        
        self._client_cb_group = ReentrantCallbackGroup()
        
        # Action Client for navigation (moving the robot)
        action_topic = f"{self.namespace}/navigate_to_pose"
        self.client = ActionClient(
            self.node, 
            NavigateToPose, 
            action_topic, 
            callback_group=self._client_cb_group
        )
        
        # Action Client for path planning (calculating costs without moving)
        plan_topic = f"{self.namespace}/compute_path_to_pose"
        self.plan_client = ActionClient(
            self.node, 
            ComputePathToPose,
            plan_topic, 
            callback_group=self._client_cb_group
        )
        
        self._goal_response_pending = False
        self._current_handle = None
        
        self.current_goal_pose: Optional[Pose] = None
        self.estimated_time_remaining: Optional[float] = None
        self.navigation_time: Optional[float] = None
        self.distance_remaining: Optional[float] = None
        
        self.lock = threading.RLock()
    
    def send_goal(self, pose: Pose, on_success: Callable[[], None], on_failure: Callable[[], None]) -> None:
        """
        Sends a navigation goal to the robot asynchronously.

        Constructs a NavigateToPose goal and sends it to the action server.
        Registers callbacks for acceptance and result handling.

        Args:
            pose (Pose): The target geometry_msgs/Pose.
            on_success (Callable): Callback executed if navigation succeeds.
            on_failure (Callable): Callback executed if navigation fails, is aborted, or rejected.

        Raises:
            Exception: If the robot is already busy with another goal.
        """
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
        
    def _goal_response_callback(self, future: Future, on_success: Callable[[], None], on_failure: Callable[[], None]) -> None:
        """
        Internal callback handling the server's response to the goal request.

        Determines if the goal was accepted or rejected by the Nav2 server.

        Args:
            future (Future): The future object containing the goal handle.
            on_success (Callable): Success callback to propagate.
            on_failure (Callable): Failure callback to propagate.
        """
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
        
            # self.logger.info(f"Robot {self.robot_id}: Goal accepted, moving...")
            self._current_handle = goal_handle
            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(
                lambda f: self._get_result_callback(f, on_success, on_failure)
            )
        
    def _get_result_callback(self, future: Future, on_success: Callable[[], None], on_failure: Callable[[], None]) -> None:
        """
        Internal callback handling the final result of the navigation action.

        Args:
            future (Future): The future object containing the result.
            on_success (Callable): Success callback to execute.
            on_failure (Callable): Failure callback to execute.
        """
        with self.lock:
            result = future.result()
            self.current_goal_pose = None
            self._current_handle = None
            
            match result.status:
                case GoalStatus.STATUS_SUCCEEDED:
                    #self.logger.info(f"Robot {self.robot_id}: Goal reached.")
                    on_success()
                case GoalStatus.STATUS_ABORTED:
                    self.logger.error(f"Robot {self.robot_id}: Goal aborted.")
                    on_failure()
                case GoalStatus.STATUS_CANCELED:
                    self.logger.warning(f"Robot {self.robot_id}: Goal canceled.")
                    on_failure()
                case _:
                    self.logger.error(f"Robot {self.robot_id}: Unknown result status : {result.status}.")
                    on_failure()
        
    def _feedback_callback(self, feedback_msg: Any) -> None:
        """
        Internal callback handling periodic feedback from the navigation server.

        Updates estimated time and distance remaining.

        Args:
            feedback_msg (Any): The feedback message from NavigateToPose action.
        """
        feedback = feedback_msg.feedback
        self.estimated_time_remaining = feedback.estimated_time_remaining.sec + feedback.estimated_time_remaining.nanosec * 1e-9
        self.navigation_time = feedback.navigation_time.sec + feedback.navigation_time.nanosec * 1e-9
        self.distance_remaining = feedback.distance_remaining
        
    def is_busy(self) -> bool:
        """
        Checks if the robot is currently processing a goal.

        Returns:
            bool: True if waiting for a goal response or if a goal is active.
        """
        with self.lock:
            return self._goal_response_pending or self._current_handle is not None
    
    def is_navigating(self) -> bool:
        """
        Checks if the robot is actively navigating (goal accepted and running).

        Returns:
            bool: True if a valid goal handle exists.
        """
        with self.lock:
            return self._current_handle is not None
    
    def cancel_goal(self) -> None:
        """
        Requests cancellation of the current navigation goal.
        """
        with self.lock:
            if self._current_handle is not None:
                self._current_handle.cancel_goal_async()
                self.current_goal_pose = None

    def destroy(self) -> None:
        """
        Cleans up resources and destroys the action client.
        """
        with self.lock:
            self.current_goal_pose = None
            self.client.destroy()
            
    def get_current_goal(self) -> Optional[Pose]:
        """
        Retrieves the current navigation goal.

        Returns:
            Optional[Pose]: The current target pose, or None if idle.
        """
        with self.lock:
            if self.is_busy():
                return self.current_goal_pose
            return None 
        

    def make_plan_async(self, start_pose_msg: Union[Pose, PoseStamped], goal_pose_msg: Union[Pose, PoseStamped]) -> Optional[Future]:
        """
        Requests a path plan computation from the Nav2 server asynchronously.

        Used to estimate travel costs without moving the robot.

        Args:
            start_pose_msg (Union[Pose, PoseStamped]): The starting position.
            goal_pose_msg (Union[Pose, PoseStamped]): The destination position.

        Returns:
            Optional[Future]: A Future object for the action goal, or None if server is unavailable.
        """
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
    
    def _to_pose_stamped(self, pose_msg: Union[Pose, PoseStamped]) -> PoseStamped:
        """
        Converts a Pose or PoseStamped message into a standardized PoseStamped.

        Adds the current timestamp and configured frame ID if missing.

        Args:
            pose_msg (Union[Pose, PoseStamped]): The input pose message.

        Returns:
            PoseStamped: The standardized message.
        """
        ps = PoseStamped()
        ps.header.frame_id = POSESTAMPED_FRAME_ID
        ps.header.stamp = self.node.get_clock().now().to_msg()
        # Si c'est déjà une PoseStamped, on prend .pose, sinon on prend l'objet tel quel
        if hasattr(pose_msg, 'pose'):
             ps.pose = pose_msg.pose
        else:
             ps.pose = pose_msg
        return ps