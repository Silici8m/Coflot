# Copyright 2024 Kazuhiro Ouchi @kanpapa
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class KobukiCmdvelSubscriber(Node):

    def __init__(self):
        super().__init__('kobuki_cmdvel_subscriber')
        self.publisher_ = self.create_publisher(Twist, 'commands/velocity', 10)
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    kobuki_cmdvel_subscriber = KobukiCmdvelSubscriber()
    rclpy.spin(kobuki_cmdvel_subscriber)
    kobuki_cmdvel_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
