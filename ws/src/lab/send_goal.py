import rclpy
import json
from std_msgs.msg import String
import time


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('send_goal')

    publisher = node.create_publisher(String, '/set_state', 10)

    msg = String()
    pos = {"replan": False}

    msg.data = json.dumps(pos)
    publisher.publish(msg)

    time.sleep(0.2)

    # set a state here to force replan
    pos = {
        "replan": True,
        "pos_object_1": 1,
        "pos_object_2": 1,
        # write a guard as a string that the guard parser can read
        "goal_as_string": "pos_object_1==2 && pos_object_2==2"
    }

    msg.data = json.dumps(pos)
    publisher.publish(msg)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
