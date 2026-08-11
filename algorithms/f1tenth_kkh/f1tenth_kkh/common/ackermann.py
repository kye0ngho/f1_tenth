"""AckermannDriveStamped helpers shared by all f1tenth_kkh controllers."""

from ackermann_msgs.msg import AckermannDriveStamped


def build_ackermann(clock, frame_id, speed, steering) -> AckermannDriveStamped:
    message = AckermannDriveStamped()
    message.header.stamp = clock.now().to_msg()
    message.header.frame_id = frame_id
    message.drive.speed = float(speed)
    message.drive.steering_angle = float(steering)
    return message


def publish_stop(publisher, clock, frame_id):
    publisher.publish(build_ackermann(clock, frame_id, 0.0, 0.0))
