import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from apriltag_msgs.msg import AprilTagDetectionArray
from sensor_msgs.msg import CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from transforms3d.euler import quat2euler
from transforms3d.quaternions import qinverse, qmult


class Lab5(Node):
    dt = 0.02
    tag_size = 0.2
    particles = 20
    fiducials = 18
    Qt = np.diag([2, 5])

    def __init__(self):
        super().__init__("lab5_node")

        self.create_timer(self.dt, self.timer_callback)
        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            5,
        )
        self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.detection_callback,
            5,
        )
        self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            5,
        )
        self.viz_publisher = self.create_publisher(
            MarkerArray,
            '/viz',
            5,
        )
        self.random = np.random.default_rng()
        self.camera_P = None
        self.z = np.ndarray((3, 0))
        self.x = np.zeros((self.particles, 1, 3, 1))
        self.mu = np.nan * np.ones((self.particles, self.fiducials, 2, 1))
        self.sigma = np.nan * np.ones((self.particles, self.fiducials, 2, 2))
        self.w = np.ones((self.particles,)) / self.particles
        self.last_odom = None
        self.odom = None

    @staticmethod
    def odometry2loc(msg):
        return np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        ])

    @staticmethod
    def odometry2quat(msg):
        return np.array([
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
        ])

    def quat_diff(self, a, b):
        if isinstance(a, Odometry):
            a = self.odometry2quat(a)
        if isinstance(b, Odometry):
            b = self.odometry2quat(b)
        return qmult(a, qinverse(b))

    def get_input(self):
        if self.odom is None:
            return 0, 0
        if self.last_odom is None:
            self.last_odom = self.odom
        dtheta = quat2euler(self.quat_diff(self.odom, self.last_odom))[2]
        dx = self.odometry2loc(self.odom) - self.odometry2loc(self.last_odom)
        self.last_odom = self.odom
        return np.linalg.norm(dx) / self.dt, dtheta / self.dt

    def timer_callback(self):
        v, w = self.get_input()
        v += 0.04 * self.random.normal(size=(self.particles, 1, 1))
        w += 0.02 * self.random.normal(size=(self.particles, 1, 1))
        theta = self.x[:, :, 2]
        self.x += np.stack([
            -v/w * np.sin(theta) + v/w * np.sin(theta + w * self.dt),
            v/w * np.cos(theta) - v/w * np.cos(theta + w * self.dt),
            w * self.dt,
        ], axis=2)
        # Your Code Here
        self.z = np.ndarray((3, 0))
        self.publish_estimate()
        self.resample()

    @staticmethod
    def h(mu, x):
        return np.stack([
            np.linalg.norm(mu - x[:,0,:2], axis=1),
            x[:,0,2] - np.arctan2(mu[:,1] - x[:,0,1], mu[:,0] - x[:,0,0])
        ], axis=1)

    @staticmethod
    def h_prime(x, mu):
        q2 = (mu[:,0,0] - x[:,0,0,0])**2 + (mu[:,1,0] - x[:,0,1,0])**2
        q = np.sqrt(q2)
        return np.array([
            [
                (x[:,0,0,0] - mu[:,0,0]) / q,
                (x[:,0,1,0] - mu[:,1,0]) / q,
            ],
            [
                (x[:,0,1,0] - mu[:,1,0]) / q2,
                (mu[:,0,0] - x[:,0,0,0]) / q2,
            ]
        ]).transpose((2,0,1))

    @staticmethod
    def h_inv(z, x):
        return x[:,0,:2] + z[0] * np.stack([
            np.cos(-z[1] + x[:,0,2]),
            np.sin(-z[1] + x[:,0,2]),
        ], axis=1)


    def resample(self):
        if np.any(np.isnan(self.w)):
            return
        if self.w.min() / self.w.max() < 0.5:
            ind = tuple(np.random.choice(self.particles, size=(self.particles,), p=self.w))
            self.x = self.x[ind,]
            self.mu = self.mu[ind,]
            self.sigma = self.sigma[ind,]
            self.w = self.w[ind,]

    def publish_estimate(self):
        mean = np.sum(self.w.reshape(-1, 1, 1, 1) * self.mu, axis=0).reshape(-1, 2)
        map = MarkerArray()
        for i in range(self.fiducials):
            marker = Marker()
            marker.id = i
            marker.ns = "map"
            marker.type = 9
            marker.action = 0
            marker.text = str(i)
            marker.scale.z = 0.5
            marker.pose.position.x = mean[i, 0]
            marker.pose.position.y = mean[i, 1]
            marker.pose.position.z = 0.0
            marker.header.frame_id = "odom"
            marker.color.a=1.0
            marker.color.b=1.0
            map.markers.append(marker)
        self.viz_publisher.publish(map)
        # mean = np.sum(self.w.reshape(-1, 1, 1, 1) * self.x, axis=0).flatten()
        particles = MarkerArray()
        for i in range(self.particles):
            particle = Marker()
            particle.id = i
            particle.ns = "particles"
            particle.type = 0
            particle.action = 0
            particle.scale.x = 0.07
            particle.scale.y = 0.15
            particle.scale.z = 0.0
            x = self.x[i,0,:,0]
            particle.points = [
                Point(x=x[0], y=x[1]),
                Point(x=x[0] + 0.25 * np.cos(x[2]), y=x[1] + 0.25 * np.sin(x[2])),
            ]
            particle.header.frame_id = "odom"
            particle.color.a = 1.0
            particle.color.r = 1.0
            particles.markers.append(particle)
        self.viz_publisher.publish(particles)

    def odom_callback(self, msg):
        self.odom = msg

    def camera_info_callback(self, msg):
        self.camera_P = msg.p.reshape((3, 4))[:, :3]

    def detection_callback(self, msg):
        if self.camera_P is None:
            return
        self.z = np.zeros((3, len(msg.detections)))
        for i, detection in enumerate(msg.detections):
            T = np.linalg.inv(self.camera_P) @ detection.homography.reshape((3, 3))
            tt = T[:, 2] / np.linalg.norm(T[:, 0]) * self.tag_size / 2
            self.z[:, i] = (
                tt[2],
                np.arctan2(tt[0], tt[2]),
                detection.id,
            )


def main(args=None):
    rclpy.init(args=args)
    node = Lab5()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
