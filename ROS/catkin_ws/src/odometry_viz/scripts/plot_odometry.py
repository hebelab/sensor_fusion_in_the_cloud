#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt

class OdometryPlotter:
    def __init__(self):
        rospy.init_node('odometry_plotter', anonymous=True)
        self.odom_subscriber = rospy.Subscriber('/integrated_to_init', Odometry, self.odom_callback)
        self.positions_x = []
        self.positions_y = []

        plt.ion()
        plt.show()

        # Set up a timer to update the plot at a fixed rate
        self.plot_update_rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.update_plot()
            self.plot_update_rate.sleep()

    def odom_callback(self, msg):
        # Extract x and y positions from the Odometry message
        position_x = msg.pose.pose.position.x
        position_y = msg.pose.pose.position.y

        # Add the positions to the lists
        self.positions_x.append(position_x)
        self.positions_y.append(position_y)

    def update_plot(self):
        # Update the plot
        plt.clf()
        plt.plot(self.positions_x, self.positions_y, 'b-')
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.title('Odometry Data')
        plt.grid(True)
        plt.draw()
        plt.pause(0.001)

if __name__ == '__main__':
    try:
        OdometryPlotter()
    except rospy.ROSInterruptException:
        pass
