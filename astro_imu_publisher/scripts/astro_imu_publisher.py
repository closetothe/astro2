#!/usr/bin/env python
'''  
 *	Jamiel Rahi
 *  Astro 2.0 Capstone Team 14
 *  2018-2019
 *  License: GPL
 *  ----------------------
 *  ---- GET IMU DATA ----
 *  ----------------------
 *  This is a driver for transferring data from the
 *  Adafruit BNO055 to ROS (and ultimately robot_localization).
 *  
 *  It uses a modified version of the Adafruit BNO055 library
 *  with calibration functions, both created by "davegun"
 *  at Adafruit forums.
 *  
 *  offsets.h lists calibration data obtained beforehand using
 *  Dave's functions. See the "test" folder for more.
 *
 *  It publishes quaternion, accel, and gyro to imu/data.
 *  It also publishes temperature to imu/temp.
 *  Finally it also publishes the calibration statuses of each
 *  component (sys, accel, gyro, mag) to imu/status 
 *
'''

import rospy
from std_msgs.msg import Header as Header
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu as Imu
from sensor_msgs.msg import Temperature as Temperature
from sensor_msgs.msg import MagneticField as MagneticField
from geometry_msgs.msg import PoseWithCovarianceStamped as PoseWithCovarianceStamped
from diagnostic_msgs.msg import DiagnosticArray as DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus as DiagnosticStatus

pi = 3.14159265358979323846

class AstroImuPublisher :
	def __init__(self):
		rospy.init_node("astro_imu_publisher")
		rospy.loginfo("Astro IMU Publisher")
		self.rate = rospy.Rate(rospy.get_param("~rate", 100.))
		self.sub = rospy.Subscriber('imu/raw', Float32MultiArray, self.raw_data_callback)
		rospy.loginfo("Setup subscriber on imu/raw [std_msgs/Float32MultiArray]")
		self.imu_data_pub = rospy.Publisher('imu/data', Imu, queue_size=100)
		rospy.loginfo("Setup publisher on imu/data [sensor_msgs/Imu]")
		self.imu_pose_pub = rospy.Publisher('imu/pose', PoseWithCovarianceStamped, queue_size=10)
		rospy.loginfo("Setup publisher on imu/pose [geomoetry_msgs/PoseWithCovarianceStamped]")
		self.imu_mag_pub = rospy.Publisher('imu/mag', MagneticField, queue_size=5)
		rospy.loginfo("Setup publisher on imu/mag [sensor_msgs/MagneticField]")		
		self.imu_temp_pub = rospy.Publisher('imu/temp', Temperature, queue_size=1)	
		rospy.loginfo("Setup publisher on imu/temp [sensor_msgs/Temperature]")	
		self.imu_status_pub = rospy.Publisher('imu/status', DiagnosticArray, queue_size=10)	
		rospy.loginfo("Setup publisher on imu/status [diagnostic_msgs/DiagnosticArray]")

		default_covar = [0,0,0,0,0,0,0,0,0]

		# Initialize Imu msg for imu/data
		self.imu_data = Imu()
		self.imu_data.header.frame_id = "imu_link"

		if not rospy.has_param("~orientation_covariance") :
			rospy.logwarn("No orientation covariance provided. Defaulting to zeroes...")
		if not rospy.has_param("~linear_acceleration_covariance") :
			rospy.logwarn("No linear acceleration covariance provided. Defaulting to zeroes...")
		if not rospy.has_param("~angular_velocity_covariance") :
			rospy.logwarn("No angular velocity covariance provided. Defaulting to zeroes...")

		self.imu_data.orientation_covariance = rospy.get_param("~orientation_covariance", default_covar)
		self.imu_data.linear_acceleration_covariance =  rospy.get_param("~linear_acceleration_covariance", default_covar)
		self.imu_data.angular_velocity_covariance = rospy.get_param("~angular_velocity_covariance", default_covar)
		
		# Initialize MagneticField msg for imu/mag
		self.imu_mag = MagneticField()
		self.imu_mag.header.frame_id = "imu_link"
		self.imu_mag.magnetic_field_covariance = rospy.get_param("~magnetic_field_covariance", default_covar)

		# Initialize Temperature msg for imu/temp
		self.imu_temp = Temperature()
		self.imu_temp.header.frame_id = "imu_link"
		self.imu_temp.variance = rospy.get_param("~temperature_variance", 0)

		# Initialize PoseWithCovarianceStamped for imu/pose
		d00 = 0
		d11 = 0
		d22 = 0

		if rospy.has_param("~orientation_covariance"):
			d00 = d11 = d22 = 1000

		d33 = self.imu_data.orientation_covariance[0]
		d44 = self.imu_data.orientation_covariance[4]
		d55 = self.imu_data.orientation_covariance[8]
		

		pose_covariance = [ d00,0,0,0,0,0,
							0,d11,0,0,0,0,
							0,0,d22,0,0,0,
							0,0,0,d33,0,0,
							0,0,0,0,d44,0,
							0,0,0,0,0,d55 ]
		self.imu_pose = PoseWithCovarianceStamped()
		self.imu_pose.header.frame_id = "imu_link"
		self.imu_pose.pose.covariance = pose_covariance

		# Initialize DiagnosticArray for imu/status
		self.imu_status = DiagnosticArray()
		self.imu_status.header.frame_id = "imu_link"
		self.imu_status.status = [DiagnosticStatus(), DiagnosticStatus(),
								  DiagnosticStatus(), DiagnosticStatus()]
		self.imu_status.status[0].level = 0
		self.imu_status.status[0].name = "System Calibration"
		self.imu_status.status[0].hardware_id = "BNO055"
		self.imu_status.status[1].level = 0
		self.imu_status.status[1].name = "Accelerometer Calibration"
		self.imu_status.status[1].hardware_id = "BNO055"		
		self.imu_status.status[2].level = 0
		self.imu_status.status[2].name = "Gyroscope Calibration"
		self.imu_status.status[2].hardware_id = "BNO055"		
		self.imu_status.status[3].level = 0
		self.imu_status.status[3].name = "Magnetomer Calibration"
		self.imu_status.status[3].hardware_id = "BNO055"

		while not rospy.is_shutdown():
			self.rate.sleep()


	def publish(self):
		self.imu_data_pub.publish(self.imu_data)
		self.imu_pose_pub.publish(self.imu_pose)
		self.imu_mag_pub.publish(self.imu_mag)
		self.imu_temp_pub.publish(self.imu_temp)
		self.imu_status_pub.publish(self.imu_status)


	def raw_data_callback(self, msg):
		# Quaternion
		if msg.data[0] == msg.data[1] == msg.data[2] == msg.data[3] == 0:
			rospy.logwarn("Non-normalized quaternion. Setting to 0 0 0 1")
			self.imu_data.orientation.w = 1
		else :
			self.imu_data.orientation.w = msg.data[0]
		self.imu_data.orientation.x = msg.data[1]
		self.imu_data.orientation.y = msg.data[2]
		self.imu_data.orientation.z = msg.data[3]
		self.imu_pose.pose.pose.orientation = self.imu_data.orientation
		# Euler (convert to radians)
		self.imu_pose.pose.pose.position.x = msg.data[4]*pi/180 # yaw
		self.imu_pose.pose.pose.position.y = msg.data[5]*pi/180 # pitch
		self.imu_pose.pose.pose.position.z = msg.data[6]*pi/180 # roll
		# Linear acceleration
		self.imu_data.linear_acceleration.x = msg.data[7]
		self.imu_data.linear_acceleration.y = msg.data[8]
		self.imu_data.linear_acceleration.z = msg.data[9]
		# Angular velocity
		self.imu_data.angular_velocity.x = msg.data[10]
		self.imu_data.angular_velocity.y = msg.data[11]
		self.imu_data.angular_velocity.z = msg.data[12]
		# Magnetic field (convert from uT to T)
		self.imu_mag.magnetic_field.x = msg.data[13]*1.0e-6
		self.imu_mag.magnetic_field.y = msg.data[14]*1.0e-6
		self.imu_mag.magnetic_field.z = msg.data[15]*1.0e-6		
		# System calibration status
		self.imu_status.status[0].level = int(msg.data[16])
		# Accel calibration status
		self.imu_status.status[1].level = int(msg.data[17])
		# Gyro calibration status
		self.imu_status.status[2].level = int(msg.data[18])
		# Mag calibration status
		self.imu_status.status[3].level = int(msg.data[19])
		# Temperature
		self.imu_temp.temperature = msg.data[20]

		# Stamp everything
		now = rospy.Time.now()
		self.imu_data.header.stamp = now
		self.imu_pose.header.stamp = now
		self.imu_mag.header.stamp = now
		self.imu_status.header.stamp = now
		self.imu_temp.header.stamp = now

		# Publish immediately in the callback
		# (to keep the sub/pub in sync)
		self.publish()


if __name__ == '__main__':
	try: 
		imu_publisher = AstroImuPublisher()
	except rospy.ROSInterruptException:  
		pass













