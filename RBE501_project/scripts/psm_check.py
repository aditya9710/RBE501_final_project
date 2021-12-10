import rospy
from ambf_client import Client
import time

def handle_psm_joint_cmds():
	_client = Client()

	_client.connect()

	# You can print the names of objects found
	print("*****Object Names*****\n", _client.get_obj_names())

	psm1_handle = _client.get_obj_handle('/ambf/env/psm1/baselink')

	time.sleep(2)

	num_joints = psm1_handle.get_num_joints()
	print("*****PSM Num of Joints*****\n", num_joints)

	if (num_joints != 0):
		input("*****Track a path***** Press Enter . . .")
		for joint in range(0, num_joints+1):
			for i in range(0,10,1):
				psm1_handle.set_joint_pos(joint, 0.1+(i/10))
				rospy.sleep(0.2)
			rospy.sleep(1)

		input("*****Try efforts***** Press Enter . . .")
		for joint in range(0, num_joints+1):
			for i in range(0,5,1):
				psm1_handle.set_joint_effort(joint, 5+(i*5))
				rospy.sleep(0.2)
	else:
		print("*****Failed*****")

	_client.clean_up()


if __name__ == '__main__':
	try:
		rospy.init_node('psm_cmd_server', anonymous=True)
		print("Ready to send cmds to PSM")
		handle_psm_joint_cmds()
		# rospy.spin()

	except rospy.ROSInterruptException:
		pass
