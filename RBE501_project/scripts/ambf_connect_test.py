from __future__ import print_function
import time
import rospy
from ambf_client import Client


class PSMJointMapping:
    def __init__(self):
        self.idx_to_name = {0: 'baselink-yawlink',
                            1: 'yawlink-pitchbacklink',
                            2: 'pitchendlink-maininsertionlink',
                            3: 'maininsertionlink-toolrolllink',
                            4: 'toolrolllink-toolpitchlink',
                            5: 'toolpitchlink-toolyawlink'
                            # 6: 'toolyawlink-toolgripper1link',
                            # 7: 'toolyawlink-toolgripper2link'
                            }

        self.name_to_idx = {'baselink-yawlink': 0,
                            'yawlink-pitchbacklink': 1,
                            'pitchendlink-maininsertionlink': 2,
                            'maininsertionlink-toolrolllink': 3,
                            'toolrolllink-toolpitchlink': 4,
                            'toolpitchlink-toolyawlink': 5
                            # 'toolyawlink-toolgripper1link' : 6,
                            # 'toolyawlink-toolgripper2link' : 7
                            }

pjm = PSMJointMapping()


class connectionTest():

    def __init__(self, client, name='psm1'):
        self.client = client
        self.name = name
        self.base = self.client.get_obj_handle(name + '/baselink')
        time.sleep(1)
        num_joint = self.base.get_num_joints()
        joint_name = self.base.get_joint_names()
        print(num_joint)
        print(joint_name)

    def servo_jp(self):
        self.base.set_joint_pos(0, 0)
        self.base.set_joint_pos(1, 0)
        self.base.set_joint_pos(2, 0)
        self.base.set_joint_pos(3, 0)
        self.base.set_joint_pos(4, 0)
        self.base.set_joint_pos(5, 0)
        # self.base.set_joint_pos(6, 0)
        # self.base.set_joint_pos(7, 0)

    def servo_jf(self):
        self.base.set_joint_pos(0, 0)
        self.base.set_joint_pos(1, 0)
        self.base.set_joint_effort(2, -49.05)
        self.base.set_joint_pos(3, 0)
        self.base.set_joint_pos(4, 0)
        self.base.set_joint_pos(5, 0)
        # self.base.set_joint_pos(6, 0)
        # self.base.set_joint_pos(7, 0)

    def measured_jp(self):
        j0 = self.base.get_joint_pos(0)
        j1 = self.base.get_joint_pos(1)
        j2 = self.base.get_joint_pos(2)
        j3 = self.base.get_joint_pos(3)
        j4 = self.base.get_joint_pos(4)
        j5 = self.base.get_joint_pos(5)
        return [j0, j1, j2, j3, j4, j5]

    def run(self):
        self.test(jp)



if __name__ == '__main__':
    c = Client()
    c.connect()
    Test = connectionTest(c, 'psm1')
    Test.servo_jp()
    rospy.sleep(1)
    # Test = connectionTest(c, 'psm')
    # input_list = raw_input('?')
    # input_list = str(input_list)
    # input_list = list(input_list.split(','))
    # jp = []
    # for i in input_list:
    #     jp.append(float(i))
    rate = rospy.Rate(200)
    jp = 0
    while not rospy.is_shutdown():
        try:
            Test.servo_jp()
        except KeyboardInterrupt:
            print('stop!')
        rate.sleep()