from __future__ import print_function
import time
import rospy
from ambf_client import Client
import os

#### Mapping for psm_full
class PSMJointMappingFull:
    def __init__(self):
        self.idx_to_name = {0: 'baselink-yawlink',
                            1: 'yawlink-pitchbacklink',
                            2: 'pitchendlink-maininsertionlink',
                            3: 'maininsertionlink-toolrolllink',
                            4: 'toolrolllink-toolpitchlink',
                            5: 'toolpitchlink-toolgripper1link',
                            6: 'toolpitchlink-toolgripper2link',
                            }

        self.name_to_idx = {'baselink-yawlink': 0,
                            'yawlink-pitchbacklink': 1,
                            'pitchendlink-maininsertionlink': 2,
                            'maininsertionlink-toolrolllink': 3,
                            'toolrolllink-toolpitchlink': 4,
                            'toolpitchlink-toolgripper1link': 5,
                            'toolpitchlink-toolgripper2link': 6,
                            }

pjm = PSMJointMappingFull()


class connectionTestFull():

    def __init__(self, client, name='psm1', csv_file_name='JointPos.csv'):
        self.client = client
        self.name = name
        self.parent_folder = os.getcwd()
        self.csv_name = csv_file_name
        self.base = self.client.get_obj_handle(name + '/baselink')
        self.tip = self.client.get_obj_handle(name + '/toolgripper1link')
        time.sleep(2)
        num_joint = self.base.get_num_joints()
        joint_name = self.base.get_joint_names()
        self.jp_values = []
        self.counter = 0
        print(num_joint)
        print(joint_name)

    def load_data(self):
        csv_file = self.parent_folder + '/' + self.csv_name
        csv_output = []
        #
        with open(csv_file) as f:
            content = f.readlines()
            csv_modify = [[]] * 6
            csv_output = []
            for num_joint in range(6):
                csv_modify[num_joint] = content[num_joint].split('\n')[0].split(',')
                # csv_modify[num_joint] = csv_modify[num_joint]

        for num_pts in range(len(csv_modify[0])):
            jp1 = float(csv_modify[0][num_pts])
            jp2 = float(csv_modify[1][num_pts])
            jp3 = float(csv_modify[2][num_pts])
            jp4 = float(csv_modify[3][num_pts])
            jp5 = float(csv_modify[4][num_pts])
            jp6 = float(csv_modify[5][num_pts])
            jp_v = [jp1, jp2, jp3, jp4, jp5, jp6]
            csv_output.append(jp_v)
        self.jp_values = csv_output
        return csv_output

    def servo_jp(self, jp):
        self.base.set_joint_pos(0, jp[0])
        self.base.set_joint_pos(1, jp[1])
        self.base.set_joint_pos(2, jp[2])
        self.base.set_joint_pos(3, jp[3])
        self.base.set_joint_pos(4, jp[4])
        self.base.set_joint_pos(5, 0)
        self.base.set_joint_pos(6, 0)
        rospy.sleep(0.01)

    def measured_jp(self):
        j0 = self.base.get_joint_pos(0)
        j1 = self.base.get_joint_pos(1)
        j2 = self.base.get_joint_pos(2)
        j3 = self.base.get_joint_pos(3)
        j4 = self.base.get_joint_pos(4)
        j5 = self.base.get_joint_pos(5)
        j6 = self.base.get_joint_pos(6)
        return [j0, j1, j2, j3, j4, j5, j6]

    def update_pose(self):
        num_data = len(self.jp_values)
        if self.counter == num_data:
            print('Finish  running! \n')
            self.counter = self.counter + 1
        elif self.counter < num_data:
            self.servo_jp(self.jp_values[self.counter])
            self.counter = self.counter + 1
            a = self.tip.get_pos()
            # print(a.x)
        else:
            pass


    def run(self):
        self.load_data()
        self.update_pose()



if __name__ == '__main__':
    name_file = 'JointPos_full_circle.csv'
    # name_file = 'JointPos_full_line.csv'
    c = Client()
    c.connect()
    Test = connectionTestFull(c, 'psm', name_file)
    rate = rospy.Rate(200)
    # # jp = 0
    while not rospy.is_shutdown():
        try:
            Test.run()
        except KeyboardInterrupt:
            print('stop!')
        rate.sleep()
