
import sys
sys.path.append('/home/pi/ArmPi/')

from ArmState import ArmState
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board

from scipy.spatial.transform import Rotation as R

STATES = [
        "FIND_CAN",
        'GRAB_CAN',
        "MOVE_CAN",
        "POUR_CAN",
]
from time import sleep
class Motion(ArmState):
    def __init__(self):
        self.ik = ArmIK()
        self.markers = {}
        self.marker_counts = 0

        self.state = STATES[0]

        # My aruco tag is located about 5cm in front of the arm
        self.base_offset = np.array([0, 0, 0])

        # This is the rotation from the pose of the aruco marker on my arm to the
        # world frame of the ArmPi. Pitch (falling towards camera axis) is flipped, and
        # the marker is rotated 90 degrees counterclockwise
        self.roboarm_rotation = R.from_euler('xyz', np.array([np.pi, 0, -np.pi/2])).as_matrix()
    
    def open_gripper(self):
        Board.setBusServoPulse(1, 100, 500)

    def close_gripper(self):
        Board.setBusServoPulse(1, 300, 500)

    def move_to(self, x, y, z):
        if not self.ik.setPitchRangeMoving((x,y,z), 0, -90, 90, 1000):
            print("Can't reach!")
            return False
        return True

    def grab_can(self, can_pos):
        # Target should be about 5cm from can
        unit_vec = can_pos / np.sqrt((can_pos**2).sum())
        target = can_pos - 5 * unit_vec

        print(f"The can is {np.sqrt((target**2).sum())} ({target[0]:5.1f}, {target[1]:5.1f}) from the robot base")
        if not self.move_to(*target[:2], 26):
            return False
        sleep(1)

        self.open_gripper()
        sleep(1)

        if not self.move_to(*target[:2], 10):
            return False
        sleep(1)

        target = can_pos + 1 * unit_vec
        if not self.move_to(*target[:2], 8):
            return False
        sleep(3)

        self.close_gripper()
        sleep(3)

        target = 10 * unit_vec
        if not self.move_to(*target[:2], 26):
            return False
        sleep(3)

        print("success")
        return True


    def step(self, markers):
        # Not sure why this is needed, but sometimes rossros will insist that
        # it has received an int and not a dict
        if type(markers) is not dict:
            return

        print(self.state)
        if self.state == 'FIND_CAN':
            # Aruco readings are noisy, let them settle in before doing anything
            counts = 200
            if self.marker_counts == 0:
                self.markers = {}

            if self.marker_counts < counts:
                if 'base' not in markers:
                    print("base tag not visible")
                elif 'can' not in markers:
                    print("can tag not visible")
                elif 'cup' not in markers:
                    print("cup tag not visible")
                else:
                    self.marker_counts += 1
                    print(f"Collected {self.marker_counts} of {counts} initialization readings")

                for k in markers:
                    if k not in self.markers:
                        self.markers[k] = markers[k]
                    else:
                        self.markers[k] = [0.9 * x + 0.1 * y for (x,y) in zip(self.markers[k], markers[k])]

            else:
                self.state = 'GRAB_CAN'
                self.marker_counts = 0

        elif self.state == 'GRAB_CAN':
            base_R, base_T = self.markers['base']
        
            # Get can rotation and translation
            can_R, can_T = self.markers['can']

            # Get difference in translation between base and can, convert to
            # cm
            can_T = np.array((base_R.T @ (base_T - can_T))).flatten() * 100

            # Aruco markers are z-up or z-out. Since our markers are  facing the camera,
            # that means we will want either x, _, z or _, y, z (x or y correspond to sizeways
            # and z will correspond to towards/away camera). In my setup, it is x, _, z, but
            # depending on the rotation of your markers you may need the other
            can_T = self.roboarm_rotation.T @ np.array([can_T[0], can_T[2] - 0.01, 0]) - self.base_offset

            if self.grab_can(can_T):
                self.state = 'MOVE_CAN'
            self.state = 'FIND_CAN'
        elif self.state == 'MOVE_CAN':
            base_R, base_T = self.markers['base']

            # Get cup rotation and translation
            cup_R, cup_T = self.markers['cup']

            # Get difference in translation between base and cup, convert to
            # cm
            cup_T = np.array((base_R.T @ (base_T - cup_T))).flatten() * 100

            cup_T = self.roboarm_rotation.T @ np.array([cup_T[0], cup_T[2] - 0.01, 20]) - self.base_offset

            if not self.move_to(*cup_T):
                sleep(10)
            self.state = 'FIND_CAN'


if __name__ == '__main__':
    from time import sleep

    # x is sideways, - is away from camera
    # 
    AK = ArmIK()
    print(AK.setPitchRangeMoving((12, 0, 20), 0, -900, 900, 1000))
    sleep(1)
