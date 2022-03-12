
import sys
sys.path.append('/home/pi/ArmPi/')

from ArmState import ArmState
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board

from scipy.spatial.transform import Rotation as R

STATES = [
        'GRAB_CAN',
        "MOVE_CAN",
        "POUR_CAN",
]
from time import sleep, time
class Motion(ArmState):
    def __init__(self):
        self.ik = ArmIK()

        self.state = STATES[0]

        # My aruco tag is located about 5cm in front of the arm
        self.base_offset = np.array([-11, 2, 0])
        self.warmup_ticks = 0

        # This is the rotation from the pose of the aruco marker on my arm to the
        # world frame of the ArmPi. Pitch (falling towards camera axis) is flipped, and
        # the marker is rotated 90 degrees counterclockwise
        self.roboarm_rotation = R.from_euler('xyz', np.array([np.pi, 0, -np.pi/2])).as_matrix()
    
    def open_gripper(self):
        Board.setBusServoPulse(1, 200, 500)

    def close_gripper(self):
        Board.setBusServoPulse(1, 400, 500)

    def tilt_gripper(self):
        Board.setBusServoPulse(2, 1000, 1000)

    def untilt_gripper(self):
        Board.setBusServoPulse(2, 500, 1000)

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

        if not self.move_to(*target[:2], 14):
            return False
        sleep(1)

        target = can_pos + 0 * unit_vec
        if not self.move_to(*target[:2], 14):
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

        if self.state == 'GRAB_CAN':
            self.close_gripper()
            self.state = 'MOVE_CAN'

        print(self.state)
        if 'cup' in markers and 'base' in markers:
            base_R, base_T = markers['base']
            # Get can rotation and translation
            cup_R, cup_T = markers['cup']

            # Center of cup is about 5cm behind marker for me
            cup_T = np.array(cup_T + cup_R @ np.array([0, 0, -0.05])).flatten()

            # Get difference in translation between base and can, convert to
            # cm
            cup_T = np.array((base_R.T @ (base_T - cup_T))).flatten() * 100

            # Aruco markers are z-up or z-out. Since our markers are  facing the camera,
            # that means we will want either x, _, z or _, y, z (x or y correspond to sideways
            # and z will correspond to towards/away from camera). In my setup, it is x, _, z, but
            # depending on the rotation of your markers you may need the other
            cup_T = self.roboarm_rotation.T @ np.array([cup_T[0], cup_T[2], 0]) - self.base_offset

            # Get the normal vector of the line pointing from the robot base to the cup
            orthogonal_unit_vector = np.array([-cup_T[1], cup_T[0], 0]) / np.sqrt((cup_T**2).sum())

            if self.state == 'MOVE_CAN':
                # Using the normal of the vector from the base to the cup, place
                # the gripper slightly to the side of the cup to get in pouring
                # position
                target = cup_T + orthogonal_unit_vector * 10#cm
                self.move_to(*target[:2], 22)

                # Mark the time when we started pouring so we know when to stop
                self.state = 'POUR_CAN'
                self.pour_start = time()
                return

            if self.state == 'POUR_CAN':
                target = cup_T + orthogonal_unit_vector * 6#cm
                self.move_to(*target[:2], 16)
                self.tilt_gripper()
                if time() > self.pour_start + 7:
                    self.state = 'DONE'
                    target = cup_T + orthogonal_unit_vector * 10#cm
                    self.move_to(*target[:2], 22)
                    sleep(0.5)
                    self.untilt_gripper()
                return

            if self.state == 'DONE':
                print('Finished')
                self.move_to(12, 0, 20)

        else:
            print("Detected markers: ", list(markers.keys()))
            return
        return
        if False or self.state == 'GRAB_CAN':
            raise RuntimeError
            base_R, base_T = markers['base']
        
            # Get can rotation and translation
            can_R, can_T = markers['can']

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
        elif self.state == 'MOVE_CAN':
            sleep(10)
            exit(0)
            base_R, base_T = markers['base']

            # Get cup rotation and translation
            cup_R, cup_T = markers['cup']

            # Get difference in translation between base and cup, convert to
            # cm
            cup_T = np.array((base_R.T @ (base_T - cup_T))).flatten() * 100

            cup_T = self.roboarm_rotation.T @ np.array([cup_T[0], cup_T[2] - 0.01, 20]) - self.base_offset

            if not self.move_to(*cup_T):
                sleep(10)
            self.state = 'FIND_CAN'


if __name__ == '__main__':

    # x is sideways, - is away from camera
    # 
    AK = ArmIK()
    print(AK.setPitchRangeMoving((12, 0, 20), 0, -900, 900, 1000))
    Board.setBusServoPulse(1, 400, 500)
    Board.setBusServoPulse(2, 500, 500)
    sleep(1)
