
import sys
sys.path.append('/home/pi/ArmPi/')

from ArmState import ArmState
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board

from scipy.spatial.transform import Rotation as R

from time import sleep, time
class Motion(ArmState):
    def __init__(self):
        self.ik = ArmIK()

        self.state = 'APPROACH_CAN'

        # My aruco tag is located about 5cm in front of the arm
        self.base_offset = np.array([-11, 1, 0])
        self.warmup_ticks = 0

        # This is the rotation from the pose of the aruco marker on my arm to the
        # world frame of the ArmPi. Pitch (falling towards camera axis) is flipped, and
        # the marker is rotated 90 degrees counterclockwise
        self.roboarm_rotation = R.from_euler('xyz', np.array([np.pi, 0, -np.pi/2])).as_matrix()
    
    def open_gripper(self):
        Board.setBusServoPulse(1, -100, 500)

    def close_gripper(self):
        Board.setBusServoPulse(1, 500, 500)

    def tilt_gripper(self):
        Board.setBusServoPulse(2, 1000, 1000)

    def untilt_gripper(self):
        Board.setBusServoPulse(2, 500, 1000)

    def move_to(self, x, y, z):
        if not self.ik.setPitchRangeMoving((x,y,z), 0, -90, 90, 1000):
            print("Can't reach!")
            return False
        return True

    def step(self, markers):
        # Not sure why this is needed, but sometimes rossros will insist that
        # it has received an int and not a dict
        if type(markers) is not dict:
            return

        if self.state == 'APPROACH_CAN':
            self.open_gripper()
            #self.state = 'MOVE_CAN'

        print(self.state)
        if 'can' in markers and 'base' in markers and self.state in ['APPROACH_CAN', 'GRAB_CAN', 'LIFT_CAN']:
            base_R, base_T = markers['base']
            can_R, can_T = markers['can']

            # Center of can is about 3cm behind marker for me
            can_T = np.array(can_T + can_R.T @ np.array([0, 0, -0.03])).flatten()

            # Get difference in translation between base and can, convert to
            # cm
            can_T = np.array((base_R.T @ (base_T - can_T))).flatten() * 100

            can_T = self.roboarm_rotation.T @ np.array([can_T[0], can_T[2], 0]) - self.base_offset

            unit_vec = can_T / np.sqrt((can_T**2).sum())

            if self.state == 'APPROACH_CAN':
                target = can_T[:2] - 5 * unit_vec[:2]
                if self.move_to(*target, 10):
                    self.state = 'GRAB_CAN'
                    sleep(1)
            elif self.state == 'GRAB_CAN':
                target = can_T[:2] - 1. * unit_vec[:2]
                if self.move_to(*target, 10):
                    self.state = 'LIFT_CAN'
            elif self.state == 'LIFT_CAN':
                self.close_gripper()
                sleep(1)
                target = can_T[:2] - 7 * unit_vec[:2]
                if self.move_to(*target, 26):
                    self.state = 'MOVE_CAN'
                    sleep(1)

            print(f"target at {target[0]:5.2f} {target[1]:5.2f}")

        elif 'cup' in markers and 'base' in markers and self.state in ['MOVE_CAN', 'POUR_CAN', 'DONE']:
            base_R, base_T = markers['base']
            # Get can rotation and translation
            cup_R, cup_T = markers['cup']

            # Center of cup is about 5cm behind marker for me
            cup_T = np.array(cup_T + cup_R.T @ np.array([0, 0, -0.02])).flatten()

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
                if self.move_to(*target[:2], 22):
                    # Mark the time when we started pouring so we know when to stop
                    self.state = 'POUR_CAN'
                    self.pour_start = time()
                return

            if self.state == 'POUR_CAN':
                target = cup_T + orthogonal_unit_vector * 6#cm
                if self.move_to(*target[:2], 16):
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

if __name__ == '__main__':

    # x is sideways, - is away from camera
    # 
    AK = ArmIK()
    print(AK.setPitchRangeMoving((12, 0, 20), 0, -900, 900, 1000))
    Board.setBusServoPulse(1, 100, 500)
    Board.setBusServoPulse(2, 500, 500)
    sleep(1)
