
from robot import Robot
from kiwi_kinematics import KiwiKinematics
from trajectory_planning_v2 import TrajectoryPlanning
from kiwi_plot import PlotlyPlot
import numpy as np
# Add path planning

class KiwiRobot(Robot):
    def __init__(self, center_to_wheel, radius, x, y, theta) -> None:
        super().__init__(center_to_wheel, radius, x, y, theta)
        self.kinematics = KiwiKinematics(self)
        self.trajectory_planning = TrajectoryPlanning(self)
        # self.plotly_plot = PlotlyPlot(self)


    def fk_robot_frame(self, v1: float, v2: float, v3: float):
        return self.kinematics.fk_robot_frame(v1, v2, v3)
    
    def fk_world_frame(self, v1: float, v2: float, v3: float):
        return self.kinematics.fk_world_frame(v1, v2, v3)
    
    def movement_anywhere(self, v1: float, v2: float, v3: float):
        return self.kinematics.movement_vel(v1, v2, v3)

    def move_forward(self, v1=-np.sqrt(3)/2, v2=0.0, v3=np.sqrt(3)/2, time=5):
        return self.kinematics.movement_vel(v1, v2, v3, time)
    
    def move_backward(self, v1=-np.sqrt(3)/2, v2=0.0, v3=np.sqrt(3)/2, time=5):
        return self.kinematics.movement_vel(v1, v2, v3, time)
    
    def move_left(self, v1=-0.5, v2=1.0, v3=-0.5, time=5):
        return self.kinematics.movement_vel(v1, v2, v3, time)

    def move_right(self, v1=0.5, v2=-1.0, v3=0.5, time=5):
        return self.kinematics.movement_vel(v1, v2, v3, time)
    
    def rotate_anywhere(self, v1=1.0, v2=1.0, v3=1.0, angle=np.pi/2):
        return self.kinematics.rotate_vel(v1, v2, v3, angle)

    def rotate_left(self, v1=1.0, v2=1.0, v3=1.0, time=5):
        return self.kinematics.movement_vel(v1, v2, v3, time)

    def rotate_right(self, v1=-1.0, v2=-1.0, v3=-1.0, time=5):
        return self.kinematics.movement_vel(v1, v2, v3, time)
    

    ## add all the methods
