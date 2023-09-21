

class Robot:
    _robot_id = 0
    def __init__(self, center_to_wheel=0.1, radius=0.1, x=0.0, y=0.0, theta=0.0)-> None:
        self.L = center_to_wheel
        self.r = radius
        self.x_w = x
        self.y_w = y
        self.t_p = theta
        Robot._robot_id += 1
        self.robot_id = Robot._robot_id

    @property
    def getWheelRadius(self):
        return self.r

    @property
    def getCenterToWheel(self):
        return self.L
    
    @property
    def getrobot_id(self):
        return self._robot_id
