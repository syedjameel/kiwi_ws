import numpy as np
import matplotlib.pyplot as plt
import time
from kinematics import Kinematics

class KiwiKinematics(Kinematics):
    def __init__(self, robot) -> None:
        super().__init__()
        self.robot = robot
        self.sqrt3 = np.sqrt(3)
        #self.theta = 180 * (np.pi/180)
        self.theta = 0.0

        self.Vx_w = 0.0
        self.Vy_w = 0.0
        self.w_p = 0.0
        self.Vx_m = 0.0
        self.Vy_m = 0.0

        self.x_m = 0.0
        self.y_m = 0.0

        # to record trajectory
        self.Xw = []
        self.Yw = []
        self.Tp = []
    
    @property
    def getCurrentPoseWorldFrame(self):
        return self.robot.x_w, self.robot.y_w, self.robot.t_p
    
    @property
    def getCurrentPoseRobotFrame(self):
        return self.x_m, self.y_m, self.robot.t_p

    @property
    def getCurrentVelWorldFrame(self):
        return self.Vx_w, self.Vy_w, self.w_p

    @property
    def getCurrentVelRobotFrame(self):
        return self.Vx_m, self.Vy_m, self.w_p

    def fk_robot_frame(self, v1: float, v2: float, v3: float):
        self.Vx_m = (2*v2 - v1 - v3)/3
        self.Vy_m = (self.sqrt3 * (v3 - v1)/3)
        self.w_p = (v1 + v2 + v3)/3

        return [self.Vx_m, self.Vy_m, self.w_p]
    
    def fk_world_frame(self, v1: float, v2: float, v3: float):
        self.Vx_m, self.Vy_m, self.w_p = self.fk_robot_frame(v1, v2, v3)
        self.Vx_w = np.cos(self.theta)*self.Vx_m - np.sin(self.theta)*self.Vy_m
        self.Vy_w = np.sin(self.theta)*self.Vx_m + np.cos(self.theta)*self.Vy_m

        return [self.Vx_w, self.Vy_w]
    
    def ik_robot_frame(self, v1: float, v2: float, v3: float):
        #TODO: add the ik here
        pass

    def ik_world_frame(self, v1: float, v2: float, v3: float):
        #TODO: add the ik here
        pass

    def robot_to_world_frame(self, Vx_m, Vy_m, t_p):
        Vx_w = np.cos(t_p)*Vx_m - np.sin(t_p)*Vy_m
        Vy_w = np.sin(t_p)*Vx_m + np.cos(t_p)*Vy_m
        return [Vx_w, Vy_w]
    
    def world_to_robot_frame(self, Vx_w, Vy_w, t_p):
        Vx_m = np.cos(t_p)*Vx_w + np.sin(t_p)*Vy_w
        Vy_m = -np.sin(t_p)*Vx_w + np.cos(t_p)*Vy_w
        return [Vx_m, Vy_m]

    def movement_vel(self, v1, v2, v3, finish_time = 10):
        v1 = self.robot.r*v1 #convertion
        v2 = self.robot.r*v2
        v3 = self.robot.r*v3
        tim = 0
        print(self.robot.robot_id, " v1 = ", v1, "v2 = ", v2, "v3 = ", v3)
        loop_start_time = time.time()
        while tim != finish_time:
            if time.time() - loop_start_time >= 1.0:
                duration = time.time() - loop_start_time
                self.Vx_m, self.Vy_m, self.w_p = self.fk_robot_frame(v1, v2, v3)
                print(self.robot.robot_id, " Duration = ", duration)
                # print("Vx_m = ", self.Vx_m, "Vy_m = ", self.Vy_m, "w_p = ", self.w_p)
                self.x_m += self.Vx_m*duration
                self.y_m += self.Vy_m*duration
                self.robot.t_p += self.w_p*duration
                # print("x_m = ", self.x_m, "y_m = ", self.y_m, "t_p = ", self.robot.t_p)
                # world frame
                self.Vx_w, self.Vy_w = self.robot_to_world_frame(self.Vx_m, self.Vy_m, self.robot.t_p)
                print(self.robot.robot_id, " Vx_w = ", self.Vx_w, "Vy_w = ", self.Vy_w)
                self.robot.x_w += self.Vx_w*duration
                self.robot.y_w += self.Vy_w*duration
                print(self.robot.robot_id, " x_w = ", self.robot.x_w, "y_w = ", self.robot.y_w, "t_p = ", self.robot.t_p)
                self.Xw.append(self.robot.x_w)
                self.Yw.append(self.robot.y_w)
                self.Tp.append(self.robot.t_p)
                loop_start_time = time.time()
                tim = tim + 1
        return self.Xw, self.Yw, self.Tp
    
    def rotate_vel(self, v1, v2, v3, angle=np.pi/2):
        v1 = self.robot.r*v1 #convertion
        v2 = self.robot.r*v2
        v3 = self.robot.r*v3
        print(self.robot.robot_id, " v1 = ", v1, "v2 = ", v2, "v3 = ", v3)
        loop_start_time = time.time()
        ang = self.robot.t_p + angle
        while self.robot.t_p < ang:
            if time.time() - loop_start_time >= 0.1:
                duration = time.time() - loop_start_time
                self.Vx_m, self.Vy_m, self.w_p = self.fk_robot_frame(v1, v2, v3)
                print(self.robot.robot_id, " Duration = ", duration)
                # print("Vx_m = ", self.Vx_m, "Vy_m = ", self.Vy_m, "w_p = ", self.w_p)
                self.x_m += self.Vx_m*duration
                self.y_m += self.Vy_m*duration
                self.robot.t_p += self.w_p*duration
                # print("x_m = ", self.x_m, "y_m = ", self.y_m, "t_p = ", self.robot.t_p)
                # world frame
                self.Vx_w, self.Vy_w = self.robot_to_world_frame(self.Vx_m, self.Vy_m, self.robot.t_p)
                print(self.robot.robot_id, " Vx_w = ", self.Vx_w, "Vy_w = ", self.Vy_w)
                self.robot.x_w += self.Vx_w*duration
                self.robot.y_w += self.Vy_w*duration
                print(self.robot.robot_id, " x_w = ", self.robot.x_w, "y_w = ", self.robot.y_w, "t_p = ", self.robot.t_p)
                self.Xw.append(self.robot.x_w)
                self.Yw.append(self.robot.y_w)
                self.Tp.append(self.robot.t_p)
                loop_start_time = time.time()
        return self.Xw, self.Yw, self.Tp

if __name__ == "__main__":

    print("hey lets do it")
    r = 0.1
    kiwi = KiwiKinematics(0.1, r)
    # print("[Vx_m, Vy_m, w_p] = ", kiwi.fk_robot_frame(-0.5, 1, -0.5))
    # print("[Vx_w, Vy_w] = ", kiwi.fk_world_frame(-0.5, 1, -0.5))

    print("________________ Slide Right  ________________________")
    # Forward
    v1 = -0.5
    v2 = 1.0
    v3 = -0.5
    kiwi.movement_vel(v1, v2, v3, 10)


    print("________________ Forward ________________________")
    # Forward
    v1 = -np.sqrt(3)/2
    v2 = 0.0
    v3 = np.sqrt(3)/2
    kiwi.movement_vel(v1, v2, v3, 10)

    print("________________ Rotate Left ________________________")
    # Rotate left
    v1 = 1.0
    v2 = 1.0
    v3 = 1.0
    kiwi.movement_vel(v1, v2, v3, 31)

    # print("________________ Rotate Right ________________________")
    # # Rotate right
    # v1 = -1.0
    # v2 = -1.0
    # v3 = -1.0
    # kiwi.movement_vel(v1, v2, v3, 10)

    print("________________ Forward ________________________")
    # Forward
    v1 = -np.sqrt(3)/2
    v2 = 0.0
    v3 = np.sqrt(3)/2
    kiwi.movement_vel(v1, v2, v3, 10)

    print("________________ Slide Right  ________________________")
    # Forward
    v1 = -0.5
    v2 = 1.0
    v3 = -0.5
    kiwi.movement_vel(v1, v2, v3, 10)






    # x_m = 0.0
    # y_m = 0.0
    # t_p = 0.0

    # x_w = 0.0
    # y_w = 0.0
    # v1 = r*-1.0
    # v2 = r*0.0
    # v3 = r*1.0
    # tim = 0
    # print("v1 = ", v1, "v2 = ", v2, "v3 = ", v3)
    # loop_start_time = time.time()
    # while tim != 10:
    #     if time.time() - loop_start_time >= 1.0:
    #         duration = time.time() - loop_start_time
    #         Vx_m, Vy_m, w_p = kiwi.fk_robot_frame(v1, v2, v3)
    #         print("Duration = ", duration)
    #         print("Vx_m = ", Vx_m, "Vy_m = ", Vy_m, "w_p = ", w_p)
    #         x_m += Vx_m*duration
    #         y_m += Vy_m*duration
    #         t_p += w_p*duration
    #         print("x_m = ", x_m, "y_m = ", y_m, "t_p = ", t_p)
    #         # world frame
    #         Vx_w = np.cos(t_p)*Vx_m - np.sin(t_p)*Vy_m
    #         Vy_w = np.sin(t_p)*Vx_m + np.cos(t_p)*Vy_m
    #         print("Vx_w = ", Vx_w, "Vy_w = ", Vy_w)
    #         x_w += Vx_w*duration
    #         y_w += Vy_w*duration
    #         print("x_w = ", x_w, "y_w = ", y_w, "t_p = ", t_p)
    #         loop_start_time = time.time()
    #         tim = tim + 1

