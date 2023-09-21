import math
import os

def create_launch_file():
    yaw = math.pi / 2  # 90 degrees in radians

    launch_file_content = f"""
    <launch>
        <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen"
            args="-urdf -model my_robot -param robot_description -robot_namespace my_robot -x 0 -y 0 -z 1 -Y {yaw}"/>
    </launch>
    """

    with open("spawn.launch", "w") as f:
        f.write(launch_file_content)

if __name__ == "__main__":
    create_launch_file()
