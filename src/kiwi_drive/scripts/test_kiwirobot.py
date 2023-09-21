from kiwirobot import KiwiRobot
import threading
from kiwi_plot import PlotlyPlot
import concurrent.futures


kiwi1 = KiwiRobot(center_to_wheel=0.1, radius=0.1, x=0.0, y=0.0, theta=0.0)
print("Robot ID = ", kiwi1.robot_id)

kiwi2 = KiwiRobot(center_to_wheel=0.1, radius=0.1, x=-1.0, y=0.0, theta=0.0)

print("Robot ID = ", kiwi2.robot_id)

kiwi3 = KiwiRobot(center_to_wheel=0.1, radius=0.1, x=1.0, y=0.0, theta=0.0)

print("Robot ID = ", kiwi3.robot_id)

# print(kiwi1.fk_world_frame(1.0, 1.0, 1.0))


# kiwi1.move_forward()

with concurrent.futures.ThreadPoolExecutor() as executor:
    # Submit the functions to the executor
    future1 = executor.submit(kiwi1.move_forward)
    future2 = executor.submit(kiwi2.move_left)
    future3 = executor.submit(kiwi3.move_right)

    # Get the results
    result1 = future1.result()
    result2 = future2.result()
    print("result 1 = ", result1)

    print("result 2 = ", result2)

    print("result 3 = ", result1)


plot_obj = PlotlyPlot()

plot_obj.plot([kiwi1, kiwi2, kiwi3])

# thread1 = threading.Thread(target=kiwi1.move_forward)
# thread2 = threading.Thread(target=kiwi2.rotate_anywhere)
# thread3 = threading.Thread(target=kiwi3.move_forward)

# thread1.start()
# thread2.start()
# thread3.start()

# thread1.join()
# thread2.join()
# thread3.join()