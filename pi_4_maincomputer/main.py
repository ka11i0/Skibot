from PosController import PosController
from hal import AngleController
from Robot import Robot
from TrajGen import Trajectory
from math import pi, radians, acos, degrees
from common import *
from visuals import *
from datahandling import *
import matplotlib.pyplot as plt



def main():
    body_len = 0.77
    limb_len = 0.2
    pole_len = 0.82
    elbow_lim = degrees(pi - acos( (2*20**2-14.5**2) / (2*20**2) ))
    alpha_angle = acos( (2*20**2-11**2) / (2*20**2) )

    # Init Robot model instance
    robot = Robot(l_B=body_len, l_L=limb_len, l_P=pole_len, alpha=alpha_angle)
        

    traj_gen = Trajectory(start=0.0, end=-0.85, robot=robot, contact_speed=0.1, return_speed=0.1, acc=1.0, resting_height=0.1)
    logger.info("Max end pcx = "+str(traj_gen.calc_min_end()))
    traj_gen.generate_trajectory(1)
    # Check if elbow constraint is exceeded
    for i, angles in enumerate(traj_gen.angles):
        if(angles[1]>elbow_lim):
            logger.warning("ELBOW LIMIT EXCEEDED AT INDEX:"+str(i)+"\n"+"ANGLE"+str(angles[1]))
        

    # Adc filter window size, bigger size increases frequency response
    filter_window_size = 1

    # Sync_mode, 
    # 0 = No sync, arms run independently 
    # 1 = Strict, arms will always be on the same path index
    # 2 = Diff, arms will be sync according to the acceptable index difference set by index _diff
    sync_mode = 1
    index_diff = 0

    # Threshold is the minimum value of angle for m0, m1 to increment to the next index
    threshold = 16

    # Run loop
    if args.run:
        angle_ctrl = AngleController(filter_window_size)
        time.sleep(1)
        pos_ctrl = PosController(angle_controller=angle_ctrl, trajectory_generator=traj_gen, sync_mode=sync_mode, threshold=threshold, robot=robot, index_diff=index_diff) 
        pos_ctrl.init(args.map)
        pos_ctrl.run()
    
    # Run test loop
    if args.test:
        angle_ctrl = AngleController(filter_window_size)
        time.sleep(1)
        pos_ctrl = PosController(angle_controller=angle_ctrl, trajectory_generator=traj_gen, sync_mode=sync_mode, threshold=threshold, robot=robot, index_diff=index_diff) 
        pos_ctrl.init(args.map)
        pos_ctrl.testrun()


    # Fast plot of path
    if args.plot:
        x_val = [x[0] for x in traj_gen.xy_traj]
        y_val = [x[1] for x in traj_gen.xy_traj]
        # for i in range(0, len(traj_gen.pcx_data)):
        #     plt.plot((x_val[i+1],traj_gen.pcx_data[i]), (y_val[i+1], 0), color='b', linewidth=0.5)
        plt.scatter(x_val, y_val, c='g', linewidths=0.1)
        plt.show()

    # Pygame visualization of the pole movement in 2d space.
    if args.vis:
        window = Window((600, 600), 100)
        window.run(traj_gen.xy_traj[0:len(traj_gen.pcx_data)], traj_gen.pcx_data)#, traj_gen.traj_angle)
    
    # Save generated path to csv
    if args.save:
        datahandler = DataHandler()
        path = "paths/angles.csv"
        datahandler.save_to_csv(traj_gen.angles, ["t0", "t1"], path)
        path = "paths/xy.csv"
        datahandler.save_to_csv(traj_gen.xy_traj, ["x", "y"], path)


if __name__=="__main__":
    main()

