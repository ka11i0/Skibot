from logging import Logger
from common import *
from datahandling.DataHandler import DataHandler
from math import radians, sqrt

class PosController:
    def __init__(self, angle_controller, trajectory_generator, sync_mode, threshold, robot, index_diff):
        """
        PosController is a modified position controller to achieve a desired arm movement pattern, using TrajGen, feedback AdcReader and AngleController

        ...

        Attributes
        ----------
        angle_controller : AngleController()
            -
        traj_gen : TrajGen()
            trajectory/path planner
        sync_mode : int
            mode of arm synchronization
        angle_tresh : int
            error threshold for reference iteration
        indexs : [int, int]
            array of left and right current reference index
        index_d : int
            acceptable index difference between arms
        robot : Robot()
            robot model
        data : (float,float)[]
            angle data collection array for testing

        Methods
        -------
        run()
            Runs the motion control loop indefinitely.
        testrun()
            Runs a test with a single poling motion and collects data.
        init(create_mapping)
            Init function which creates a adc mapping if create_mapping=1 or mapping does not exist
        check_angles()
            Checks motor angles and returns a tuple of (0/1, 0/1) if right or left error are within the threshold
        inc_index(i)
            Increments indexs based on i tuple.
        update_angles()
            Uses check_angles together with inc_index to update the reference angles for the arms.

        """
        self.angle_controller = angle_controller
        self.traj_gen = trajectory_generator
        self.sync_mode = sync_mode
        self.angle_thres = threshold
        self.indexs = [0,0]
        self.index_d = index_diff
        self.robot = robot
        self.data = []

    def run(self):
        """Runs the motion control loop indefinitely.

        Parameters
        ----------
        none

        Returns
        -------
        none
        """
        self.angle_controller.set_angles(self.traj_gen.angles[self.indexs[0]]+self.traj_gen.angles[self.indexs[1]])
        time.sleep(2)
        while(True):
            self.update_angles()
            time.sleep(0.02)
        return 0
    
    def testrun(self):
        """Runs a test with a single poling motion and collects data.

        Parameters
        ----------
        none

        Returns
        -------
        none
        """
        # Control loop
        self.angle_controller.set_angles(self.traj_gen.angles[self.indexs[0]]+self.traj_gen.angles[self.indexs[1]])
        last_i = len(self.traj_gen.angles)-1
        t = []
        time.sleep(5)
        t.append(time.monotonic())
        self.data.append(self.angle_controller.get_angles()+self.indexs)
        while(not self.indexs[0]==last_i or not self.indexs[1]==last_i):
            self.update_angles()
            t.append(time.monotonic())
            self.data.append(self.angle_controller.get_angles()+self.indexs)
            time.sleep(0.02)
        datahandler = DataHandler()
        path = "testdata/angles.csv"
        datahandler.save_to_csv(self.data, ["r0", "r1", "l0", "l1", "i1", "i2"], path)

        xy_data = []
        for angles in self.data:
            right_xy = self.robot.calc_hand_pos((radians(angles[0]),radians(angles[1])))
            left_xy = self.robot.calc_hand_pos((radians(angles[2]),radians(angles[3])))
            xy_data.append(right_xy+left_xy)
        path = "testdata/xy.csv"
        datahandler.save_to_csv(xy_data, ["rx", "ry", "lx", "ly"], path)

        path = "testdata/r_angles.csv"
        datahandler.save_to_csv(self.traj_gen.angles, ["t0", "t1"], path)

        path = "testdata/r_xy.csv"
        datahandler.save_to_csv(self.traj_gen.xy_traj, ["x", "y"], path)

        errors = []
        for i,xy in enumerate(xy_data):
            er = sqrt( (self.traj_gen.xy_traj[self.data[i][4]][0]-xy[0])**2+(self.traj_gen.xy_traj[self.data[i][4]][1]-xy[1])**2 )
            el = sqrt( (self.traj_gen.xy_traj[self.data[i][5]][0]-xy[2])**2+(self.traj_gen.xy_traj[self.data[i][5]][1]-xy[3])**2 )
            errors.append((er,el))
        path = "testdata/error.csv"
        datahandler.save_to_csv(errors, ["er", "el"], path)

        path = "testdata/t.csv"
        t_epi = t[0]
        for i in range(0,len(t)):
            t[i] = t[i] - t_epi
        datahandler.save_to_csv(zip(t), "t", path)

        logger.info("Test finished")
        return 0

    
    def init(self, create_map):
        """Init function which creates a adc mapping if create_mapping=1 or mapping does not exist

        Parameters
        ----------
        create_map : int

        Returns
        -------
        none
        """
        if create_map or not exists('hal/adc_mapping.txt'):
            self.angle_controller.create_mapping()
        else:
            self.angle_controller.import_mapping()


    def check_angles(self):
        """Checks motor angles and returns a tuple of (0/1, 0/1) if right or left error are within the threshold

        Parameters
        ----------
        none

        Returns
        -------
        (r_arm, l_arm) : (int, int)
            X_arm = 1, if angle error is within the threshold 
        """
        angles = self.angle_controller.get_angles()
        r_arm = 0
        l_arm = 0
        angle_errors = (abs(angles[0]-self.traj_gen.angles[self.indexs[0]][0]), 
                        abs(angles[1]-self.traj_gen.angles[self.indexs[0]][1]), 
                        abs(angles[2]-self.traj_gen.angles[self.indexs[1]][0]), 
                        abs(angles[3]-self.traj_gen.angles[self.indexs[1]][1]))

        logger.debug("r_angles:"+str(self.traj_gen.angles[self.indexs[0]]+self.traj_gen.angles[self.indexs[1]]) )
        logger.debug("errors: "+str(angle_errors))
        if(angle_errors[0]<self.angle_thres and angle_errors[1]<self.angle_thres):
            r_arm = 1
        if(angle_errors[2]<self.angle_thres and angle_errors[3]<self.angle_thres):
            l_arm = 1
        return (r_arm, l_arm)
    
    
    def inc_index(self,i):
        """Increments indexs based on i tuple.

        Parameters
        ----------
        i : (int, int)
            Which self.indexs to increment, (right, left)

        Returns
        -------
        (r_arm, l_arm) : (int, int)
            X_arm = 1, if angle error is within the threshold 
        """
        for upd_index in i:
            self.indexs[upd_index] += 1 
            if(args.run):
                self.indexs[upd_index] %= len(self.traj_gen.angles)
    
    def update_angles(self):
        """Increments indexs based on i tuple.

        Parameters
        ----------
        i : (int, int)
            Which self.indexs to increment, (right, left)

        Returns
        -------
        (r_arm, l_arm) : (int, int)
            X_arm = 1, if angle error is within the threshold 
        """
        if(self.sync_mode==0):
            self.inc_index(self.check_angles())
        else:
            thres_check = self.check_angles()
            if(self.sync_mode==1):
                if sum(thres_check)==2:
                    self.inc_index((0,1))
            else:
                if sum(thres_check)>0:
                    if(abs(self.indexs[0]-self.indexs[1])<self.index_d):
                        self.inc_index(thres_check)
                    else:
                        return 0
        self.angle_controller.set_angles(self.traj_gen.angles[self.indexs[0]]+self.traj_gen.angles[self.indexs[1]])
        return 0