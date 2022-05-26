import logging
import matplotlib.pyplot as plt
import pychrono as chrono
import os
import common
from copy import copy, deepcopy
from window import *
from robot import *


def main():
    ## Simulation environment setup

    # Body parameters
    body_height = 0.8
    body_width = 0.2
    body_density = 1562.5 #kg/m3 based on 50kg body
    #print(chrono.CH_C_PI_2)

    # Limb params
    limb_length = 0.2
    limb_depth = 0.05
    limb_width = 0.03
    limb_mass = 0.1
    limb_density = limb_mass/(limb_depth*limb_width*limb_length)

    pole_len = 0.8
    pole_rad = 0.01
    pole_density = 100
    pole_contact_rad = 0.1

    # Motor params
    motor_speed = chrono.CH_C_PI / 0.78
    motor_stall_torque = 2.5

    # Left and right body shoulder motor coordinates
    body_r = chrono.ChVectorD(0,body_height,body_width/2)
    body_l = chrono.ChVectorD(0,body_height,-(body_width/2))


    ## Simulation system
    mysystem = chrono.ChSystemNSC()
    #mysystem.Set_G_acc( chrono.ChVectorD(0,0,-9.82) )

    # Create a contact material, shared by all collision shapes
    material = chrono.ChMaterialSurfaceNSC()
    material.SetFriction(1)
    body_mat = chrono.ChMaterialSurfaceNSC()
    body_mat.SetFriction(0)
    pole_mat = chrono.ChMaterialSurfaceNSC()
    pole_mat.SetFriction(1)

    chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.1)
    chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.001)


    ## Ground 
    ground = chrono.ChBodyEasyBox(80, 2, 80, 1000, False, True, material)
    ground.SetPos( chrono.ChVectorD(0, -2, 0) )
    ground.SetBodyFixed(True)
    mysystem.Add(ground)

    ground_marker = chrono.ChMarker()
    ground.AddMarker(ground_marker)

    ## Body
    body = chrono.ChBodyEasyBox(body_width, body_height, body_width, body_density, False, True, body_mat)
    #body.SetBodyFixed(True)
    body.SetPos( chrono.ChVectorD(0, body_height/2, 0) )
    body.GetCollisionModel().SetFamily(1)
    mysystem.Add(body)

    body_marker = chrono.ChMarker()
    body.AddMarker(body_marker)


    # Fix rotations and block translation along Z
    fix_rot = chrono.ChLinkLockAlign()
    fix_rot.Initialize(body_marker, ground_marker)
    mysystem.Add(fix_rot)

    fix_xy = chrono.ChLinkLockPointPlane()
    fix_xy.Initialize(body_marker, ground_marker)
    mysystem.Add(fix_xy)


    ## Limbs

    # Right upper
    limb_r1 = chrono.ChBodyEasyBox(limb_width, limb_length, limb_depth, limb_density)
    limb_r1.SetPos( body_r+chrono.ChVectorD(0,-limb_length/2,limb_depth) ) # Place center of gravity at correct distance
    mysystem.Add(limb_r1)

    # Right lower
    limb_r2 = chrono.ChBodyEasyBox(limb_width, limb_length, limb_depth, limb_density)
    limb_r2.SetPos( body_r+chrono.ChVectorD(0,-(limb_length+limb_length/2),limb_depth) ) 
    mysystem.Add(limb_r2)

    # Left upper
    limb_l1 = chrono.ChBodyEasyBox(limb_width, limb_length, limb_depth, limb_density)
    limb_l1.SetPos( body_l+chrono.ChVectorD(0,-limb_length/2,-limb_depth) ) 
    mysystem.Add(limb_l1)

    # Left lower
    limb_l2 = chrono.ChBodyEasyBox(limb_width, limb_length, limb_depth, limb_density)
    limb_l2.SetPos( body_l+chrono.ChVectorD(0,-(limb_length+limb_length/2),-limb_depth) ) 
    mysystem.Add(limb_l2)


    ## Poles

    # Right
    pole_r = chrono.ChBodyEasyCylinder(pole_rad, pole_len-pole_contact_rad, pole_density)
    pole_r.SetPos( body_r+chrono.ChVectorD(0,-(2*limb_length+pole_len/2),limb_depth) )
    mysystem.Add(pole_r)

    pole_r_contact = chrono.ChBodyEasySphere(pole_contact_rad, pole_density, False, True, pole_mat)
    pole_r_contact.SetPos( body_r+chrono.ChVectorD(0,-(2*limb_length+pole_len-pole_contact_rad),limb_depth) ) 
    
    # Disable collision with body
    pole_r_contact.GetCollisionModel().SetFamily(2)
    pole_r_contact.GetCollisionModel().SetFamilyMaskNoCollisionWithFamily(1)
    mysystem.Add(pole_r_contact)

    # Attach contact to pole
    contact_r_link = chrono.ChLinkMateFix()
    contact_r_link.Initialize(pole_r, pole_r_contact)
    mysystem.Add(contact_r_link)

    
    # Left
    pole_l = chrono.ChBodyEasyCylinder(pole_rad, pole_len-pole_contact_rad, pole_density)
    pole_l.SetPos( body_l+chrono.ChVectorD(0,-(2*limb_length+pole_len/2),-limb_depth) )
    mysystem.Add(pole_l)

    pole_l_contact = chrono.ChBodyEasySphere(0.1, pole_density, False, False, pole_mat)
    pole_l_contact.SetPos( body_l+chrono.ChVectorD(0,-(2*limb_length+pole_len-pole_contact_rad),-limb_depth) ) 

    # Disable collision with body
    pole_l_contact.GetCollisionModel().SetFamily(2)
    pole_l_contact.GetCollisionModel().SetFamilyMaskNoCollisionWithFamily(1)
    mysystem.Add(pole_l_contact)

    # Attach contact to pole
    contact_l_link = chrono.ChLinkMateFix()
    contact_l_link.Initialize(pole_l, pole_l_contact)
    mysystem.Add(contact_l_link)
    

    ## Motor joints
    m1_limits = (chrono.CH_C_PI/2, -chrono.CH_C_PI/2)
    m2_limits = (chrono.CH_C_PI/2, 0)
    spring_limits = (0, -chrono.CH_C_PI+chrono.CH_C_PI/8)

    ## Right
    # Limb to body
    m_r1 = chrono.ChLinkLockRevolute()
    m_r1.Initialize(body,   # the first connected body
                        limb_r1,   # the second connected body
                        chrono.ChCoordsysD(body_r)) # where to create the motor in abs.space
    mysystem.Add(m_r1)
    setConstraints(m_r1, m1_limits)
    
    #print(m_r1_limit.IsRotation())

    # Upper and lower limb
    m_r2 = chrono.ChLinkLockRevolute()
    m_r2.Initialize(limb_r1,   
                        limb_r2,   
                        chrono.ChCoordsysD(body_r+chrono.ChVectorD(0,-limb_length,limb_depth))) 
    mysystem.Add(m_r2)
    setConstraints(m_r2, m2_limits)

    # Lower limb and pole
    spring_r = chrono.ChLinkLockRevolute()
    spring_r.Initialize(limb_r2,   
                        pole_r,   
                        chrono.ChCoordsysD(body_r+chrono.ChVectorD(0,-2*limb_length,limb_depth))) 
    mysystem.Add(spring_r)
    setConstraints(spring_r, spring_limits)

    
    ## Left
    # Limb to body
    m_l1 = chrono.ChLinkLockRevolute()
    m_l1.Initialize(body,   
                        limb_l1,   
                        chrono.ChCoordsysD(body_l)) 
    mysystem.Add(m_l1)
    setConstraints(m_l1, m1_limits)

    # Upper and lower limb
    m_l2 = chrono.ChLinkLockRevolute()
    m_l2.Initialize(limb_l1,   
                        limb_l2,   
                        chrono.ChCoordsysD(body_l)) 
    mysystem.Add(m_l2)
    setConstraints(m_l2, m2_limits)

    # Pole and lower limb
    spring_l = chrono.ChLinkLockRevolute()
    spring_l.Initialize(limb_l2,   
                        pole_l,
                        chrono.ChCoordsysD(body_l+chrono.ChVectorD(0,-2*limb_length,-limb_depth)))
    mysystem.Add(spring_l)
    setConstraints(spring_l, spring_limits)

    

    # Create a ChLinePath geometry, for the hand path, and insert arc/lines sub-paths:
    mpath = chrono.ChLinePath()
    ma1 = chrono.ChLineArc(
                chrono.ChCoordsysD(pole_r_contact.GetCoord()),   # arc plane alignment (default: xy plane) 
                0.2, # radius 
                0, # start arc ngle (counterclockwise, from local x)
                chrono.CH_C_PI, # end arc angle 
                True)
    mpath.AddSubLine(ma1,2.5)
    ma2 = chrono.ChLineSegment(ma1.GetEndB(), ma1.GetEndA())
    mpath.AddSubLine(ma2,0.5)
    mpath.Set_closed(True)

    # Create a ChLineShape, a visualization asset for lines.
    mpathasset = chrono.ChLineShape()
    mpathasset.SetLineGeometry(mpath)
    ground.AddAsset(mpathasset)

    # This is the constraint that uses the trajectory
    mtrajectory = chrono.ChLinkTrajectory()
    # Define which parts are connected (the trajectory is considered in the 2nd body).
    mtrajectory.Initialize(pole_r_contact, # body1 that follows the trajectory
            body,                 # body2 that 'owns' the trajectory
            chrono.VNULL,           # point on body1 that will follow the trajectory, in body1 coords
            mpath                   # the trajectory (reuse the one already added to body2 as asset)
            )
    mysystem.Add(mtrajectory)


    # Simulation params
    dt = 0.005
    sim_dur = 10

    # Simulation loop
    logger.info("simulation started")
    # Initialize these lists to store values to plot.
    array_time = []
    array_body = []
    array_limb1 = []
    array_limb2 = []
    array_pole = []
    array_pole_contact = []
    array_motor1 = []
    array_motor2 = []
    array_spring = []
    arrays = [array_body, array_limb1, array_limb2, array_pole, array_pole_contact, array_motor1, array_motor2, array_spring]
    array_robot_params = []

    # Run the interactive simulation loop
    mysystem.SetChTime(0)
    while(mysystem.GetChTime() < sim_dur):
        
        ## Collect simulation data
        # if(mysystem.GetChTime()>3.4):
        #     m_r1.Lock(True)
        #     mtrajectory.SetDisabled(True)
        

        # for plotting, append instantaneous values:
        array_time.append(mysystem.GetChTime())
        array_body.append(body.GetPos().x)
        array_limb1.append(limb_r1.GetPos().x)
        array_limb2.append(limb_r2.GetPos().x)
        array_pole.append(pole_r.GetPos().x)
        array_pole_contact.append(pole_r_contact.GetPos().x)
        array_motor1.append(m_r1.GetRelAngle())
        array_motor2.append(m_r2.GetRelAngle())
        array_spring.append(spring_r.GetRelAngle())


        # for python visuals
        l1 = pygame.Vector2(sin(m_r1.GetRelAngle()), cos(m_r1.GetRelAngle()))
        l2 = pygame.Vector2(sin(m_r2.GetRelAngle()), cos(m_r2.GetRelAngle()))
        pole = pygame.Vector2(sin(spring_r.GetRelAngle()), cos(spring_r.GetRelAngle()))
        array_robot_params.append([l1, l2, pole, body.GetPos().y])

        ## Propagate simulation
        mysystem.DoStepDynamics(dt)
    
    logger.info("simulation finished")

    
    
    # Pygame visuals
    ## Window params
    if(args.vis):
        window = Window((600, 600), 2)
        window.run()

        clock = pygame.time.Clock()

        # Create frames
        frames = []

        for vects in array_robot_params:
            window.drawArm(vects)
            frames.append(window.display.copy())
            
        # To save frames to files:
        # for i in range(0,360,1):
        #     pygame.image.save(frames[i], 'output/frame_'+str(i)+'.png')

        # Traverse frames
        i = 0
        logger.info("pygame visuals loop started")
        while True:
            try:
                event = pygame.event.poll()
                if event.type == pygame.QUIT:
                    break
                window.display.blit(frames[i], (0,0))
                pygame.display.update()  
                pygame.time.delay(int(dt*1000))
                i = (i + 1) % (len(frames))
            except Exception as e:
                print(e)
                pygame.quit()
                break
        pygame.quit()
        logger.info("pygame.quit()")


    if(args.plot):
        logger.info("plt.show()")
        fig, axes = plt.subplots(len(arrays))
        for i, array in enumerate(arrays):
            axes[i].plot(array_time, array)
            axes[i].grid()

        plt.show()
        plt.close('all')
        logger.info("plt.close()")


if __name__=="__main__":
    main()