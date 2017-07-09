import numpy as np
import time
# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            Rover.send_pickup = False
            Rover.stopped = False
            
            # If the rover didn't move that much within 5 seconds change the mode in to got_stuck
            if Rover.dist < 0.2 and Rover.total_time > 15:
                Rover.last_time = Rover.current_time
                Rover.mode = 'got_stuck'
            
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'
        # If there is a rock sample in vision
        elif Rover.mode == 'rock_nav':
            # If the rover is moving
            if ((Rover.vel > 0) and (not Rover.stopped)):
                # Stop acclerating
                Rover.throttle = 0
                # Set brake to stored brake value
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # Once we're sure that the rover stopped moving, change the flag of 'stopped' to 'True'
            elif ((Rover.vel == 0) and (not Rover.stopped)):
                Rover.stopped = True
            # Start navigating to the rock sample
            elif Rover.stopped:
                # Start moving toward the rock slowlly
                if ((Rover.vel) < 0.5 and (not Rover.near_sample)):
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                # If the rover is near to the rock sample
                elif (Rover.near_sample):
                    # Pick up the rock sample
                    Rover.send_pickup = True
                    # Change the mode to forward
                    Rover.mode = 'forward'
                    
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
        # If the rover got stuck, perform 4 wheel turn
        elif Rover.mode == 'got_stuck':
             # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0


            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                
                Rover.throttle = 0
                # Release the brake to allow turning
                Rover.brake = 0
                # Perform 4-wheel turning
                angle = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                if(angle >= 0):
                    Rover.steer = 15
                else:
                    Rover.steer = -15

                if((Rover.current_time - Rover.last_time) > 2):
                    print('go for')
                    Rover.dist = 10.0
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'

                
                



        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0

            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0

    return Rover

