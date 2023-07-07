import numpy as np

integral_term = 0

class PID_controller:
    def __init__(self):
        self.prev_action = 0 #action is in torq


    def get_action(self, state, image_state, random_controller=False):
        #terminal, Boolean
        #timestep, int
        #x, float, [-2.4, 2.4]
        #x_dot, float, [-inf, inf]
        #theta, float, [-pi/2, pi/2], radians
        #theta_dot, float, [-inf, inf]
        #reward, int, 0 or 1
        #image state is a (800, 400, 3) numpy image array; ignore it for assignment 2

        terminal, timestep, x, x_dot, theta, theta_dot, reward = state

        global integral_term

        if random_controller:
            return np.random.uniform(-1, 1)
        else:
            print("Balancing...")
            
            if timestep == 0:
                integral_term = 0
        
            integral_term += theta

            Ku = 5.0
            Tu = 12.0

            #2(A)
            #PID - Proportional + Integral + Derivative
            return (0.6 * Ku) * (theta + ((Tu / 8) * theta_dot) + ((2 / Tu) * integral_term))


            #2(B)
            #P-Control
            #return (0.5 * Ku) * theta

            #PD
            #return (0.8 * Ku) * (theta + ((Tu / 8) * theta_dot))

            #PI
            #return (0.45 * Ku) * (theta + ((1.2 / Tu) * integral_term))

            #DI
            #return ((Tu / 8) * theta_dot) + ((2 / Tu) * integral_term)
            
