

import numpy as np



# TODO Part 3: Comment the code explaining each part
class kalman_filter:
    
    # DONE Part 3: Initialize the covariances and the states    
    def __init__(self, P,Q,R, x, dt):
        # Initialization of the class kalman_filter

        # Assign class variables on instance instantiation
        self.P = P
        self.Q = Q
        self.R = R
        self.x = x
        self.dt = dt
        
    # DONE Part 3: Replace the matrices with Jacobians where needed        
    def predict(self):
        """
        This function is the prediction step of the extended kalman filter
        """

        # Getting the jacobian of the state matrix
        self.A = self.jacobian_A()

        # Getting the jacobian of the measurment matrix
        # C is set in predict because it is dependent on previous self.x
        self.C = self.jacobian_H()
        
        # Setting the mean prediction using the derived motion model
        # This is step 1 in the EK_filter algorithm
        self.motion_model()
        
        # Determining the covariance prediction using the jacobian state matrix, covariance matrix, and Q
        # This is step 2 in the EK_filter algorithm
        self.P= np.dot( np.dot(self.A, self.P), self.A.T) + self.Q

    # DONE Part 3: Replace the matrices with Jacobians where needed
    def update(self, z):
        """
        This function performs the update step of the extended kalman filter
        """

        # This is step 3 in the EK_filter algorithm
        S=np.dot(np.dot(self.C, self.P), self.C.T) + self.R
        kalman_gain=np.dot(np.dot(self.P, self.C.T), np.linalg.inv(S))
        
        # This is step 4 in the EK_filter algorithm
        # The suprise_error calculates the difference between the measured values
        # from the ODOM/IMU and modeled values based on our predicted state
        surprise_error= z - self.measurement_model()
        # x is our mean for our belief, the prediction (self.x)
        # is updated and overwrites self.x to form the belief
        self.x=self.x + np.dot(kalman_gain, surprise_error)

        # P is our covariance for our belief
        # This is step 5 in the EK_filter algorithm
        self.P=np.dot( (np.eye(self.A.shape[0]) - np.dot(kalman_gain, self.C)) , self.P)
        
    
    # DONE Part 3: Implement here the measurement model
    def measurement_model(self):
        """
        This function returns the predicted [v, w, ax, ay] of the motion model
        to be compared with the measured values
        """
        x, y, th, w, v, vdot = self.x
        return np.array([
            v,# v
            w,# w
            vdot, # ax
            v*w, # ay (kf_ay = kf_vx * kf_w for this lab)
        ])
        
    # DONE Part 3: Impelment the motion model (state-transition matrice)
    def motion_model(self):
        """
        This function calculates the predicted [x, y, th, w, v, vdot] of the motion model
        to then be updated using the kalman gain and suprise error to form the belief.
        """
        x, y, th, w, v, vdot = self.x
        dt = self.dt
        
        """
        Calculate the kinematics equations for all the state variables
            - Note: Acceleration components for x, y are disregarded becuase
            dt**2 values are extremely small
            - x_k+1 = x + v * np.cos(th) * dt
            - y_k+1 = y + v * np.sin(th) * dt
            - th_k+1 = th + w * dt
            - w_k+1 = w
            - v_k+1 = v + vdot*dt
            - vdot_k+1 = vdot
        """

        # Doesn't return variable, updates state of class varible
        self.x = np.array([
            x + v * np.cos(th) * dt,
            y + v * np.sin(th) * dt,
            th + w * dt,
            w,
            v + vdot*dt,
            vdot,
        ])
        


    
    def jacobian_A(self):
        """
        This function returns the jacobian of the state matrix.
        """
        x, y, th, w, v, vdot = self.x
        dt = self.dt
        
        """
        Steps:
        1. Calculate the kinematics equations for all the state variables
            - Note: Acceleration components for x, y are disregarded becuase
            dt**2 values are extremely small (and based on given values in jacobian_A)
            - x_k+1 = x + v * np.cos(th) * dt
            - y_k+1 = y + v * np.sin(th) * dt
            - th_k+1 = th + w * dt
            - w_k+1 = w
            - v_k+1 = v + vdot*dt
            - vdot_k+1 = vdot
        2. Calculate all the partial derivatives of each equation WRT to each state variable
        3. Put it into the matrix
        """

        # Theta for row 1 (partial derivative of x_k+1 WRT th)
        # Commented the acceleration component due to reason mentioned in steps above
        th_1 = -v * np.sin(th) * dt # + (-0.5 * vdot * np.sin(th) * dt**2)

        # Theta for row 1 (partial derivative of x_k+1 WRT v)
        v_1 = np.cos(th) * dt

        # Commented the acceleration component due to reason mentioned in steps above
        #vdot_1 = 0.5 * np.cos(th) * dt**2


        # Theta for row 2 (partial derivative of y_k+1 WRT th)
        # Commented the acceleration component due to reason mentioned in steps above
        th_2 = v * np.cos(th) * dt # + (0.5 * vdot * np.cos(th) * dt**2)

        # Theta for row 2 (partial derivative of y_k+1 WRT v)
        v_2 = np.sin(th) * dt

        # Commented the acceleration component due to reason mentioned in steps above
        #vdot_2 = 0.5 * np.sin(th) * dt**2

        # Matrix of partial derivatives (Jacobian Matrix)
        return np.array([
            #x, y,               th, w,             v, vdot
            [1, 0,              th_1, 0,          v_1,  0],
            [0, 1,              th_2, 0,          v_2,  0],
            [0, 0,                1, dt,           0,  0],
            [0, 0,                0, 1,            0,  0],
            [0, 0,                0, 0,            1,  dt],
            [0, 0,                0, 0,            0,  1 ]
        ])
    
    
    # DONE Part 3: Implement here the jacobian of the H matrix (measurements)    
    def jacobian_H(self):
        """
        This function returns the jacobian of the measurment matrix.
        """
        x, y, th, w, v, vdot=self.x
        return np.array([
            #x, y,th, w, v,vdot
            [0,0,0  , 0, 1, 0], # v
            [0,0,0  , 1, 0, 0], # w
            [0,0,0  , 0, 0, 1], # ax
            [0,0,0  , v, w, 0], # ay (kf_ay = kf_vx * kf_w for this lab)
        ])
        
    # DONE Part 3: return the states here    
    def get_states(self):
        #Return the class states variable containing [x, y,th, w, v,vdot]
        return self.x
