##################################################################################
# Your goal is to follow the comments and complete the the tasks asked of you.
#
# Good luck designing your proportional integral and derivative controller!
##################################################################################

class PID_Controller:

    def __init__(self, kp=0.0, ki=0.0, kd=0.0, max_windup=20, start_time=0,
                 alpha=1.0, u_bounds=[float('-inf'), float('inf')]):
        
        # The PID controller is initalized with specific kp, ki, and kd values
        self.kp_ = float(kp)
        self.ki_ = float(ki)
        self.kd_ = float(kd)
        
        # Set max wind up
        self.max_windup_ = float(max_windup)
        
        # Set alpha for derivative filter smoothing factor
        self.alpha = float(alpha) 
        
        # Setting control effort saturation limits
        self.umin = u_bounds[0]
        self.umax = u_bounds[1]

        # Store relevant data
        self.last_timestamp_ = 0.0
        self.set_point_ = 0.0
        self.start_time_ = start_time
        self.error_sum_ = 0.0
        self.last_error_ = 0.0

        # Control effort history
        self.u_p = [0]
        self.u_i = [0]
        self.u_d = [0]

    def reset(self):
        """Add a reset function to clear the class variables"""
        self.set_point_ = 0.0
        self.kp_ = 0.0
        self.ki_ = 0.0
        self.kd_ = 0.0
        self.error_sum_ = 0.0
        self.last_timestamp_ = 0.0
        self.last_error_ = 0
        self.last_last_error_ = 0
        self.last_windup_ = 0.0

    def setTarget(self, target):
        """Set the altitude target

        :param target: Float of the target altitude
        """
        self.set_point_ = float(target)

    def setKP(self, kp):
        """Set the kp (proportional) value.

        :param kp: Float of the kp value
        """
        self.kp_ = float(kp)

    def setKI(self, ki):
        """Set the ki (interval) value.

        :param ki:: Float of the ki value
        """
        self.ki_ = float(ki)

    def setKD(self, kd):
        """Set the kd (derivative) value.

        :param kd:: Float of the kd value
        """
        self.kd_ = float(kd)

    def setMaxWindup(self, max_windup):
        """Set the max windup value.

        :param max_windup: Integer of the max windup value
        """
        self.max_windup_ = int(max_windup)

    def update(self, measured_value, timestamp):
        """Calculate delta_time using the _last_timestamp and the provided
        timestamp argument.

        :param measured_value: Float of the measured value
        :param timestamp: Timestamp of the current time
        :return: Float of the control effort
        """

        delta_time = timestamp - self.last_timestamp_
        if delta_time == 0:
            # Delta time is zero
            return 0
        
        # Calculate the error 
        error = self.set_point_ - measured_value
        
        # Set the last_timestamp_ 
        self.last_timestamp_ = timestamp

        # Sum the errors
        self.error_sum_ += error * delta_time
        
        # Update the past error
        self.last_error_ = error
        
        # Find delta_error
        delta_error = error - self.last_error_
        
        # Update the past error
        self.last_error_ = error
        
        # Address max windup
        if self.error_sum_ > self.max_windup_:
            self.error_sum_ = self.max_windup_
        elif self.error_sum_ < -self.max_windup_:
            self.error_sum_ = -self.max_windup_
        
        # Proportional error
        p = self.kp_ * error
       
        # Integral error
        i = self.ki_ * self.error_sum_
       
        # Recalculate the derivative error here incorporating derivative smoothing!
        d = self.kd_ * (self.alpha * delta_error / delta_time + (1-self.alpha))
        
        # Set the control effort
        u = p + i + d
        
        # Enforce actuator saturation limits
        if u > self.umax:
            u = self.umax
        elif u < self.umin:
            u = self.umin
    
        # Here we are storing the control effort history for post control
        # observations. 
        self.u_p.append(p)
        self.u_i.append(i)
        self.u_d.append(d)

        return u
