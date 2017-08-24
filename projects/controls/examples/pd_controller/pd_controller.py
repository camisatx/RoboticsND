##################################################################################
# Your goal is to follow the comments and complete the the tasks asked of you.
#
# Good luck designing your proportional derivative controller!
##################################################################################


class PD_Controller:

    def __init__(self, kp = 0.0, kd = 0.0, start_time = 0):
        """Initialize the class variables.

        :param kp: Optional float of the kp (proportional) value
        :param kd: Optional float of the kd (derivative) value
        :param start_time: Optional float of the start time
        """   

        # The PD controller can be initalized with a specific kp and kd value
        self.kp_ = float(kp)
        self.kd_ = float(kd)
        
        # Define last_error_ and set to 0.0
        self.last_error_ = 0.0

        # Store relevant data
        self.last_timestamp_ = 0.0
        self.set_point_ = 0.0
        self.start_time_ = start_time
        self.error_sum_ = 0.0

        # Control effort history
        self.u_p = [0]
        self.u_d = [0]

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
        
    def setKD(self, kd):
        """Set the kd (derivative) value.

        :param kd:: Float of the kd value
        """
        self.kd_ = float(kd)

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

        # Find error_sum_
        self.error_sum_ += error * delta_time
        
        # Calculate the delta_error
        delta_error = (error - self.last_error_) / delta_time
        
        # Update the past error with the current error
        self.last_error_ = error

        # Proportional error
        p = self.kp_ * error
       
        # Calculate the derivative error
        d = self.kd_ * delta_error
        
        # Set the control effort (u), which is the sum of all your errors. In
        #   this case it is justthe proportional and derivative error.
        u = p + d
        
        # Here we are storing the control effort history for post control
        #   observations. 
        self.u_p.append(p)
        self.u_d.append(d)

        return u
