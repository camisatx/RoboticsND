##################################################################################
# Your goal is to follow the comments and complete the the tasks asked of you.
#
# Good luck designing your proportional integral controller!
#
##################################################################################


class PI_Controller:

    def __init__(self, kp=0.0, ki=0.0, start_time=0):
        """Initialize the class variables.

        :param kp: Optional float of the kp (proportional) value
        :param ki: Optional float of the ki (integral) value
        :param start_time: Optional float of the start time
        """
        
        # The PI controller can be initalized with a specific kp and ki value
        self.kp_ = float(kp)
        self.ki_ = float(ki)

        # Define error_sum_ and set to 0.0
        self.error_sum_ = 0

        # Store relevant data
        self.last_timestamp_ = 0.0
        self.set_point_ = 0.0
        self.start_time_ = start_time

        # Control effort history
        self.u_p = [0]
        self.u_i = [0]

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

        # Calculate the error_sum_ = summation of area of delta time by error
        self.error_sum_ += error * delta_time
        
        # Proportional error
        p = self.kp_ * error
       
        # Integral error. Be sure to access the internal Ki class variable
        i = self.ki_ + error
        
        # Set the control effort, u, which is the sum of all your errors. In
        #   this case it is just the proportional and integral error.
        u = p + i
        
        # Here we are storing the control effort history for post control
        #   observations. 
        self.u_p.append(p)
        self.u_i.append(i)

        return u
