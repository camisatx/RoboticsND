##################################################################################
# Your goal is to follow the comments and complete the the tasks asked of you.
# Take this time to try and understand the workings of the empty class structure.
# The following quizzes will assume a understanding of a working class strucutre.
#
# Good luck designing your proportional controller!
#
##################################################################################

class P_Controller:

    def __init__(self, kp=0.0, start_time=0):
        """Initialize the class variables.

        :param kp: Optional float of the kp (proportional) value
        :param start_time: Optional float of the start time
        """

        # The P controller can be initalized with a specific kp value
        self._kp = float(kp)

        # Create internal class variable to store the start time
        self._start_time = start_time

        # Create internal class variables to store current set point
        self._set_point = 0.0

        # Store last timestamp
        self._last_timestamp = 0.0

        # Create a variable to store the control effort
        self.u = 0

        # Control effort history
        self.u_p = [0]

    def setTarget(self, target):
        """Set the altitude target

        :param target: Float of the target altitude
        """
        self._set_point = float(target)

    def setKP(self, kp):
        """Set the kp (proportional) value.

        :param kp: Float of the kp value
        """
        self._kp = float(kp)

    def update(self, measured_value, timestamp):
        """Calculate delta_time using the _last_timestamp and the provided
        timestamp argument.

        :param measured_value: Float of the measured value
        :param timestamp: Timestamp of the current time
        :return: Float of the control effort
        """

        # Calculate delta time using the last timestamp and the provided
        #   timestamp variables
        delta_time = timestamp - self._last_timestamp
        
        if delta_time == 0:
            # Delta time is zero
            return 0
        
        # Calculate the error as the differnce between the _set_point and
        #   the measured_value
        error = self._set_point - measured_value
        
        # Set the last_timestamp_ to current timestamp
        self._last_timestamp = timestamp

        # Calculate the proportional error here. Be sure to access the
        #   internal Kp class variable
        p = self._kp * error

        # Set the control effort, u, which is the sum of all your errors. In
        #   this case it is just the proportional error.
        u = p
        
        # Here we are storing the control effort history for post control
        #   observations. 
        self.u_p.append(p)

        return u
