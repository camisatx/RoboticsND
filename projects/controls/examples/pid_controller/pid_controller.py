##################################################################################
#
# No changes need to be made to the this code. Yay! 
#
##################################################################################

class PID_Controller:

    def __init__(self, kp=0.0, ki=0.0,  kd=0.0, start_time=0):
        
        # The PID controller can be initalized with a specific kp value
        # ki value, and kd value
        self.kp_ = float(kp)
        self.ki_ = float(ki)
        self.kd_ = float(kd)

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

    def setTarget(self, target):
        self.set_point_ = float(target)

    def setKP(self, kp):
        self.kp_ = float(kp)
        
    def setKI(self, ki):
        self.ki_ = float(ki)
       
    def setKD(self, kd):
        self.kd_ = float(kd)

    def update(self, measured_value, timestamp):
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
        
        # Calculate the delta_error
        delta_error = error - self.last_error_
        
        # Update the past error with the current error
        self.last_error_ = error
        
        # Proportional error
        p = self.kp_ * error
       
        # Integral error
        i = self.ki_ * self.error_sum_
       
        # Derivative error 
        d = self.kd_ * (delta_error / delta_time)

        # Set the control effort
        u = p + i + d
        
        # Here we are storing the control effort history for post control
        # observations. 
        self.u_p.append(p)
        self.u_i.append(i)
        self.u_d.append(d)

        return u
