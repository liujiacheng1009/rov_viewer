
class PID:
    def __init__(self):
        self.kp = 600
        self.ki = 100 
        self.kd = 50
        self.pwm_max = 1900 
        self.pwm_min = 1500

    def control_pid(self, value,target,delta_t):
        """PID controller
        Transform pressure to depth value
        Calulate the integrate value with euler method

        Input:
        ------
        p: absolute presssure in Pa

        Return:
        -------
        command calculated to reach the depth desired

        """
        delta_v =  target - value
        if delta_t == 0:
            D_v = 0
        else:
            D_v = delta_v/delta_t #derivative term 

        I_v = delta_v*delta_t #integrate term
        u = self.ki*self.I_v + self.kp*delta_v - self.kd*D_v
        return u


    