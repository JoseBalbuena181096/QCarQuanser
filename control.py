class ControlSystem:
    def __init__(self, kp=0.00225, ki=0.00015, kd=0.00075, setpoint=-22.452118490490875):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.integral = 0
        self.previous_error = 0
        self.max_integral = 100  # Anti-windup
        
    def control_p(self, ubicacion_linea):
        """Simple proportional control with improved gain"""
        error = self.setpoint + ubicacion_linea
        return error * self.kp
    
    def control_pid(self, ubicacion_linea):
        """PID control for smoother line following"""
        error = self.setpoint + ubicacion_linea
        
        # Proportional term
        p_term = error * self.kp
        
        # Integral term with anti-windup
        self.integral += error
        if self.integral > self.max_integral:
            self.integral = self.max_integral
        elif self.integral < -self.max_integral:
            self.integral = -self.max_integral
        i_term = self.ki * self.integral
        
        # Derivative term
        d_term = self.kd * (error - self.previous_error)
        self.previous_error = error
        
        # Calculate control output
        control_output = p_term + i_term + d_term
        
        return control_output

    @staticmethod
    def saturate(raw_steering, major, less):
        """Limit the steering value to within a range"""
        if raw_steering <= less:
            return less
        elif raw_steering >= major:
            return major
        return raw_steering

    def set_kp(self, new_kp):
        """Method to modify the kp value."""
        self.kp = new_kp

    def get_kp(self):
        """Method to retrieve the current kp value."""
        return self.kp
        
    def set_ki(self, new_ki):
        """Method to modify the ki value."""
        self.ki = new_ki
        
    def get_ki(self):
        """Method to retrieve the current ki value."""
        return self.ki
        
    def set_kd(self, new_kd):
        """Method to modify the kd value."""
        self.kd = new_kd
        
    def get_kd(self):
        """Method to retrieve the current kd value."""
        return self.kd
        
    def reset(self):
        """Reset the integral and previous error"""
        self.integral = 0
        self.previous_error = 0