class ThresholdController:
    def __init__(self, lambda_low, lambda_high, max_torque):
        self.lambda_low = lambda_low
        self.lambda_high = lambda_high
        self.max_torque = max_torque
        self.previous_torque = max_torque

    def calculate_brake_torque(self, slip_ratio):
        if slip_ratio > self.lambda_high:
            self.previous_torque = 0.0
        elif slip_ratio < self.lambda_low:
            self.previous_torque = self.max_torque

        return self.previous_torque
