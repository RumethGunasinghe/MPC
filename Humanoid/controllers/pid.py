import numpy as np

class PIDHumanoid:
    def __init__(self, Kp=200, Kd=40):
        self.Kp = Kp
        self.Kd = Kd

    def compute(self, torso_angle, torso_velocity):
        return self.Kp * torso_angle + self.Kd * torso_velocity