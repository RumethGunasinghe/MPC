import numpy as np

STEP_LENGTH = 0.08
STEP_HEIGHT = 0.03


class FootTrajectory:

    def generate(

        self,
        progress
    ):

        x = STEP_LENGTH * progress

        z = STEP_HEIGHT * np.sin(

            np.pi * progress
        )

        return x, z