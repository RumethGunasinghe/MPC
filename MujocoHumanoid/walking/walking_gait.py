class WalkingGait:

    def __init__(self):

        self.phase = "LEFT_SUPPORT"

        self.phase_time = 0.0

        self.step_time = 0.5

    def update(self, dt):

        self.phase_time += dt

        if self.phase_time >= self.step_time:

            if self.phase == "LEFT_SUPPORT":

                self.phase = "RIGHT_SUPPORT"

            else:

                self.phase = "LEFT_SUPPORT"

            self.phase_time = 0.0

        return self.phase

    def get_phase_progress(self):

        return self.phase_time / self.step_time