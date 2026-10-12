from walking_gait import WalkingGait

gait = WalkingGait()

for i in range(100):

    phase = gait.update(0.02)

    print(
        i,
        phase,
        gait.get_phase_progress()
    )