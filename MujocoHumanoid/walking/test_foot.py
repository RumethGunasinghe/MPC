from foot_trajectory import FootTrajectory

traj = FootTrajectory()

for i in range(20):

    p = i / 20

    x, z = traj.generate(p)

    print(

        f"progress={p:.2f}",
        f"x={x:.3f}",
        f"z={z:.3f}"

    )