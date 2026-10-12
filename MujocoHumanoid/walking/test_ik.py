from inverse_kinematics import leg_ik

hip, knee, ankle = leg_ik(

    0.05,
    -0.75

)

print(hip)
print(knee)
print(ankle)