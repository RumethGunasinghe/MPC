import numpy as np

THIGH = 0.40
SHIN = 0.40


def leg_ik(

    x,
    z
):

    D = (

        x*x
        +
        z*z
        -
        THIGH**2
        -
        SHIN**2

    ) / (

        2 *
        THIGH *
        SHIN

    )

    D = np.clip(

        D,
        -1,
        1
    )

    knee = np.arccos(D)

    hip = (

        np.arctan2(z, x)

        -

        np.arctan2(

            SHIN*np.sin(knee),

            THIGH + SHIN*np.cos(knee)

        )

    )

    ankle = -(hip + knee)

    

    return (

        hip,
        knee,
        ankle
    )