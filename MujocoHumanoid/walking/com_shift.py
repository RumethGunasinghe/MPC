import numpy as np

SHIFT_AMOUNT = 0.05

def get_com_shift(phase, progress):

    if phase == "LEFT_SUPPORT":
        return SHIFT_AMOUNT

    return -SHIFT_AMOUNT