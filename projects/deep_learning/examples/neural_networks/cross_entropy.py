import numpy as np


def cross_entropy(Y, P):
    Y = np.float_(Y)
    P = np.float_(P)
    entropy = -np.sum(Y * np.log(P) + (1 - Y) * np.log(1 - P))
    return entropy
