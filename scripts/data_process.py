import matplotlib.pyplot as plt
import numpy as np
from globals import *

times = 0
file_name = (
    "_rc:" + str(WireVar.rc) + "_N:" + str(NUM) + "_times:" + str(times) + ".txt"
)

if __name__ == "__main__":
  with open("../data/yts" + file_name, "r") as file:
    