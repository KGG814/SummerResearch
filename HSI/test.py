import cv2
import numpy as np
from matplotlib import pyplot as plt
from math import asin,atan2,degrees,sqrt, sin 
import pickle
import os
import string
import math
import sys
from subprocess import Popen, PIPE
directory = os.getcwd() + "/HSI_Viewer.exe"
print directory
pipe = Popen(directory, shell=True, bufsize=100, stdin=PIPE).stdin
