import time
import timeit
start = timeit.default_timer()

import os
import datetime
import numpy as np
np.set_printoptions(suppress=True)
import os

from bpbot.device import DynPickClient, DynPickControl



dc = DynPickClient()
fs = DynPickControl()
#sensitivity = np.array(cfgdata["force_sensor"]["sensitivity"])


#wt=fs.record(stop=1)
#f_t=dc.get()
#print("loading valuess")
#print(wt[2])

#print(f_t)
for detected_load in fs.record(stop=1):
    wt_pre=detected_load[2]  # Accessing the third element directly during iteration

print("current weight ",wt_pre)

# Loop through the detected load values in real-time
for detected_load in fs.record(stop=10):
    if detected_load[2]<(wt_pre-0.3):
        print("final weight of spaghetti is ",detected_load[2])
        break
    print("Force in the z-axis:", detected_load[2])

print("finalllll weight of spaghetti is ",detected_load[2])

"""
time.sleep(1)
wt=fs.record(stop=5)
print(wt[2])
print("ovarimashita")
"""