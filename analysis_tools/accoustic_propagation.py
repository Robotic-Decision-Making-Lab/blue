import numpy as np 
import matplotlib.pyplot as plt

f = 69 #kHz
alpha = 10.3 #(ATTENUATION COEFFICIENT) assumes 69kHz signal in salt water
#note that alpha is in dB/km


#NOTE THAT R MUST BE IN METERS
def get_pressure_loss(alpha, r):
    return 20 * np.log10(r) + alpha * r * 10**-3

print(get_pressure_loss(alpha, 300))