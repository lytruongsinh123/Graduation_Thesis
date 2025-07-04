import librosa
import librosa.display
import matplotlib.pyplot as plt
import math
import numpy as np

file_path = 'clip.wav'
frame_length = 16896
hop_length = 8448
overlap_ratio = hop_length/frame_length

clip, sr = librosa.load(file_path)
clip_length = len(clip)
file_nums = math.floor(clip_length/hop_length) - 1
y = []

for i in range(file_nums):
    frame = clip[(i*hop_length):(i*hop_length+frame_length)]
    y.append(frame)

#End program
print("end program")




