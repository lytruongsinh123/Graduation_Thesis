import numpy as np
import librosa

a = np.array([[1, 2, 3, 4],
              [5, 6, 7, 8]])

print(a)
a = np.pad(a, pad_width=((0, 0), (2, 3)), mode='constant')
print(a)