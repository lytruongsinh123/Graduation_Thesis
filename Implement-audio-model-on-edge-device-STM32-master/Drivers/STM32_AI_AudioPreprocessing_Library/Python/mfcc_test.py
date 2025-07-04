import librosa
import numpy as np
import matplotlib.pyplot as plt
import librosa.display

ys, sr = librosa.load("03a01285.wav", sr=16000, mono=True, dtype=np.float32)
ys = ys[:1024]
#S = librosa.feature.melspectrogram(y=ys, sr=sr, n_fft=1024, n_mels=30,
#                                   hop_length=512, center=False)
#S_log = librosa.power_to_db(S)

mfcc = librosa.feature.mfcc(y=ys, sr=sr, n_fft=1024,  n_mels=30, hop_length=512,
                            center=False, n_mfcc=30, dct_type=3, norm=None)

#end program
print("end program!")