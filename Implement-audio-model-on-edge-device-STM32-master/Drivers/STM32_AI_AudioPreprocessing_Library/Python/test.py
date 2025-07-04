import librosa
import numpy as np
import matplotlib.pyplot as plt
import librosa.display

ys, sr = librosa.load("02fb6c5b.wav", sr=16000, mono=True, dtype=np.float32)

ys = ys[:16896]

S_mel = librosa.feature.melspectrogram(y=ys, sr=sr, n_fft=1024,
                                       hop_length=512, n_mels=30, center=False)
S_mel = S_mel / S_mel.max()
S_log_mel = librosa.power_to_db(S_mel, top_db=80)

# Plot first feature/spectrogram
plt.figure(figsize=(10, 4))
librosa.display.specshow(S_log_mel, sr=16000, y_axis='mel', fmax=8000, x_axis='time', cmap='viridis')
plt.colorbar(format='%+2.0f dB')
plt.title('Mel spectrogram')
plt.tight_layout()
plt.show()

#endprogram
print("End program")