import sys
import os
import IPython as IP
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import librosa
import librosa.display
import pickle
from include import helpers

from sklearn.preprocessing import LabelEncoder
from sklearn.model_selection import train_test_split
from tensorflow.keras.utils import to_categorical
import math
import scipy.io.wavfile

us8k_path = os.path.abspath('./data')
audio_path = os.path.join(us8k_path, 'wav')
metadata_path = os.path.join(us8k_path, 'metadata/test.csv')

# Load the metadata from the generated CSV
metadata = pd.read_csv(metadata_path)
metadata.head()

# Iterate through all audio files and extract MFCC
features = []
labels = []
frames_max = 0
counter = 0
total_samples = len(metadata)
n_mels = 30
duration = 1 #second
counter2 = 0

frame_length = 16896
hop_length = 8448
overlap_ratio = hop_length/frame_length

for index, row in metadata.iterrows():
    file_path = os.path.join(os.path.abspath(audio_path), 'test', str(row["slice_file_name"]))
    class_label = row["class"]

    try:
        # Load audio file
        clip, sr = librosa.load(file_path, sr=16000, mono=True, dtype=np.float32)
        clip_length = len(clip)
        file_nums = math.floor(clip_length / hop_length) - 1

        for i in range(file_nums):
            y = clip[(i * hop_length):(i * hop_length + frame_length)]
            # Normalize audio data between -1 and 1
            #normalized_y = y
            #y = librosa.util.normalize(y)  # y=y/max(abs(y))

            # Compute Mel
            mel = librosa.feature.melspectrogram(y=y, sr=16000, n_fft=1024, hop_length=512,
                                                 n_mels=n_mels, center=False)
            mel = mel/mel.max()

            # Convert sound intensity to log amplitude:
            mel_db = librosa.power_to_db(mel, top_db=80)

            # Normalize MFCC between -1 and 1
            normalized_mel = mel_db
            # normalized_mfcc = librosa.util.normalize(mfcc)   #my_mfcc = mfcc/(np.max(abs(mfcc)))

            mels = normalized_mel

            # Extract MFCCs (do not add padding)
            # mfccs = helpers.get_mfcc(file_path, 0, n_mfcc)

            # Save current frame count
            num_frames = mels.shape[1]

            # Add row (feature / label)
            features.append(mels)
            labels.append(class_label)

            # Update frames maximum
            if (num_frames > frames_max):
                frames_max = num_frames
                print(f"num_frames = {num_frames}")
                print(f"file name : {file_path}")

            # Notify update every N files
            if (counter == 50):
                print("Status: {}/{}".format(index + 1, total_samples))
                counter = 0

            # original_name = str(row["slice_file_name"])
            # new_name = str(counter2)+'-'+original_name
            # split_path = os.path.join(os.path.abspath(audio_path), 'test_split', new_name)
            # scipy.io.wavfile.write(split_path, sr, y)

            counter += 1
            counter2+= 1
    except Exception as e:
        print("Error parsing wavefile: ", e)
        print(file_path)

print(f"frame_max = {frames_max}")
print("Finished: {}/{}".format(index, total_samples))

# Add padding to features with less than frames than frames_max
padded_features = helpers.add_padding(features, frames_max)


# Verify shapes
print("Raw features length: {}".format(len(features)))
print("Padded features length: {}".format(len(padded_features)))
print("Feature labels length: {}".format(len(labels)))


# Convert features (X) and labels (y) to Numpy arrays
X = np.array(padded_features)
y = np.array(labels)

# Optionally save the features to disk
np.save("data/X-mel-test", X)
np.save("data/y-mel-test", y)

le = LabelEncoder()
y_test_encoded = to_categorical(le.fit_transform(y))
y_encoded = pd.DataFrame(y_test_encoded)
y_encoded.to_csv('y_test_mel.csv', index=False, index_label=False, header=False)

X_reshape = X.reshape(len(padded_features), 30*32)
X = pd.DataFrame(X_reshape)
X.to_csv('X_test_mel.csv', index=False, index_label=False, header=False)

with open('parameter-test-mel.pickle', 'wb') as handle:
    pickle.dump([n_mels, frames_max], handle, protocol=pickle.HIGHEST_PROTOCOL)

print("End program")