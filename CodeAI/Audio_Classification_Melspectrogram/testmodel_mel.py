import os
import IPython
import math
import numpy as np
import pandas as pd
import pickle
import matplotlib.pyplot as plt
import tensorflow as tf

import random
from datetime import datetime
from include import helpers

from tensorflow.keras import backend as keras_backend
from tensorflow.keras.models import Sequential, load_model
from tensorflow.keras.layers import Dense, SpatialDropout2D, Activation, Conv2D, MaxPooling2D, BatchNormalization, GlobalAveragePooling2D, LeakyReLU
from tensorflow.keras.optimizers import Adam
from tensorflow.keras.utils import to_categorical
from tensorflow.keras.callbacks import ModelCheckpoint
from tensorflow.keras.regularizers import l2

from sklearn.preprocessing import LabelEncoder
from sklearn.metrics import classification_report, confusion_matrix

tf.compat.v1.disable_v2_behavior()

# Set your path to the dataset
models_path = os.path.abspath('./models')
data_path = os.path.abspath('./data')

# Define a labels array for future use
labels = [
    'Computer_Keyboard',
    'Knock',
    'Talking',
    'Telephone',
    'Glass_Breaking'
    ]

model_file = 'my_model_mel.h5'
model_path = os.path.join(models_path, model_file)

X_test = np.load("data/X-mel-test.npy")
y_test = np.load("data/y-mel-test.npy")

X_test = X_test.reshape(X_test.shape[0], 30, 32, 1)

le = LabelEncoder()
y_test_encoded = to_categorical(le.fit_transform(y_test))

# Load best saved model
model = load_model(model_path)

# Predict probabilities for test set
y_probs = model.predict(X_test, verbose=0)

# Get predicted labels
yhat_probs = np.argmax(y_probs, axis=1)
y_trues = np.argmax(y_test_encoded, axis=1)

# Add "pred" column
#test_meta['pred'] = yhat_probs

# Sets decimal precision (for printing output only)
np.set_printoptions(precision=2)

# Compute confusion matrix data
cm = confusion_matrix(y_trues, yhat_probs)

count = 0
for i in range(len(y_trues)):
    if (y_trues[i] != yhat_probs[i]):
        print(f"i:{i}, y_predicted:{yhat_probs[i]}, y: {y_trues[i]}, y_prob: {y_probs[i][yhat_probs[i]]}")
        count+=1
print("count = ", count)

helpers.plot_confusion_matrix(cm,
                          labels,
                          normalized=False,
                          title="Model Performance",
                          cmap=plt.cm.Blues,
                          size=(12,12))