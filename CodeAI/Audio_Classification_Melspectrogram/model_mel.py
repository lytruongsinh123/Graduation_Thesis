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
from tensorflow.keras.layers import Dense, SpatialDropout2D, Activation, Conv2D, MaxPooling2D, BatchNormalization, GlobalAveragePooling2D, LeakyReLU, GlobalMaxPooling2D
from tensorflow.keras.optimizers import Adam
from tensorflow.keras.utils import to_categorical
from tensorflow.keras.callbacks import ModelCheckpoint
from tensorflow.keras.regularizers import l2

from sklearn.preprocessing import LabelEncoder
from sklearn.metrics import classification_report, confusion_matrix

tf.compat.v1.disable_v2_behavior()

# Set your path to the dataset
us8k_path = os.path.abspath('./data')
audio_path = os.path.join(us8k_path, 'wav/train')
metadata_path = os.path.join(us8k_path, 'metadata/train.csv')
models_path = os.path.abspath('./models')
data_path = os.path.abspath('./data')

# Ensure "channel last" data format on Keras
keras_backend.set_image_data_format('channels_last')

# Define a labels array for future use
labels = [
    'Computer_Keyboard',
    'Knock',
    'Talking',
    'Telephone',
    'Glass_Breaking'
    ]

# Pre-processed MFCC coefficients
X = np.load("data/X-mel.npy")
y = np.load("data/y-mel.npy")

# Metadata
metadata = pd.read_csv(metadata_path)

#total = len(metadata)
total = X.shape[0]
indexes = list(range(0, total))

# Randomize indexes
random.shuffle(indexes)

# Divide the indexes into Train and Test
test_split_pct = 10
split_offset = math.floor(test_split_pct * total / 100)

# Split the metadata
test_split_idx = indexes[0:split_offset]
train_split_idx = indexes[split_offset:total]


# Split the features with the same indexes
X_test = np.take(X, test_split_idx, axis=0)
y_test = np.take(y, test_split_idx, axis=0)
X_train = np.take(X, train_split_idx, axis=0)
y_train = np.take(y, train_split_idx, axis=0)

# Also split metadata
#test_meta = metadata.iloc[test_split_idx]
#train_meta = metadata.iloc[train_split_idx]

# Print status
#print("Test split: {} \t\t Train split: {}".format(len(test_meta), len(train_meta)))
print("X test shape: {} \t X train shape: {}".format(X_test.shape, X_train.shape))
print("y test shape: {} \t\t y train shape: {}".format(y_test.shape, y_train.shape))


le = LabelEncoder()
y_test_encoded = to_categorical(le.fit_transform(y_test))
y_train_encoded = to_categorical(le.fit_transform(y_train))

with open('data/parameter-mel.pickle', 'rb') as handle:
    num_rows, num_columns = pickle.load(handle)
num_channels = 1

# Reshape to fit the network input (channel last)
X_train = X_train.reshape(X_train.shape[0], num_rows, num_columns, num_channels)
X_test = X_test.reshape(X_test.shape[0], num_rows, num_columns, num_channels)

# Total number of labels to predict (equal to the network output nodes)
num_labels = y_train_encoded.shape[1]

#MODEL
def create_model(spatial_dropout_rate_1=0, spatial_dropout_rate_2=0, l2_rate=0):
        # Create a secquential object
        model = Sequential()

        # Conv 1
        model.add(Conv2D(filters=16,
                         kernel_size=(3, 3),
                         kernel_regularizer=l2(l2_rate),
                         input_shape=(num_rows, num_columns, num_channels)))
        model.add(LeakyReLU(alpha=0.1))
        model.add(BatchNormalization())

        model.add(SpatialDropout2D(spatial_dropout_rate_1))
        model.add(Conv2D(filters=16,
                         kernel_size=(3, 3),
                         kernel_regularizer=l2(l2_rate)))
        model.add(LeakyReLU(alpha=0.1))
        model.add(BatchNormalization())

        # Max Pooling #
        model.add(MaxPooling2D(pool_size=(2, 2)))

        model.add(SpatialDropout2D(spatial_dropout_rate_1))
        model.add(Conv2D(filters=32,
                         kernel_size=(3, 3),
                         kernel_regularizer=l2(l2_rate)))
        model.add(LeakyReLU(alpha=0.1))
        model.add(BatchNormalization())

        model.add(SpatialDropout2D(spatial_dropout_rate_2))
        model.add(Conv2D(filters=32,
                         kernel_size=(3, 3),
                         kernel_regularizer=l2(l2_rate)))
        model.add(LeakyReLU(alpha=0.1))
        model.add(BatchNormalization())

        # Reduces each h×w feature map to a single number by taking the average of all h,w values.
        model.add(GlobalMaxPooling2D())

        # Softmax output
        model.add(Dense(num_labels, activation='softmax'))

        return model


# Regularization rates
spatial_dropout_rate_1 = 0.07
spatial_dropout_rate_2 = 0.14
l2_rate = 0.0005

model = create_model(spatial_dropout_rate_1, spatial_dropout_rate_2, l2_rate)

adam = Adam(lr=1e-4, beta_1=0.99, beta_2=0.999)
model.compile(
    loss='categorical_crossentropy',
    metrics=['accuracy'],
    optimizer=adam)

# Display model architecture summary
model.summary()
tf.keras.utils.plot_model(
    model, to_file='model.png', show_shapes=True, show_layer_names=True,
    rankdir='TB'
)

num_epochs = 350
num_batch_size = 128
model_file = 'my_model_mel.h5'
model_path = os.path.join(models_path, model_file)


# Save checkpoints
checkpointer = ModelCheckpoint(filepath=model_path,
                               verbose=1,
                               save_best_only=True)
start = datetime.now()
history = model.fit(X_train,
                    y_train_encoded,
                    batch_size=num_batch_size,
                    epochs=num_epochs,
                    validation_split=1/12.,
                    callbacks=[checkpointer],
                    verbose=1)
duration = datetime.now() - start
print("Training completed in time: ", duration)


# Load best saved model
model = load_model(model_path)

helpers.model_evaluation_report(model, X_train, y_train_encoded, X_test, y_test_encoded)

helpers.plot_train_history(history, x_ticks_vertical=True)

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

helpers.plot_confusion_matrix(cm,
                          labels,
                          normalized=False,
                          title="Model Performance",
                          cmap=plt.cm.Blues,
                          size=(12,12))
