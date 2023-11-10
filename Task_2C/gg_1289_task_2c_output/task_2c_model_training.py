import numpy as np
import cv2

import os

import matplotlib.pylab as plt

import tensorflow as tf
import tensorflow_hub as hub
from tensorflow.keras.preprocessing.image import ImageDataGenerator
from tensorflow.keras.layers import Conv2D,MaxPool2D,Dropout,concatenate, ReLU, AvgPool2D, Dense, Flatten
from tensorflow import keras
from tensorflow.keras import layers
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Input
from tensorflow.keras.models import Model

import glob

IMAGE_SHAPE = (224, 224)

'''classifier = tf.keras.Sequential([
    hub.KerasLayer("https://tfhub.dev/google/tf2-preview/mobilenet_v2/classification/4", input_shape=IMAGE_SHAPE+(3,))
])'''

train_images_dict = {
    'combat': list(glob.glob('training/Combat/*jpeg')),
    'destroyedbuidings': list(glob.glob('training/DestroyedBuildings/*jpeg')),
    'fire': list(glob.glob('training/Fire/*jpeg')),
    'humaniatrianaid': list(glob.glob('training/Humanitarian Aid and rehabilitation/*jpeg')),
    'military': list(glob.glob('training/Military vehicles and weapons/*jpeg')),
}
test_images_dict = {
    'combat': list(glob.glob('test/Combat/*jpeg')),
    'destroyedbuidings': list(glob.glob('test/DestroyedBuildings/*jpeg')),
    'fire': list(glob.glob('test/Fire/*jpeg')),
    'humaniatrianaid': list(glob.glob('test/Humanitarian Aid and rehabilitation/*jpeg')),
    'military': list(glob.glob('test/Military vehicles and weapons/*jpeg')),
}



labels_dict = {
    'combat': 0,
    'destroyedbuidings': 1,
    'fire': 2,
    'humaniatrianaid': 3,
    'military': 4,
}

x_test, y_test,x_train,y_train = [], [],[],[]

for name, images in train_images_dict.items():
    for image in images:
        img = cv2.imread(str(image))
        resized_img = cv2.resize(img,(224,224))
        x_train.append(resized_img)
        y_train.append(labels_dict[name])
for name, images in test_images_dict.items():
    for image in images:
        img = cv2.imread(str(image))
        resized_img = cv2.resize(img,(224,224))
        x_test.append(resized_img)
        y_test.append(labels_dict[name])



x_train = np.array(x_train)
y_train = np.array(y_train)
x_test = np.array(x_test)
y_test = np.array(y_test)

X_train_scaled = x_train / 255
X_test_scaled = x_test / 255

feature_extractor_model = "https://tfhub.dev/google/tf2-preview/mobilenet_v2/classification/4"

pretrained_model_without_top_layer = hub.KerasLayer(
feature_extractor_model, input_shape=(224, 224, 3), trainable=False)
number_of_classes = 5


datagen = ImageDataGenerator(
    rotation_range=20,
    width_shift_range=0.1,
    height_shift_range=0.1,
    shear_range=0.2,
    zoom_range=0.2,
    horizontal_flip=True,
    fill_mode='nearest'
)

# Create a generator for training data with data augmentation
datagen.fit(X_train_scaled)

# Build your model
model = tf.keras.Sequential([
  pretrained_model_without_top_layer,
  tf.keras.layers.Dense(128, activation='relu'),
  tf.keras.layers.Dropout(0.5),  # Add dropout
  tf.keras.layers.Dense(128, activation='relu'),
  tf.keras.layers.Dropout(0.5),  # Add dropout
  tf.keras.layers.Dense(number_of_classes, activation='softmax')
])


# Compile the model
model.compile(
    optimizer="adam",
    loss=tf.keras.losses.SparseCategoricalCrossentropy(),
    metrics=['acc']
)

# Train the model with data augmentation
history = model.fit(datagen.flow(X_train_scaled, y_train, batch_size=32),
                    steps_per_epoch=len(X_train_scaled) / 32,  # Adjust batch size if needed
                    epochs=5)

# Evaluate the model on the test data
model.evaluate(X_test_scaled, y_test)

# Save model weights
model.save_weights('my_model_weights_with_augmentation.h5')




