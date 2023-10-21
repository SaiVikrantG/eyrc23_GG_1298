import numpy as np
import cv2

import os

import matplotlib.pylab as plt

import tensorflow as tf
import tensorflow_hub as hub

from tensorflow import keras
from tensorflow.keras import layers
from tensorflow.keras.models import Sequential
import glob

IMAGE_SHAPE = (224, 224)

classifier = tf.keras.Sequential([
    hub.KerasLayer("https://tfhub.dev/google/tf2-preview/mobilenet_v2/classification/4", input_shape=IMAGE_SHAPE+(3,))
])

train_images_dict = {
    'combat': list(glob.glob('dataset_eyrc/Combat/*jpeg')),
    'destroyedbuidings': list(glob.glob('dataset_eyrc/DestroyedBuildings/*jpeg')),
    'fire': list(glob.glob('dataset_eyrc/Fire/*jpeg')),
    'humaniatrianaid': list(glob.glob('dataset_eyrc/Humanitarian Aid and rehabilitation/*jpeg')),
    'military': list(glob.glob('dataset_eyrc/Military vehicles and weapons/*jpeg')),
}
test_images_dict = {
    'combat': list(glob.glob('test_dataset_eyrc/Combat/*jpeg')),
    'destroyedbuidings': list(glob.glob('test_dataset_eyrc/DestroyedBuildings/*jpeg')),
    'fire': list(glob.glob('test_dataset_eyrc/Fire/*jpeg')),
    'humaniatrianaid': list(glob.glob('test_dataset_eyrc/Humanitarian Aid and rehabilitation/*jpeg')),
    'military': list(glob.glob('test_dataset_eyrc/Military vehicles and weapons/*jpeg')),
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

feature_extractor_model = "https://tfhub.dev/google/tf2-preview/mobilenet_v2/feature_vector/4"

pretrained_model_without_top_layer = hub.KerasLayer(
feature_extractor_model, input_shape=(224, 224, 3), trainable=False)
number_of_classes = 5

model = tf.keras.Sequential([
  pretrained_model_without_top_layer,
  tf.keras.layers.Dense(number_of_classes)
])

# model.summary()
model.compile(
  optimizer="adam",
  loss=tf.keras.losses.SparseCategoricalCrossentropy(from_logits=True),
  metrics=['acc'])

model.fit(X_train_scaled, y_train, epochs=5)

model.evaluate(X_test_scaled,y_test)
model.save_weights('my_model_weights.h5')



