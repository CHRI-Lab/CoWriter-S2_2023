#######################################
# This method did work well, plz dont use it
########################################


import matplotlib.pyplot as plt
import pandas as pd
from PIL import Image
import numpy as np
import math
import os
from shutil import copyfile
import keras
from keras.models import Sequential
from keras.layers import Dense
from keras.preprocessing.image import ImageDataGenerator
from keras.models import load_model
import tensorflow as tf
from keras.layers import Conv2D, MaxPooling2D, Flatten, Dense
from keras.layers import Flatten


# if not os.path.isdir('dataset'):
#   os.mkdir('dataset')
#
# if not os.path.isdir('dataset/train'):
#   os.mkdir('dataset/train')
# if not os.path.isdir('dataset/valid'):
#   os.mkdir('dataset/valid')
# if not os.path.isdir('dataset/test'):
#   os.mkdir('dataset/test')
#
# for i in sorted(os.listdir('English/Fnt')):
#   if not os.path.isdir('dataset/train/'+i):
#     os.mkdir('dataset/train/'+i)
#   if not os.path.isdir('dataset/valid/'+i):
#     os.mkdir('dataset/valid/'+i)
#   if not os.path.isdir('dataset/test/'+i):
#     os.mkdir('dataset/test/'+i)
#
# base = 'English/Fnt/Sample'
#
# for char  in range(1, 63):
#   classLen = len(os.listdir(base + str(char).zfill(3)))
#
#   trainLen = math.floor(classLen*0.80)
#   validLen = math.ceil(classLen*0.15)
#
#   randFnt = np.random.randint(low = 1, high = classLen, size = classLen)
#   randTrain = randFnt[:trainLen]
#   randValid = randFnt[trainLen : trainLen+validLen]
#   randTest = randFnt[trainLen+validLen :]
#
#   for imgNo in randTrain:
#     src = base+str(char).zfill(3)+'/img'+str(char).zfill(3)+'-'+str(imgNo).zfill(5)+'.png'
#     des = 'dataset/train/Sample'+str(char).zfill(3)+'/img'+str(char).zfill(3)+'-'+str(imgNo).zfill(5)+'.png'
#     copyfile(src, des)
#
#   for imgNo in randValid:
#     src = base+str(char).zfill(3)+'/img'+str(char).zfill(3)+'-'+str(imgNo).zfill(5)+'.png'
#     des = 'dataset/valid/Sample'+str(char).zfill(3)+'/img'+str(char).zfill(3)+'-'+str(imgNo).zfill(5)+'.png'
#     copyfile(src, des)
#
#   for imgNo in randTest:
#     src = base+str(char).zfill(3)+'/img'+str(char).zfill(3)+'-'+str(imgNo).zfill(5)+'.png'
#     des = 'dataset/test/Sample'+str(char).zfill(3)+'/img'+str(char).zfill(3)+'-'+str(imgNo).zfill(5)+'.png'
#     copyfile(src, des)

num_classes = 62
image_resize = 128
batch_size_training = 128
batch_size_validation = 64

data_generator = ImageDataGenerator(rescale=1.0 / 255.0)

train_generator = data_generator.flow_from_directory(
  'dataset/train',
  target_size=(image_resize, image_resize),
  batch_size=batch_size_training,
  color_mode='grayscale',
  class_mode='categorical'
)

validation_generator = data_generator.flow_from_directory(
  'dataset/valid',
  target_size=(image_resize, image_resize),
  batch_size=batch_size_training,
  color_mode='grayscale',
  class_mode='categorical'
)

def ocrModel():
  model = Sequential()
  model.add(Conv2D(32, (4, 4), strides=(1, 1), activation='relu', input_shape=(128, 128, 1)))
  model.add(MaxPooling2D(pool_size=(4, 4), strides=(2, 2)))
  model.add(Conv2D(64, (4, 4), strides=(1, 1), activation='relu', input_shape=(128, 128, 1)))
  model.add(MaxPooling2D(pool_size=(4, 4), strides=(2, 2)))

  model.add(Flatten())

  model.add(Dense(310, activation='relu'))
  model.add(Dense(num_classes, activation='softmax'))

  model.compile(optimizer='adam', loss='categorical_crossentropy', metrics=['accuracy'])

  return model

steps_per_epoch_training = len(train_generator)//2
steps_per_epoch_validation = len(train_generator)//2
num_epochs = 3

model = ocrModel()

print(len(train_generator))
fit_history = model.fit(
  train_generator,
  steps_per_epoch=steps_per_epoch_training,
  epochs=num_epochs,
  validation_data=validation_generator,
  validation_steps=steps_per_epoch_validation,
  verbose=1
)

classArr = [str(i) for i in range(10)]
classArr.extend([chr(i) for i in range(ord('A'), ord('Z')+1)])
classArr.extend([chr(i) for i in range(ord('a'), ord('z')+1)])

res = ''
img = Image.open('cat-c.jpg')
imgnp = np.array(img)
imgnp = np.reshape(imgnp, (1,imgnp.shape[0],imgnp.shape[1], 1))
predict = model.predict(imgnp)
predict = predict[0]

print (predict)
print (max(predict))

index = np.argmax(predict)
res += classArr[index]
# img = Image.open('cat-ao.jpg')
# imgnp = np.array(img)
# imgnp = np.reshape(imgnp, (1,imgnp.shape[0],imgnp.shape[1], 1))
# predict = model.predict(imgnp)
# res += classArr[predict[0]]
# img = Image.open('cat-t.jpg')
# imgnp = np.array(img)
# imgnp = np.reshape(imgnp, (1,imgnp.shape[0],imgnp.shape[1], 1))
# predict = model.predict(imgnp)
# res += classArr[predict[0]]

print(res)