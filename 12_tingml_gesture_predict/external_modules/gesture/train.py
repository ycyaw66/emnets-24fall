import glob
import tensorflow as tf
import tensorflow.keras as keras
import numpy as np
from sklearn.model_selection import train_test_split
from tensorflow.keras.callbacks import ModelCheckpoint
from tensorflow.keras.models import load_model
import os

# 设置环境变量，控制日志级别
# 0 = INFO, 1 = WARNING, 2 = ERROR, 3 = FATAL
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
DATA_PATH = "../../../10_tingml_datasets/"

LABELS = ["Stationary", "Tilted", "Rotating", "Moving"]
SAMPLES_PER_GESTURE = 10
def load_one_label_data(label):
    path = DATA_PATH + label + "*.npy"
    files = glob.glob(path)
    datas = []
    for file in files:
        print(file)
        try:
            data = np.load(file)
            num_slice = len(data) // SAMPLES_PER_GESTURE
            datas.append(data[: num_slice * SAMPLES_PER_GESTURE, :])
        except Exception as e:
            print(e)
    datas = np.concatenate(datas, axis=0)
    datas = np.reshape(
        datas,
        (
            -1,
            6 * SAMPLES_PER_GESTURE,
        ),
    )  # Modified here
    idx = LABELS.index(label)
    labels = np.ones(datas.shape[0]) * idx
    return datas, labels


all_datas = []
all_labels = []
for label in LABELS:
    datas, labels = load_one_label_data(label)
    print(label)
    all_datas.append(datas)
    all_labels.append(labels)

dataX = np.concatenate(all_datas, axis=0)
dataY = np.concatenate(all_labels, axis=0)
print(dataX.shape, dataY.shape)

xTrain, xTest, yTrain, yTest = train_test_split(
    dataX, dataY, test_size=0.2, stratify=dataY
)
def mlp():
    model = keras.Sequential()
    model.add(keras.layers.Dense(16, activation="relu", input_shape=(6 * SAMPLES_PER_GESTURE,)))
    model.add(keras.layers.Dropout(0.2))
    model.add(keras.layers.Dense(len(LABELS), activation="softmax"))
    return model
model = mlp()
model.summary()


optimizer = keras.optimizers.Adam(lr=0.001)
model.compile(
    loss="sparse_categorical_crossentropy",
    optimizer=optimizer,
    metrics=["sparse_categorical_accuracy"],
)

# Specify the path to save the model
filepath = "best_model.h5"

# Create a ModelCheckpoint callback
checkpoint = ModelCheckpoint(
    filepath,
    monitor="val_sparse_categorical_accuracy",
    verbose=1,
    save_best_only=True,
    mode="max",
)

# Add the callback to the fit function
history = model.fit(
    xTrain,
    yTrain,
    batch_size=8,
    validation_data=(xTest, yTest),
    epochs=50,
    verbose=1,
    callbacks=[checkpoint],
)
print(xTrain.shape, yTrain.shape)
# history = model.fit(xTrain, yTrain, batch_size=4, validation_data=(xTest, yTest), epochs=100, verbose=1, callbacks=[early_stop])

from sklearn.metrics import confusion_matrix

model = load_model(filepath)
# Get model predictions
predictions = model.predict(xTest)
predictions = np.argmax(predictions, axis=1)

# Get confusion matrix
cm = confusion_matrix(yTest, predictions)
print(cm)
# converter = tf.lite.TFLiteConverter.from_keras_model(model)
# tflite_model = converter.convert()
# open("model_basic.tflite", "wb").write(tflite_model)

#  Convert the model to the TensorFlow Lite format with quantization
converter = tf.lite.TFLiteConverter.from_keras_model(model)

# Reshape the data
data_train = xTrain.astype("float32")
data_train = np.reshape(data_train, (-1, 6 * SAMPLES_PER_GESTURE, 1))

data_ds = tf.data.Dataset.from_tensor_slices((data_train)).batch(1)


# Rest of your code...
def representative_data_gen():
    for input_value in data_ds.take(100):
        yield [input_value]


converter.representative_dataset = representative_data_gen
converter.optimizations = [tf.lite.Optimize.OPTIMIZE_FOR_SIZE]
tflite_model = converter.convert()
# # Save the quantized model to disk
import os

open("model.tflite", "wb").write(tflite_model)
basic_model_size = os.path.getsize("model_basic.tflite")
print("Basic model is %d bytes" % basic_model_size)
quantized_model_size = os.path.getsize("model.tflite")
print("Quantized model is %d bytes" % quantized_model_size)
difference = basic_model_size - quantized_model_size
print("Difference is %d bytes" % difference)

# Now let's verify the model on a few input digits
# Instantiate an interpreter for the model
model_quantized_reloaded = tf.lite.Interpreter("model.tflite")

# Allocate memory for each model
model_quantized_reloaded.allocate_tensors()

# Get the input and output tensors so we can feed in values and get the results
model_quantized_input = model_quantized_reloaded.get_input_details()[0]["index"]
model_quantized_output = model_quantized_reloaded.get_output_details()[0]["index"]
# Create arrays to store the results
model_quantized_predictions = np.empty(xTest.size)
for i in range(20):
    # Reshape the data and ensure the type is float32
    test_data = np.reshape(
        xTest[i],
        (
            1,
            6 * SAMPLES_PER_GESTURE,
            1,
        ),
    ).astype("float32")
    print(test_data.shape)
    # Invoke the interpreter
    model_quantized_reloaded.set_tensor(model_quantized_input, test_data)
    model_quantized_reloaded.invoke()
    model_quantized_prediction = model_quantized_reloaded.get_tensor(
        model_quantized_output
    )

    print("Digit: {} - Prediction:\n{}".format(yTest[i], model_quantized_prediction))
    print("")
