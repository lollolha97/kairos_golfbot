import tensorflow as tf

print("Num GPUs Available: ", len(tf.config.experimental.list_physical_devices('GPU')))

# GPU가 사용 가능한지 확인
if tf.config.experimental.list_physical_devices('GPU'):
    print("GPU is available.")
else:
    print("GPU is not available.")