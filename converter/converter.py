import tensorflow as tf
import numpy as np
import argparse

def representative_dataset_generator():
    for _ in range(100):
        yield [np.random.rand(1, 416, 416, 3).astype(np.float32)]


def convert_model(saved_model_path, output_model_path):
    converter = tf.lite.TFLiteConverter.from_saved_model(saved_model_path)

    converter.optimizations = [tf.lite.Optimize.DEFAULT]

    converter.representative_dataset = representative_dataset_generator

    converter.target_spec.supported_ops = [
        tf.lite.OpsSet.TFLITE_BUILTINS_INT8,
        tf.lite.OpsSet.TFLITE_BUILTINS  # Allow some float ops
    ]

    converter.inference_input_type = tf.int8
    converter.inference_output_type = tf.float32

    tflite_hybrid_model = converter.convert()

    with open(output_model_path + 'model_hybrid_quantized.tflite', 'wb') as f:
        f.write(tflite_hybrid_model)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert a TensorFlow SavedModel to TFLite with integer quantization and float outputs.")
    parser.add_argument("saved_model_path", type=str, help="Path to the SavedModel directory.")
    parser.add_argument("output_model_path", type=str, help="Path to save the quantized TFLite model.")
    args = parser.parse_args()

    convert_model(args.saved_model_path, args.output_model_path)
