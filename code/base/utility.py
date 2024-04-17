import example_robot_data as robex
import os

def get_model_file_path(model_file_path):
  model_dir = robex.getModelPath(model_file_path)
  return os.path.join(model_dir, model_file_path.strip('/')), os.path.dirname(os.path.dirname(model_dir))
