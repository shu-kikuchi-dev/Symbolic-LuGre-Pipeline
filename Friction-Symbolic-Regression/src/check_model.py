import pickle
import os
from pysr import PySRRegressor

# Target model name
model_name = "26-10-15-AM-09-35_withz_v13_dzdt_target-v-dzdt.pkl"

# PATH for a model
model_path = "../models/" + model_name

# Load the saved model
with open(model_path, "rb") as f:
    model = pickle.load(f)

# Show results
print("---Inside of the model---")
print(model)

# Show the detail just for the best equation
print("\n---The Best Equation---")
print(model.get_best()['equation']) # remove the limitation of length with ['equation']

# Show all equations within data frame method
# print(model.equations_)