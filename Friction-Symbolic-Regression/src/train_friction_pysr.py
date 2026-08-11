import pandas as pd
import numpy as np
from pysr import PySRRegressor
import os
import pickle
from  pathlib import Path

# --- MODEL NAMING ---
version = "v9"
explanation = "try-more-massive-maxsize-from-20-to-30-to-get-a-term-of-stribeck-effect-without-overfitting"
#target = "F"
target = "dzdt"
model_id = "26-08-10_withz_" + version + "_" + target + "_" + explanation

# --- SETUP PATHS ---
# Update below file name when you use a new dataset
dataset_csv_name = "26-08-10_script-generatepysrdatawithz_refine-to-have-35-50-15-ratio_ode23tbf_maxstepsize-1en4_relativetolerance-1en7_absolutetolerance-1en10.csv"
DATA_PATH = os.path.join("..", "..", "LuGre_Sim", "outputs", "csv_files", "datasets-withz", dataset_csv_name)
MODEL_DIR = "../models/"
CONFIG_DIR = "../models/configs/"
os.makedirs(MODEL_DIR, exist_ok=True)
os.makedirs(CONFIG_DIR, exist_ok=True)

def save_model_config(model, model_id):
    # Save the PySR configuration parameters to a text file.
    config_path = os.path.join(CONFIG_DIR, f"{model_id}_config.txt")
    params = model.get_params()

    with open(config_path, "w", encoding="utf-8") as f:
        f.write(f"PySR Model Configuration Log\n")
        f.write(f"Model ID: {model_id}\n")
        f.write("============================================\n")
        for key, value in params.items():
            f.write(f"{key}: {value}\n")
    print(f"Configuration saved to: {config_path}")

def main():
    # --- DATA LOADING & CLEANING
    print(f"Loading data from : {DATA_PATH}...")
    try:
        # Load the data
        df = pd.read_csv(DATA_PATH)

        # Apply the discovery scale to velocity
        df['v_norm'] = df['v'] * 1e5
    except FileNotFoundError:
        print("Error: CV file not found. Check your DATA_PATH.")
        return

    # We exclude 'Source' so that AI finds a universal law.
    # We use .values to provide raw numpy arrays to the Julia engine.
    #X = df[['v_norm', 'z_norm', 'dzdt_norm']].values
    #y = df['F'].values
    X = df[['v_norm', 'z_norm']].values
    y = df['dzdt_norm'].values

    print(f"Dataset loaded. Size: {X.shape[0]} rows.")
    #print(f"Features: v, z_norm, dzdt_norm | Target: F")
    print(f"Features: v_norm, z_norm | Target: dzdt_norm")

    # --- PYSR REGRESSION CONFIGURATION
    model = PySRRegressor(
        niterations=1000,         # Low number for the first test run
        binary_operators=["+", "-", "*", "/"],
        unary_operators=[
            #"exp",              # Essential for Stribeck Effect
            "abs",              # Essential for Symmetry/Direction
            "square",           # Helps with the (v/vs)^2 term
        ],
        # We penalize complex equations to keep them lightweight.
        maxsize=30,             # Max tokens in the formula (admittable complexity)
        #complexity_of_operators={"exp": 3, "abs": 2},   # Make exp more expensive
        complexity_of_operators={"abs": 2},   # Exclude exp

        model_selection="best", # Automatically pick the best accuracy/complexity balance
        batching=True,          # Required for 200 000 rows to avoid RAM lag
        batch_size=1024,        # Check 1024 rows at a time

        procs=4,                # Use 4 CPU cores
        multithreading=True,
        timeout_in_seconds=600  # Stop after 10 minuites for this first test
    )

    # --- START THE EVOLUTION ---
    print("\n --- Starting PySR Evolution ---")
    print("PySR will now search for an algebraic formula F = f(v, z, dzdt)")

    model.fit(X, y)

    # --- OUTPUT RESULTS ---
    print("\n --- Discovery Complete ---")
    print("Best Equation Found:")
    print(model.get_best())

    # Save the model
    with open(os.path.join(MODEL_DIR, f"{model_id}.pkl"), "wb") as f:
        pickle.dump(model, f)

    # Save the configuration details
    save_model_config(model, model_id)

    print(f"\nAll files for {model_id} saved successfully.")

if __name__ == "__main__":
    main()