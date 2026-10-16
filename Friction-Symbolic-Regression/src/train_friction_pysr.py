import pandas as pd
import numpy as np
from pysr import PySRRegressor
import os
import pickle
from  pathlib import Path
from get_date import get_formatted_date

# --- MODEL NAMING ---
date = get_formatted_date()
version = "v15"
explanation = "relax-penalty-division-to-1"
#target = "F"
target = "dzdt"
model_id = date + "_withz_" + version + "_" + target + "_" + explanation

# --- SETUP PATHS ---
# Update below file name when you use a new dataset
dataset_csv_name = "26-10-13_only-velocity_fix-simulink-variable-apply-problem_params-paper_modelsetting-usual.csv"
DATA_PATH = os.path.join("..", "..", "LuGre_Sim", "outputs", "26-10-13_only-velocity_fix-simulink-variable-apply-problem", dataset_csv_name)
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

    except FileNotFoundError:
        print("Error: CSV file not found. Check your DATA_PATH.")
        return

    # We exclude 'Source' so that AI finds a universal law.
    # We use .values to provide raw numpy arrays to the Julia engine.
    #X = df[['v_norm', 'z_norm', 'dzdt_norm']].values
    #y = df['F'].values
    #X = df[['v_norm', 'z_norm']].values
    #y = df['dzdt_norm'].values
    X = df[['v_norm', 'z_norm']].values
    y = (df['v_norm'] - df['dzdt_norm']).values

    print(f"Dataset loaded. Size: {X.shape[0]} rows.")
    #print(f"Features: v, z_norm, dzdt_norm | Target: F")
    #print(f"Features: v_norm, z_norm | Target: dzdt_norm")
    print(f"Features: v_norm, z_norm | Target: v_norm - dzdt_norm")

    # --- PYSR REGRESSION CONFIGURATION
    model = PySRRegressor(
        # --- Population and Iteration ---
        niterations=200,
        populations=20,
        population_size=50,

        # --- Allowed Operators ---
        binary_operators=["+", "-", "*", "/"],
        unary_operators=["abs"],

        # --- Operator Costs ---          
        complexity_of_operators={
            "+": 1,
            "-": 1,
            "*": 1,
            "/": 1,
            "abs": 1
        },

        # --- Structural Constrains ---
        maxsize=20,
        nested_constraints={
            "/": {"/": 0},
            "abs": {"abs": 0},
        },

        # --- Data Performance ---
        batching=True,
        batch_size=2000,

        # --- Loss and Selection ---
        # relative error way, see 10-02 note for the logic
        # by adding 1 to the denominator, we avoid division by zero (we have dzdt=0 poit at break-away)
        loss="loss(prediction, target) = ((prediction - target) / (abs(target) + 1.0))^2",
        parsimony=0.001,
        random_state=42        # For reproducibility
    )

    # --- START THE EVOLUTION ---
    print("\n --- Starting PySR Evolution ---")
    #print("PySR will now search for an algebraic formula F = f(v, z, dzdt)")

    model.fit(X, y)

    # --- OUTPUT RESULTS ---
    print("\n --- Discovery Complete ---")
    print("Best Equation Found:")
    print(model.sympy())

    # Save the model
    with open(os.path.join(MODEL_DIR, f"{model_id}.pkl"), "wb") as f:
        pickle.dump(model, f)

    # Save the configuration details
    save_model_config(model, model_id)

    print(f"\nAll files for {model_id} saved successfully.")

if __name__ == "__main__":
    main()