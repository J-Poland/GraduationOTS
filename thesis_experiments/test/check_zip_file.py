import pandas as pd
import zipfile
import os
import io


# set file path and name
folder_path = r'C:\Users\jesse\Documents\Java\TrafficSimulation-workspace\traffic-sim\src\main\resources\full_level_runs_0\seed_0\run_0'
file_name = r'sequenceOutputData.csv'

zip_path = os.path.join(folder_path, file_name[:-4] + '.zip')

# Open the ZIP file
with zipfile.ZipFile(zip_path, 'r') as zip_ref:
    # Check if the file exists inside the ZIP
    if file_name not in zip_ref.namelist():
        raise FileNotFoundError(f"{file_name} not found in {zip_path}")

    # Open the file in the ZIP and check its size
    with zip_ref.open(file_name) as file:
        # Check if the file is empty
        if zip_ref.getinfo(file_name).file_size == 0:
            raise ValueError(f"{file_name} in {zip_path} is empty.")

        # Wrap the file in a text IO wrapper for reading as a CSV
        with io.TextIOWrapper(file) as text_file:
            # Now, read the CSV into a DataFrame
            df_output = pd.read_csv(text_file)

            print(len(df_output))

