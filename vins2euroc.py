#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Author: Zhengzhe Xu
# Email: xuzhengzhe810@gmail.com

"""
This Python script is designed to convert CSV files output by VINS-Fusion into
EuRoC trajectory format files. It provides a graphical user interface (GUI) to
select a CSV file and save the converted file in the specified EuRoC format.
"""

import tkinter as tk
from tkinter import filedialog
import pandas as pd

def select_file():
    # Open file selection dialog
    filepath = filedialog.askopenfilename(
        title="Select CSV File",
        filetypes=(("CSV files", "*.csv"), ("All files", "*.*"))
    )
    if filepath:
        convert_to_euroc(filepath)

def convert_to_euroc(filepath):
    # Read CSV file
    data = pd.read_csv(filepath, header=None)
    
    # Ensure data only has eight columns
    if data.shape[1] > 8:
        data = data.iloc[:, :8]
    
    # Remove extraneous whitespace and commas
    data = data.applymap(lambda x: x.strip() if isinstance(x, str) else x)
    
    # Generate new filename
    new_filename = filepath.replace(".csv", "_euroc.csv")
    
    # Save new file
    data.to_csv(new_filename, index=False, header=False)
    label.config(text=f"File saved as: {new_filename}")

# Create main window
root = tk.Tk()
root.title("CSV to EuRoC Converter")

# Add button and label
tk.Button(root, text="Select CSV File", command=select_file).pack(pady=20)
label = tk.Label(root, text="")
label.pack(pady=20)

# Run GUI
root.mainloop()
