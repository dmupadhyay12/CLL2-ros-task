#!/usr/bin/python3

# This plot takes the data from target_error.csv and plots both the euclidean and angular error 
# against the number of iterations of the control loop. It serves to show how the errors converge
# onto the desired limits as time goes on

import pandas as pd
import matplotlib.pyplot as plt

print("Starting generation of error plots")

# Load the CSV data
df = pd.read_csv('target_error.csv')

# Step 2: Plot the data
plt.figure(figsize=(10, 6))

# Plot Euclidean error
plt.plot(df['Iterations'], df['Euclidean Error'], label='Euclidean Error', color='blue', marker=None, linewidth=2)

# Plot Angular error
plt.plot(df['Iterations'], df['Angular Error'], label='Angular Error', color='red', marker=None, linewidth=2)

# Step 3: Customize the plot
plt.title('Euclidean and Angular Errors vs Iterations')
plt.xlabel('Iterations')
plt.ylabel('Error')
plt.legend()  # Show legend
plt.grid(True)  # Show grid for better readability

# Display the plot
plt.show()