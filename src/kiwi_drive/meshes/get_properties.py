import numpy as np
from stl import mesh

# Load the STL file
your_mesh = mesh.Mesh.from_file('omniwheelorigin_scaled.stl')

# Compute the inertia properties
volume, cog, inertia = your_mesh.get_mass_properties()

# Print the results
print("Volume: {0}".format(volume))
# Define the density of mild steel (in kg/m³)
density = 7900

# Calculate the mass
mass = density * volume

# Print the mass
print("Mass: {0} kg".format(mass))
print("Center of Gravity (COG): {0}".format(cog))
print("Inertia Tensor:")
print(inertia)

