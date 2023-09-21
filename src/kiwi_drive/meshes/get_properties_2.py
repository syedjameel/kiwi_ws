import trimesh

# Load the STL file
mesh = trimesh.load_mesh('kiwibasev4_scaled.stl')

# Compute the inertia tensor
inertia = mesh.moment_inertia

# Print the inertia tensor
print(inertia)

