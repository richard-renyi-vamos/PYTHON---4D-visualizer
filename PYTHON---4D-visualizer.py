import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from tkinter import Tk, Label, Scale, Button, HORIZONTAL, DoubleVar

# Define vertices of a 4D hypercube
def generate_tesseract():
    vertices = []
    for i in range(16):
        vertex = [(i >> j) & 1 for j in range(4)]
        vertex = [2 * v - 1 for v in vertex]  # Map 0,1 to -1,1
        vertices.append(vertex)
    return np.array(vertices)

# Generate edges of the 4D hypercube
def generate_edges(vertices):
    edges = []
    for i, v1 in enumerate(vertices):
        for j, v2 in enumerate(vertices):
            if i < j and np.sum(np.abs(v1 - v2)) == 2:
                edges.append((i, j))
    return edges

# Project from 4D to 3D
def project_4D_to_3D(vertices, angle):
    rotation_matrix = np.array([
        [np.cos(angle), -np.sin(angle), 0, 0],
        [np.sin(angle), np.cos(angle), 0, 0],
        [0, 0, 1, 0]
    ])
    return vertices @ rotation_matrix.T

# Visualization function
def visualize_tesseract(angle):
    vertices = generate_tesseract()
    edges = generate_edges(vertices)
    projected_vertices = project_4D_to_3D(vertices, angle)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot vertices
    ax.scatter(projected_vertices[:, 0], projected_vertices[:, 1], projected_vertices[:, 2], c='b', s=50)
    
    # Plot edges
    for edge in edges:
        v1, v2 = edge
        line = np.array([projected_vertices[v1], projected_vertices[v2]])
        ax.plot(line[:, 0], line[:, 1], line[:, 2], c='r')
    
    ax.set_title(f"4D Hypercube (Angle: {np.degrees(angle):.2f}°)")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    plt.show()

# GUI function
def create_gui():
    def update_visualization():
        angle = np.radians(angle_var.get())
        visualize_tesseract(angle)

    # Create GUI window
    root = Tk()
    root.title("4D Hypercube Visualization Settings")

    # Angle slider
    angle_var = DoubleVar(value=30)  # Default angle in degrees
    Label(root, text="Rotation Angle (Degrees)").pack(pady=10)
    angle_slider = Scale(root, from_=0, to=360, orient=HORIZONTAL, variable=angle_var)
    angle_slider.pack(pady=10)

    # Visualize button
    Button(root, text="Visualize", command=update_visualization).pack(pady=20)

    # Run GUI loop
    root.mainloop()

# Run GUI
create_gui()
