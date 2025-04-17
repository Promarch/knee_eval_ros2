#%%
import numpy as np
import os
import yaml
from scipy.spatial.distance import pdist, squareform
from scipy.spatial.transform import Rotation as R

ref_points = {
    "sensor": [
        {"x":0.027, "y": 0.0, "z":0.025},
        {"x":-0.025, "y": 0.0, "z":0.025},
        {"x":0.0, "y": 0.0355, "z":0.0},
        {"x":0.0, "y": -0.0255, "z":0.0},
        {"x":0.0, "y": 0.0, "z":0.043}],
    "femur": [ # "main"
        {"x":0.0194, "y": 0.0, "z":0.015},
        {"x":-0.0176, "y": 0.0, "z":0.015},
        {"x":0.0, "y": -0.02, "z":0.0},
        {"x":0.0, "y": 0.018, "z":0.0},
        {"x":0.0, "y": 0.0, "z":0.0254}],
    "tibia": [ # "V3"
        {"x":0.0234, "y": 0.0, "z":0.020},
        {"x":-0.0176, "y": 0.0, "z":0.020},
        {"x":0.0, "y": -0.028, "z":0.0},
        {"x":0.0, "y": 0.018, "z":0.0},
        {"x":0.0, "y": 0.0, "z":0.0354}]
}

def kabsch(p, q):
    """Calculate the optimal rigid transformation matrix from Q -> P using Kabsch algorithm"""

    centroid_p = np.mean(p, axis=0)
    centroid_q = np.mean(q, axis=0)

    p_centered = p - centroid_p
    q_centered = q - centroid_q

    H = np.dot(p_centered.T, q_centered)

    U, vt = np.linalg.svd(H)

    R = np.dot(vt.T, U.T)

    if np.linalg.det(R) < 0:
        vt[-1, :] *= -1
        R = np.dot(vt.T, U.T)

    t = centroid_q - np.dot(centroid_p, R.T)

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t

    return T

def sort_points_relative(points1, points2):

    # Compute pairwise distances for each list
    distances_1 = squareform(pdist(points1))
    distances_2 = squareform(pdist(points2))
    # Sum the distances for each point
    sum_distances_1 = np.sum(distances_1, axis=1)
    sum_distances_2 = np.sum(distances_2, axis=1)
    # Get the sorted indices based on the sum of distances
    sorted_indices_1 = np.argsort(sum_distances_1)
    sorted_indices_2 = np.argsort(sum_distances_2)
    # Sort the points based on the computed indices
    sorted_points1 = points1[sorted_indices_1]
    sorted_points2 = points2[sorted_indices_2]

    return sorted_points1, sorted_points2

def convert_dict_list_to_point_array(dict_list):
    """Convert a list of dicts with x,y,z keys to a numpy array of points"""
    return np.array([[p['x'], p['y'], p['z']] for p in dict_list])

def convert_point_array_to_dict_list(point_array):
    """Convert a numpy array of points to a list of dicts with x,y,z keys"""
    return [{'x': float(p[0]), 'y': float(p[1]), 'z': float(p[2])} for p in point_array]

#%%
# Create rotation matrix
r = R.from_euler('z', 30, degrees=True)
print("Rotation matrix: \n", np.round(r.as_matrix(), 2))
# Rotate points
femur_marker = convert_dict_list_to_point_array(ref_points["femur"])
rotated_points = np.transpose(r.as_matrix() @ femur_marker.T)
mixed_points = rotated_points[[0,3,2,4,1],:]

sorted_orig, sorted_ref = sort_points_relative(femur_marker, mixed_points)
print("Sorted original points:\n", sorted_orig, "\nSorted reference points:\n", sorted_ref)
print("Original points:\n", femur_marker, "\nReference Points\n", rotated_points)

# %%
# Create YAML file
yaml_data = {}
yaml_data["original_points"] = convert_point_array_to_dict_list(femur_marker)
yaml_data["rotated_points"] = convert_point_array_to_dict_list(np.round(rotated_points,4))
# Write to YAML file
current_folder = os.path.dirname(os.path.abspath(__file__))
yaml_file = current_folder + "/points.yaml"
with open(yaml_file, 'w') as f:
    yaml.dump(yaml_data, f, default_flow_style=False, sort_keys=False)
