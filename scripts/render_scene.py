import sys, os
sys.path.append("/usr/local/lib/python3.10/site-packages")
import bpy
import numpy as np
from scipy.spatial.transform import Rotation
import math
import csv
import time


def cylinder_between(x1, y1, z1, x2, y2, z2, r, mat=None, reuse_cyl=None):
    """
    https://blender.stackexchange.com/questions/5898/how-can-i-create-a-cylinder-linking-two-points-with-python

    reuse_cyl : cylinder object to reuse (instead of creating a new one)
    """
    dx = x2 - x1
    dy = y2 - y1
    dz = z2 - z1
    dist = math.sqrt(dx**2 + dy**2 + dz**2)

    if dist == 0:
        return

    phi = math.atan2(dy, dx)
    theta = math.acos(dz / dist)

    if reuse_cyl is None:
        bpy.ops.mesh.primitive_cylinder_add(
            vertices=8, radius=r, depth=1, location=(dx / 2 + x1, dy / 2 + y1, dz / 2 + z1)
        )

        cyl = bpy.context.object
        if mat is not None:
            cyl.data.materials.append(mat)

    else:
        cyl = reuse_cyl
        cyl.location = (dx / 2 + x1, dy / 2 + y1, dz / 2 + z1)

    cyl.rotation_euler[1] = theta
    cyl.rotation_euler[2] = phi
    cyl.scale = (1.0, 1.0, dist)

    return cyl


def look_at_object(obj_name="Cube"):
    track_to_constraint = bpy.data.objects["Camera"].constraints.new(
        "TRACK_TO")
    track_to_constraint.target = bpy.data.objects[obj_name]
    track_to_constraint.up_axis = 'UP_Y'
    track_to_constraint.track_axis = "TRACK_NEGATIVE_Z"


def save_still(file_path="/home/hartvi/Pictures/img.png"):
    bpy.context.scene.render.filepath = file_path
    bpy.context.scene.render.resolution_x = 800
    bpy.context.scene.render.resolution_y = 600
    bpy.ops.render.render(write_still=True)


def add_and_set_color(obj, color, mat=None):
    # Add new material
    if mat is None:
        mat = bpy.data.materials.new(name=f"Material")
    obj.data.materials.append(mat)

    # Enable "Use nodes" and add Vertex Color Node
    mat.use_nodes = True
    nodes = mat.node_tree.nodes

    # Set the color and alpha parameters
    bsdf = nodes["Principled BSDF"]
    bsdf.inputs[0].default_value = color  # color
    bsdf.inputs[18].default_value = color[3]  # alpha


def create_point(location, color, size=0.2, mat=None) -> str:
    """Create a point and return its name"""
    # Create uv_sphere at `location` and assing a new material with `color` to it
    bpy.ops.mesh.primitive_uv_sphere_add(location=location, radius=size)
    obj = bpy.context.active_object
    obj.location = location
    add_and_set_color(obj, color, mat)
    return obj.name_full


def start_scene():
    set_creation_time(bpy.data)
    try:
        bpy.data.objects.remove(bpy.data.objects["Cube"])
    except:
        pass


def set_creation_time(scene):
    for obj in scene.objects:
        if "creation_time" not in obj:
            obj["creation_time"] = time.time()


def import_obj(filepath):
    # blender importing seems to be weird.
    # It switches y and z and shift the third object by 2 in the x axis
    bpy.ops.import_scene.obj(filepath=filepath, axis_forward='Y', axis_up='Z')
    set_creation_time(bpy.data)


def get_newest_object():
    newest_object = None
    newest_creation_time = 0

    # Iterate through all objects in the scene
    for obj in bpy.data.objects:
        # Check if the object is a mesh (or adjust based on the object type you're interested in)
        creation_time = obj["creation_time"]

        # Compare the creation time to find the newest object
        if creation_time > newest_creation_time:
            newest_object = obj
            newest_creation_time = creation_time

    return newest_object



class RenderItem:
    file = 0
    rotation = 1
    translation = 2


def get_render_item(render_item_id):
    if render_item_id == 0:
        return ""
    if render_item_id == 1:
        return np.zeros((3, 3))
    if render_item_id == 2:
        return np.zeros((3, ))


def render_env(path_to_file):

    start_scene()
    
    # bbb = create_point((3,0,0), (1, 0, 1, 1), size=0.1)
    with open(path_to_file, "r") as f:
        lines = f.read().split("\n")
        load_item: RenderItem = None
        k = 0
        while k < len(lines):

            if "file" in lines[k]:
                k += 1
                import_obj(lines[k])
                newest_object = get_newest_object()

                k += 1
                # this is basically a 2x OR operation
                for _ in range(2):
                    if "R" in lines[k]:
                        k += 1
                        R = np.zeros((3,3))
                        for i in range(3):
                            numbers = lines[k].split(",")
                            while "" in numbers:
                                numbers.remove("")
                            values = list(map(float, numbers))
                            for j in range(3):
                                R[i,j] = values[j]
                            
                            k += 1
                        
                        rotation: Rotation = Rotation.from_matrix(R)
                        eulers = rotation.as_euler("xyz", False)
                        for i in range(3):
                            newest_object.rotation_euler[i] = eulers[i]

                        
                    if "t" in lines[k] and len(lines[k]) < 2 or "t," in lines[k] and len(lines[k]) < 3:
                        k += 1
                        t = np.zeros((3, ))
                        numbers = lines[k].split(",")
                        while "" in numbers:
                            numbers.remove("")
                        values = list(map(float, numbers))
                        for j in range(3):
                            t[j] = values[j]
                        
                        k += 1

                        for i in range(3):
                            newest_object.location[i] = t[i]

            k += 1

    test_png_path = "/home/hartvi/Pictures/env_test.png"
    if os.path.exists(test_png_path):
        os.remove(test_png_path)

    camera = bpy.data.objects["Camera"]
    camera.location = (-2, 2, 2)

    center_point = create_point((0, 0, 0), (1, 0, 0, 0.5), 0.01)
    look_at_object(center_point)

    # CANNOT OVERWRITE FILES
    bpy.context.scene.render.filepath = test_png_path
    bpy.context.scene.render.resolution_x = 1600
    bpy.context.scene.render.resolution_y = 1600
    bpy.ops.render.render(write_still=True)


if __name__ == "__main__":
    test_path = "/home/hartvi/Documents/CVUT/diploma_thesis/burs_of_free_space/build/test_file.txt"
    render_env(test_path)