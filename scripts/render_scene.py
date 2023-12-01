import sys, os
import bpy
from blender_funcs import *
import numpy as np
from scipy.spatial.transform import Rotation


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
    camera.location = (2, 2, 2)

    center_point = create_point((0, 0, 0), (1, 0, 0, 0.5), 0.05)
    look_at_object(center_point)

    # CANNOT OVERWRITE FILES
    bpy.context.scene.render.filepath = test_png_path
    bpy.context.scene.render.resolution_x = 1600
    bpy.context.scene.render.resolution_y = 1600
    bpy.ops.render.render(write_still=True)


if __name__ == "__main__":
    
    render_env(sys.argv[1])