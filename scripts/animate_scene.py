import pathlib
import argparse
import time
import math
from scipy.spatial.transform import Rotation
import numpy as np
import bpy
from datetime import datetime
import os
import sys
sys.path.append("/usr/local/lib/python3.10/site-packages")

FPS = 24
WIDTH = 1920
HEIGHT = 1080
FRAME_INCREMENT = 1
FILE_ARG = "--test_file"
TARGET_DIR_ARG = "--target_dir"
CAMX_ARG = "--camx"
CAMY_ARG = "--camy"
CAMZ_ARG = "--camz"
GRASP_ARG = "--grasp_file"
TREE_ARG = "--tree_file"


scene = bpy.context.scene
collection = bpy.context.collection


def float_to_colour(val):
    fill_speed = 1.0 / 3.0
    if val < fill_speed:
        return (0, 0, val / fill_speed, 1)
    elif val < fill_speed * 2:
        colval = (val - fill_speed) / fill_speed
        return (0, colval, 1 - colval, 1)
    else:
        colval = (val - 2 * fill_speed) / fill_speed
        return (colval, 1 - colval, 0, 1)
    # fill_speed = 1.0/3.0
    # fill_speed = 1.0/3.0
    # return (min())

    increment = 0.25  # 1/8
    float_increment = (val/increment)
    int_increment = int(round(float_increment))
    modulus_val = val - int_increment*increment
    if int_increment == 0:
        return (0, 0, modulus_val, 1)
    elif int_increment == 1:
        return (0, modulus_val, modulus_val, 1)
    elif int_increment == 2:
        return (0, modulus_val, 1, 1)
    elif int_increment == 3:
        return (modulus_val, modulus_val, 1, 1)
    else:
        return (modulus_val, 1, 1, 1)


def load_csv_floats(csv_file):
    ret = list()
    with open(csv_file, "r") as f:
        lines = f.read().split("\n")
        for line in lines:
            numbers = line.split(",")
            while "" in numbers:
                numbers.remove("")
            values = list(map(float, numbers))
            ret.append(values)
    return ret


def vis_points(point_positions, max_frame=0):
    # print("MAX FRAME", max_frame)
    # exit(1)
    n = len(point_positions)
    max_val = float(n-1)
    for i in range(n):
        position = tuple(point_positions[i])
        fraction = float(i) / max_val
        col = float_to_colour(fraction)
        # print('fraction:', fraction, "colour:", col)
        current_object = create_point(position, col, 0.015)
        # print("position: ", position)
        newest_object = bpy.context.object

        current_frame = i * max_frame / max_val
        newest_object.hide_viewport = True
        newest_object.hide_render = True
        newest_object.keyframe_insert(
            data_path="hide_viewport", frame=current_frame - 1)
        newest_object.keyframe_insert(
            data_path="hide_render", frame=current_frame - 1)
        newest_object.hide_viewport = False
        newest_object.hide_render = False
        newest_object.keyframe_insert(
            data_path="hide_viewport", frame=current_frame)
        newest_object.keyframe_insert(
            data_path="hide_render", frame=current_frame)

        newest_object.keyframe_insert(
            data_path="location", frame=current_frame)


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
    # Disable global shadows in Eevee
    # bpy.context.scene.eevee.use_shadow = False

    # Disable shadows for each light source in the scene
    for light in bpy.data.lights:
        light.use_shadow = False

    # For Cycles, to disable a light casting shadows
    for light in bpy.data.objects:
        if light.type == 'LIGHT':
            try:
                light.cycles_visibility.shadow = False
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


"""
        quad_vel.location = [px, py, pz]
        quad_vel.rotation_euler = rotation
        quad_vel.scale = scale

        quad_vel.keyframe_insert(data_path="location", frame=frame)
        quad_vel.keyframe_insert(data_path="rotation_euler", frame=frame)
        quad_vel.keyframe_insert(data_path="scale", frame=frame)
"""


class ObjectMode:
    none = -1
    robot = 0
    obstacle = 1


def render_env(path_to_file: str, extra_points_file: str = None, tree_points_file: str = None, target_dir: str = "", cam_pos=(-1, 3, 3)):

    start_scene()

    if extra_points_file and not extra_points_file == "None" and not extra_points_file.isspace() and not extra_points_file == "":
        with open(extra_points_file, "r") as f:
            lines = f.read().split("\n")
            for k in range(len(lines)):
                T = np.zeros((4, 4))
                numbers = lines[k].split(",")

                try:
                    values = list(map(float, numbers))
                    for i in range(16):
                        T[i//4, i % 4] = values[i]

                    position = T[:3, 3]
                    current_object = create_point(
                        position, (1, 0, 0, 1), 0.015)
                    # print("position: ", position)
                    newest_object = bpy.context.object
                    newest_object.keyframe_insert(
                        data_path="location", frame=0)
                except:
                    print("invalid data:", numbers)
                    pass

    print(f"PATH TO FILE: {path_to_file}")
    with open(path_to_file, "r") as f:
        lines = f.read().split("\n")
        k = 0
        built_models = dict()

        frame = -1
        # changes the speed at which it animates. Higher = slower

        while k < len(lines):
            # currently displaying robot segments: THEY ARE UNIQUE, SO NO ID NEEDED
            object_mode = ObjectMode.none
            model_id = None

            # line: `robot,segment_id``
            if "robot" in lines[k]:

                # for frame counting: when it changes to robot base segment, increment frame since it moved
                line_split = lines[k].split(",")

                if len(line_split) > 1:
                    segment_num = lines[k].split(",")[1]

                    if segment_num == "0":
                        frame += FRAME_INCREMENT

                object_mode = ObjectMode.robot
                k += 1

            elif "obstacle" in lines[k]:
                # second element in the line: obstacle,id
                model_id = lines[k].split(",")[1]
                object_mode = ObjectMode.obstacle
                k += 1

            if "file" in lines[k]:
                k += 1
                model_name = lines[k]

                if object_mode == ObjectMode.obstacle:
                    ...  # handled above
                elif object_mode == ObjectMode.robot:
                    model_id = model_name

                if not model_id in built_models:
                    import_obj(model_name)
                    # bpy.context.object
                    newest_object = get_newest_object()
                    built_models[model_id] = newest_object

                current_object = built_models[model_id]
                k += 1
                # this is basically a 2x OR operation
                for _ in range(2):
                    if "R" in lines[k]:
                        k += 1
                        T = np.zeros((3, 3))
                        for i in range(3):
                            numbers = lines[k].split(",")
                            while "" in numbers:
                                numbers.remove("")
                            values = list(map(float, numbers))
                            for j in range(3):
                                T[i, j] = values[j]

                            k += 1

                        rotation: Rotation = Rotation.from_matrix(T)
                        euler = rotation.as_euler('xyz', False)

                        for i in range(3):
                            current_object.rotation_euler[i] = euler[i]

                        current_object.keyframe_insert(
                            data_path="rotation_euler", frame=frame)

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
                            current_object.location[i] = t[i]

                        # insert frame of current rotation
                        current_object.keyframe_insert(
                            data_path="location", frame=frame)
            k += 1

    if tree_points_file and not tree_points_file == "None" and not tree_points_file.isspace() and not tree_points_file == "":
        try:
            tree_points = load_csv_floats(tree_points_file)
            vis_points(tree_points, frame)
            print("VIS TREE")
        except:
            print("NO TREE FILE", tree_points_file)

    camera = bpy.data.objects["Camera"]
    camera.location = cam_pos

    center_point = create_point((0, 0, 0.5), (1, 0, 0, 0.5), 0.001)
    look_at_object(center_point)

    # CANNOT OVERWRITE FILES
    render_animation(0, frame + FRAME_INCREMENT,
                     FRAME_INCREMENT, path_to_file, target_dir)


def render_animation(frame_start: int, frame_end: int, frame_step: int, path_to_src_file: str = None, target_dir: str = None):
    # Render animation
    scene = bpy.context.scene
    scene.render.engine = "BLENDER_EEVEE"

    scene.render.resolution_x = WIDTH
    scene.render.resolution_y = HEIGHT

    scene.frame_start = frame_start
    scene.frame_end = frame_end
    scene.frame_step = frame_step

    scene.render.image_settings.file_format = "FFMPEG"
    scene.render.ffmpeg.format = "MPEG4"
    scene.render.ffmpeg.codec = "H264"
    scene.render.ffmpeg.constant_rate_factor = "MEDIUM"

    timestamp = time.time()
    date_time = datetime.fromtimestamp(timestamp)
    str_date_time = date_time.strftime("%d_%m_%Y_%H_%M_%S")

    # the file name that will be save
    output_file_name = f"vid-{str_date_time}.mp4"
    if path_to_src_file is not None:
        tgt_name = pathlib.Path(os.path.basename(path_to_src_file)).stem
        output_file_name = f"{tgt_name}.mp4"

    output_dir_name = f"{os.getcwd()}/"
    # the directory where to save the file
    if target_dir is not None:
        output_file_name = target_dir

    print(f"OUTPUT DIR: {output_dir_name}")
    print(f"OUTPUT FILE: {output_file_name}")
    scene.render.filepath = os.path.join(output_dir_name, output_file_name)

    scene.render.use_overwrite = True

    scene = bpy.context.scene
    scene.render.fps = FPS
    bpy.ops.render.render(animation=True)


# if __name__ == "__main__":
#     test_path = "/home/hartvi/Documents/CVUT/diploma_thesis/burs_of_free_space/lel.vis"
#     # grasps_file = "/home/hartvi/Documents/CVUT/diploma_thesis/burs_of_free_space/jogramop/scenarios/005/export/grasps.csv"
#     grasps_file = None
#     tree_file = None
#     camX = -3
#     camY = 2
#     camZ = 3
#     if len(sys.argv) > 1:
#         test_path = sys.argv[1]
#         print("try file", test_path)
#         camX = float(sys.argv[2])
#         camY = float(sys.argv[3])
#         camZ = float(sys.argv[4])
#         if len(sys.argv) > 5:
#             # grasps
#             grasps_file = sys.argv[5]
#             print("visualizing grasps", grasps_file)
#         if len(sys.argv) > 6:
#             tree_file = sys.argv[6]
#             print("visualizing tree from", tree_file)

#     render_env(test_path, grasps_file, tree_file)


def main():
    parser = argparse.ArgumentParser(
        description="Render environment with optional grasps and tree visualization.")

    parser.add_argument(FILE_ARG, type=str,
                        default=None, help="Path to the test file")
    parser.add_argument(TARGET_DIR_ARG, type=str,
                        default=None, help="Path to the directory where to save animations to")
    parser.add_argument(CAMX_ARG, type=float, default=-
                        3, help="Camera X position")
    parser.add_argument(CAMY_ARG, type=float, default=2,
                        help="Camera Y position")
    parser.add_argument(CAMZ_ARG, type=float, default=3,
                        help="Camera Z position")
    parser.add_argument(GRASP_ARG, type=str, default=None,
                        help="File for grasps visualization")
    parser.add_argument(TREE_ARG, type=str, required=False, default=None,
                        help="File for tree visualization")

    args = parser.parse_args()

    print(f"Animating file {args.test_file}")
    print(f"Camera position: X={args.camx}, Y={args.camy}, Z={args.camz}")

    # if args.grasp_file:
    # print(f"visualizing grasps {args.grasp_file}")

    # if args.tree_file:
    # print(f"visualizing tree from {args.tree_file}")

    render_env(args.test_file, extra_points_file=args.grasp_file, tree_points_file=args.tree_file,
               target_dir=args.target_dir, cam_pos=(args.camx, args.camy, args.camz))


if __name__ == "__main__":
    main()
