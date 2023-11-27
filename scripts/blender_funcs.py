import bpy
import os
import sys
import math
import csv
import time

# create a scene
# scene = bpy.data.scenes.new("Scene")
# camera_data = bpy.data.cameras.new("Camera")


# constr = camera.constraints.new(type="TRACK_TO")
"""
 bpy.data.objects["Camera"].constraints.new("LOL")
Traceback (most recent call last):
  File "/usr/lib/python3.8/code.py", line 90, in runcode
    exec(code, self.locals)
  File "<blender_console>", line 1, in <module>
TypeError: ObjectConstraints.new(): error with argument 1, "type" -  enum "LOL" not found in (, 'CAMERA_SOLVER', 'FOLLOW_TRACK', 'OBJECT_SOLVER', 'COPY_LOCATION', 'COPY_ROTATION', 'COPY_SCALE', 'COPY_TRANSFORMS', 'LIMIT_DISTANCE', 'LIMIT_LOCATION', 'LIMIT_ROTATION', 'LIMIT_SCALE', 'MAINTAIN_VOLUME', 'TRANSFORM', 'TRANSFORM_CACHE', 'CLAMP_TO', 'DAMPED_TRACK', 'IK', 'LOCKED_TRACK', 'SPLINE_IK', 'STRETCH_TO', 'TRACK_TO', 'ACTION', 'ARMATURE', 'CHILD_OF', 'FLOOR', 'FOLLOW_PATH', 'PIVOT', 'SHRINKWRAP')
"""


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


if __name__ == "__main__":

    test_png_path = "/home/hartvi/Pictures/imgtesttt.png"
    if os.path.exists(test_png_path):
        os.remove(test_png_path)

    start_scene()
    out_path = "/home/hartvi/Documents/CVUT/diploma_thesis/load_obj_test/out.txt"
    with open(out_path, newline='') as csvfile:  # blender only
        #    with open(sys.argv[1], newline='') as csvfile:
        reader = csv.reader(csvfile, delimiter=',', quotechar='|')

        reader_rows = [row for row in reader]
        # file1 = reader_rows[0][1]
        # obj1_name = import_obj(file1)

        file2 = reader_rows[0][3]
        import_obj(os.path.join(os.path.dirname(
            out_path), file2.replace(" ", "")))
        obj2 = get_newest_object()
#        obj2.location = (2, 0, 0)

        # print("obj1:", obj1_name, "obj2:", obj2_name)
        for row in reader_rows:
            if row[0] == "init":
                center_point = create_point(
                    list(map(float, row[1:4])), (1, 1, 0, 1), size=0.5)
            if row[0] == "goal":
                center_point = create_point(
                    list(map(float, row[1:4])), (0, 0, 1, 1), size=0.5)
            if row[0] == "rand_only":
                rand_collision_coords = list(map(float, row[1:4]))
                create_point(rand_collision_coords, (1, 0, 0, 1))
            if row[0] == "new":
                no_collision_coords = list(map(float, row[1:4]))
                create_point(no_collision_coords, (0, 1, 0, 1))
                previous_coords = list(map(float, row[5:]))
                cylinder_between(*no_collision_coords, *previous_coords, 0.1)
        # for row in reader:
        #     print(', '.join(row))
    # print("reader.line_num", reader.line_num)

    camera = bpy.data.objects["Camera"]
    camera.location = (20, -20, 20)

    center_point = create_point((0, 0, 0), (1, 1, 1, 1))
    look_at_object(center_point)

    # CANNOT OVERWRITE FILES
    bpy.context.scene.render.filepath = test_png_path
    bpy.context.scene.render.resolution_x = 1600
    bpy.context.scene.render.resolution_y = 1600
    bpy.ops.render.render(write_still=True)
