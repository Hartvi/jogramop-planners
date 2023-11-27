from blender_funcs import *
import numpy as np
import sys, os

import math


def rotation_matrix_to_euler_angles(R):
    """
    Convert a rotation matrix to Euler angles in XYZ order.
    """
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else:
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    return np.array([x, y, z])


class RobotSegment:
    def __init__(self, path, R, t):
        self.path = path
        self.R = R
        self.t = t

    def __repr__(self):
        return f"RobotSegment(path='{self.path}', R={self.R}, t={self.t})"


def load_robot_segments_from_csv(filename: str) -> list[RobotSegment]:
    segments = []
    with open(filename, 'r') as file:
        lines = file.readlines()

    i = 0
    while i < len(lines):
        if lines[i].strip() == 'file,':
            path = lines[i + 1].strip()
            i += 3  # Skip to R matrix
            R = np.array([list(map(float, line.strip().split(','))) for line in lines[i:i + 3]])
            i += 4  # Skip to t vector
            t = np.array(list(map(float, lines[i].strip().split(','))))
            segments.append(RobotSegment(path, R, t))
            i += 2  # Move to next segment
        i += 1

    return segments


def switchYZ_vectors(vectors: np.ndarray):
    vectors[1], vectors[2] = vectors[2], vectors[1]


def switchYZ_point(point: np.ndarray):
    return point[0], point[2], point[1]


def main(args: list[str]):
    start_scene()

    path_file = args[1]
    segments = load_robot_segments_from_csv(path_file)

    for segment in segments:
        import_obj(segment.path)
        o = get_newest_object()
        # o.location = switchYZ_point(segment.t)
        
        o.location = segment.t
        o.rotation_euler = rotation_matrix_to_euler_angles(segment.R)

    camera = bpy.data.objects["Camera"]
    camera.location = (3, -3, 3)

    center_point = create_point((0, 0, 0), (1, 1, 1, 1), size=0.05)
    look_at_object(center_point)
    
    test_png_path = "testpng.png"

    # CANNOT OVERWRITE FILES
    bpy.context.scene.render.filepath = test_png_path
    bpy.context.scene.render.resolution_x = 1600
    bpy.context.scene.render.resolution_y = 1600
    bpy.ops.render.render(write_still=True)
        

if __name__ == "__main__":
    main(sys.argv)
