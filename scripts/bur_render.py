from blender_funcs import *
import numpy as np
import sys, os


def switchYZ_vectors(vectors: np.ndarray):
    vectors[1], vectors[2] = vectors[2], vectors[1]


def switchYZ_point(point: np.ndarray):
    return point[0], point[2], point[1]


def main(args):
    path_file = args[1]

    start_scene()
    
    bbb = create_point((3,0,0), (1, 0, 1, 1), size=1)
    with open(path_file, "r") as f:
        file_contents = f.read()
        lines = file_contents.split("\n")

        old_coordinates = None
        for k, line in enumerate(lines):
            # line => list[float] of coordinates
            cpp_coordinates = list(map(float, line.split(",")[:-1]))
            
            if cpp_coordinates:
                blender_coordinates: tuple[float] = switchYZ_point(cpp_coordinates)
                
                print(blender_coordinates)

                if len(blender_coordinates) == 3:
                    center_point = create_point(blender_coordinates, (0, 0, 1, 1), size=0.2)

                    if old_coordinates is not None:
                        cylinder_between(*blender_coordinates, *old_coordinates, 0.1)

                    old_coordinates: list[float] = blender_coordinates


    camera = bpy.data.objects["Camera"]
    camera.location = (10, -10, 10)

    center_point = create_point((0, 0, 0), (1, 1, 1, 1))
    look_at_object(center_point)
    
    test_png_path = "testpng.png"

    # CANNOT OVERWRITE FILES
    bpy.context.scene.render.filepath = test_png_path
    bpy.context.scene.render.resolution_x = 1600
    bpy.context.scene.render.resolution_y = 1600
    bpy.ops.render.render(write_still=True)


if __name__ == "__main__":
    main(sys.argv)
