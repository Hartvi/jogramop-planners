from distutils.core import setup, Extension

def main():
    up = "../"
    bur_related = "bur_related/"
    env_related = "env_related/"
    model_related = "model_related/"
    robot_related = "robot_related/"

    ext = Extension("Burs", 
        sources=[
            up + bur_related + "base_planner.cc",
            up + bur_related + "bur_tree.cc",
            up + bur_related + "urdf_planner.cc",
            up + env_related + "base_env.cc",
            up + env_related + "collision_env.cc",
            up + model_related + "pqp_load.cc",
            up + model_related + "rt_model.cc",
             "burs.cc",
            up + robot_related + "robot_base.cc",
            up + robot_related + "robot_collision.cc",
        ],
        # include_dirs=['/home/hartvi/Documents/CVUT/diploma_thesis/burs_of_free_space/include/', '/usr/include/eigen3/', '/usr/include/', '/usr/include/python3.8', '/home/hartvi/Documents/CVUT/diploma_thesis/PQP/include/'],  
        include_dirs=['/home/hartvi/Documents/CVUT/diploma_thesis/burs_of_free_space/include/', '/usr/include/eigen3/', '/usr/include/', '/usr/include/python3.9', '/home/hartvi/Documents/CVUT/diploma_thesis/PQP/include/'],  
        library_dirs=['/home/hartvi/Documents/CVUT/diploma_thesis/PQP/lib/'],  # Add any library directories if you're linking against external libraries
        libraries=['kdl_parser', 'urdf', 'lz4', 'PQP', 'flann'],  # Add the names of the libraries you're linking against (without 'lib' prefix and '.so' or '.dll' suffix)
        extra_compile_args=['-std=c++17',  '-Wall', '-Wextra', '-Wpedantic']  # Specify C++17 standard
    )

    setup(name="Burs",
          version="1.0.0",
          description="Python interface for path planner library",
          author="Jir Hartv",
          author_email="hartvjir@fel.cvut.cz",
          ext_modules=[ext])

if __name__ == "__main__":
    main()
