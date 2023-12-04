import sys
import Burs

# Check if the major and minor version of Python is 3.8
if sys.version_info.major == 3 and sys.version_info.minor == 8:
    # Your code here
    print("Running on Python 3.8.x")
else:
    # Exit the script if not running on Python 3.8
    print("This script requires Python 3.8.x")
    sys.exit(1)


def main(args):
    urdf_path = "/home/hartvi/Documents/CVUT/diploma_thesis/burs_of_free_space/jogramop/robots/franka_panda/mobile_panda.urdf"
    num_iters = 10
    delta_q = 0.5
    d_crit = 0.01
    epsilon_q = 0.05
    num_burs = 7
    p = Burs.URDFPlanner(urdf_path, num_iters, delta_q, d_crit, epsilon_q, num_burs)
    #
    start = [0.0839963 -0.0947054, -1.56511, -1.27556, -1.97629, -2.57375, -1.21169, 0.700806, -1.79374]
    goal = [-0.0424896 ,-0.0217785,2.12436,1.56464,2.19127,-0.574181,2.24271,3.55635,1.86062]

    path = p.PlanPath(start, goal)
    print("path: ", path)

if __name__=="__main__":
    main(sys.argv)