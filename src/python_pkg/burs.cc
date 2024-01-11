#define TINYOBJLOADER_IMPLEMENTATION
#include "model_related/tiny_obj_loader.h"
#include <python3.9/Python.h>
#include <fstream>

#include "bur_related/urdf_planner.h"
#include <Python.h>

/*
TODOs
- animation
- random obstacle generation
- connect to simulation environment
*/

Eigen::VectorXd PyListToVectorXd(PyObject *pyList)
{
    if (!PyList_Check(pyList))
    {
        throw std::invalid_argument("Input is not a list");
    }

    Py_ssize_t size = PyList_Size(pyList);
    Eigen::VectorXd vec(size);

    for (Py_ssize_t i = 0; i < size; ++i)
    {
        PyObject *item = PyList_GetItem(pyList, i); // Borrowed reference, no need to DECREF
        if (!PyNumber_Check(item))
        {
            throw std::invalid_argument("List item is not a number");
        }

        double value = PyFloat_AsDouble(item);
        if (PyErr_Occurred())
        {
            throw std::runtime_error("Error converting list item to double");
        }

        vec(i) = value;
    }

    return vec;
}

PyObject *VectorXdToPyList(const Eigen::VectorXd &vec)
{
    PyObject *pyList = PyList_New(vec.size());
    if (!pyList)
    {
        throw std::runtime_error("Failed to create new Python list");
    }

    for (int i = 0; i < vec.size(); ++i)
    {
        PyObject *num = PyFloat_FromDouble(vec(i));
        if (!num)
        {
            Py_DECREF(pyList);
            throw std::runtime_error("Failed to create Python float");
        }

        PyList_SetItem(pyList, i, num); // This steals a reference to num
    }

    return pyList;
}

PyObject *VectorOfVectorXdToPyList(const std::vector<Eigen::VectorXd> &vecList)
{
    PyObject *pyList = PyList_New(vecList.size());
    if (!pyList)
    {
        throw std::runtime_error("Failed to create new Python list for vector of VectorXd");
    }

    for (size_t i = 0; i < vecList.size(); ++i)
    {
        PyObject *vecPyList = VectorXdToPyList(vecList[i]);
        if (!vecPyList)
        {
            Py_DECREF(pyList);
            throw std::runtime_error("Failed to convert VectorXd to Python list");
        }

        // Note: PyList_SetItem steals a reference to vecPyList
        PyList_SetItem(pyList, i, vecPyList);
    }

    return pyList;
}

Eigen::MatrixXd PyListOfListsToMatrixXd(PyObject *pyList)
{
    if (!PyList_Check(pyList))
    {
        throw std::invalid_argument("Input is not a list");
    }

    Py_ssize_t num_rows = PyList_Size(pyList);
    if (num_rows == 0)
    {
        // Return an empty matrix if the list is empty
        return Eigen::MatrixXd(0, 0);
    }

    // Assume the number of columns is determined by the first row
    PyObject *firstRow = PyList_GetItem(pyList, 0); // Borrowed reference
    if (!PyList_Check(firstRow))
    {
        throw std::invalid_argument("First item is not a list");
    }

    Py_ssize_t num_cols = PyList_Size(firstRow);
    Eigen::MatrixXd mat(num_rows, num_cols);

    for (Py_ssize_t i = 0; i < num_rows; ++i)
    {
        PyObject *pyRow = PyList_GetItem(pyList, i); // Borrowed reference
        if (!PyList_Check(pyRow))
        {
            throw std::invalid_argument("Row is not a list");
        }

        if (PyList_Size(pyRow) != num_cols)
        {
            throw std::invalid_argument("Inconsistent row size in list of lists");
        }

        for (Py_ssize_t j = 0; j < num_cols; ++j)
        {
            PyObject *item = PyList_GetItem(pyRow, j);
            if (!PyFloat_Check(item) && !PyLong_Check(item))
            {
                throw std::invalid_argument("Non-numeric value in matrix");
            }

            double value = PyFloat_AsDouble(item);
            if (PyErr_Occurred())
            {
                throw std::runtime_error("Error converting list item to double");
            }

            mat(i, j) = value;
        }
    }

    return mat;
}

// A simple structure to hold URDFPlanner instance
typedef struct
{
    PyObject_HEAD
        Burs::URDFPlanner *planner;
} URDFPlannerObject;

// Wrapper for the constructor
static PyObject *URDFPlanner_new(PyTypeObject *type, PyObject *args, PyObject *kwds)
{
    URDFPlannerObject *self;

    self = (URDFPlannerObject *)type->tp_alloc(type, 0);
    if (self != NULL)
    {
        char *urdf_file;
        int max_iters;
        double d_crit, delta_q, epsilon_q;
        int num_spikes;

        // Parse the arguments from Python
        if (!PyArg_ParseTuple(args, "sidddi", &urdf_file, &max_iters, &d_crit, &delta_q, &epsilon_q, &num_spikes))
        {
            Py_DECREF(self);
            return NULL;
        }

        // Create the C++ object and store its pointer in the Python object
        self->planner = new Burs::URDFPlanner(std::string(urdf_file), max_iters, d_crit, delta_q, epsilon_q, num_spikes);
    }

    return (PyObject *)self;
}

static PyObject *URDFPlanner_GetNrOfJoints(URDFPlannerObject *self)
{
    if (!self->planner)
    {
        PyErr_SetString(PyExc_TypeError, "Invalid URDFPlanner object");
        return NULL;
    }
    unsigned int joints = self->planner->GetNrOfJoints();
    return PyLong_FromUnsignedLong(joints);
}

static PyObject *URDFPlanner_PlanPath(URDFPlannerObject *self, PyObject *args)
{
    PyObject *py_start, *py_goal;

    // Extract arguments from Python
    if (!PyArg_ParseTuple(args, "OO", &py_start, &py_goal))
    {
        return NULL;
    }

    try
    {
        Eigen::VectorXd start = PyListToVectorXd(py_start);
        Eigen::VectorXd goal = PyListToVectorXd(py_goal);

        std::optional<std::vector<Eigen::VectorXd>> path_opt = self->planner->PlanPath(start, goal);
        if (path_opt)
        {
            return VectorOfVectorXdToPyList(*path_opt);
        }
        else
        {
            // Return an empty list if no path is found
            return PyList_New(0);
        }
    }
    catch (const std::exception &e)
    {
        PyErr_SetString(PyExc_RuntimeError, e.what());
        return NULL;
    }
}

static PyObject *URDFPlanner_AddObstacle(URDFPlannerObject *self, PyObject *args)
{
    char *obstacle_file;
    PyObject *py_R, *py_t;

    // Extract arguments from Python
    if (!PyArg_ParseTuple(args, "sOO", &obstacle_file, &py_R, &py_t))
    {
        return NULL;
    }

    try
    {
        Eigen::Matrix3d R = PyListOfListsToMatrixXd(py_R);
        Eigen::Vector3d t = PyListToVectorXd(py_t);

        int result = self->planner->AddObstacle(std::string(obstacle_file), R, t);
        return PyLong_FromLong(result);
    }
    catch (const std::exception &e)
    {
        PyErr_SetString(PyExc_RuntimeError, e.what());
        return NULL;
    }
}

static PyObject *URDFPlanner_SetObstacleRotation(URDFPlannerObject *self, PyObject *args)
{
    int id;
    PyObject *py_R, *py_t;

    // Extract arguments from Python
    if (!PyArg_ParseTuple(args, "iOO", &id, &py_R, &py_t))
    {
        return NULL;
    }

    try
    {
        Eigen::Matrix3d R = PyListOfListsToMatrixXd(py_R);
        Eigen::Vector3d t = PyListToVectorXd(py_t);

        self->planner->SetObstacleRotation(id, R, t);
        Py_RETURN_NONE;
    }
    catch (const std::exception &e)
    {
        PyErr_SetString(PyExc_RuntimeError, e.what());
        return NULL;
    }
}

static PyObject *URDFPlanner_ToString(URDFPlannerObject *self, PyObject *args)
{
    PyObject *py_q_in, *py_obstacles;

    // Extract arguments from Python
    if (!PyArg_ParseTuple(args, "Op", &py_q_in, &py_obstacles))
    {
        return NULL;
    }

    try
    {
        Eigen::VectorXd q_in = PyListToVectorXd(py_q_in);
        bool obstacle_only = PyObject_IsTrue(py_obstacles);

        std::ostringstream res;

        res << self->planner->ToString(q_in, obstacle_only);

        return PyUnicode_FromString(res.str().c_str());
    }
    catch (const std::exception &e)
    {
        PyErr_SetString(PyExc_RuntimeError, e.what());
        return NULL;
    }
}

// static PyObject *URDFPlanner_StringifyPath(URDFPlannerObject *self, PyObject *args)
// {
//     PyObject *py_path;

//     // Extract arguments from Python
//     if (!PyArg_ParseTuple(args, "O", &py_path))
//     {
//         return NULL;
//     }

//     try
//     {
//         Eigen::VectorXd q_in = PyListToVectorXd(py_q_in);
//         std::ostringstream res;

//         res << self->planner->StringifyPath(q_in, true);
//         res << self->planner->ToString(q_in, false);
//         return PyUnicode_FromString(res.str().c_str());
//     }
//     catch (const std::exception &e)
//     {
//         PyErr_SetString(PyExc_RuntimeError, e.what());
//         return NULL;
//     }
// }

static PyObject *URDFPlanner_InterpolatePath(URDFPlannerObject *self, PyObject *args)
{
    PyObject *py_path;
    double threshold;

    // Extract arguments from Python
    if (!PyArg_ParseTuple(args, "Od", &py_path, &threshold))
    {
        return NULL;
    }

    try
    {
        if (!PyList_Check(py_path))
        {
            throw std::invalid_argument("Expected a list of lists");
        }

        std::vector<Eigen::VectorXd> path;
        Py_ssize_t num_elements = PyList_Size(py_path);
        for (Py_ssize_t i = 0; i < num_elements; ++i)
        {
            PyObject *py_vec = PyList_GetItem(py_path, i);
            path.push_back(PyListToVectorXd(py_vec));
        }

        std::vector<Eigen::VectorXd> result = self->planner->InterpolatePath(path, threshold);
        PyObject *py_result = VectorOfVectorXdToPyList(result);
        return py_result;
    }
    catch (const std::exception &e)
    {
        PyErr_SetString(PyExc_RuntimeError, e.what());
        return NULL;
    }
}

static PyMethodDef URDFPlanner_methods[] = {
    {"GetNrOfJoints", (PyCFunction)URDFPlanner_GetNrOfJoints, METH_NOARGS, "Get number of joints."},
    {"PlanPath", (PyCFunction)URDFPlanner_PlanPath, METH_VARARGS, "Plan a path from start to goal."},
    {"AddObstacle", (PyCFunction)URDFPlanner_AddObstacle, METH_VARARGS, "Add an obstacle with file path, rotation matrix, and translation vector."},
    {"SetObstacleRotation", (PyCFunction)URDFPlanner_SetObstacleRotation, METH_VARARGS, "Set the rotation and translation of an obstacle."},
    {"ToString", (PyCFunction)URDFPlanner_ToString, METH_VARARGS, "Get a string representation of the planner state for a given configuration."},
    {"InterpolatePath", (PyCFunction)URDFPlanner_InterpolatePath, METH_VARARGS, "Interpolate a given path with a specified threshold."},
    {NULL, NULL, 0, NULL} /* Sentinel */
};

static PyTypeObject URDFPlannerType = {
    PyVarObject_HEAD_INIT(NULL, 0)
        .tp_name = "Burs.URDFPlanner",
    .tp_basicsize = sizeof(URDFPlannerObject),
    .tp_flags = Py_TPFLAGS_DEFAULT,
    .tp_methods = URDFPlanner_methods,
    .tp_new = URDFPlanner_new,
};

static struct PyModuleDef module = {
    PyModuleDef_HEAD_INIT,
    "Burs", // Model maker And Collision Engine
    "Python interface for the Burs planning library",
    -1,
    NULL};

PyMODINIT_FUNC PyInit_Burs(void)
{
    PyObject *m;

    if (PyType_Ready(&URDFPlannerType) < 0)
        return NULL;

    m = PyModule_Create(&module);
    if (m == NULL)
        return NULL;

    Py_INCREF(&URDFPlannerType);
    if (PyModule_AddObject(m, "URDFPlanner", (PyObject *)&URDFPlannerType) < 0)
    {
        Py_DECREF(&URDFPlannerType);
        Py_DECREF(m);
        return NULL;
    }

    return m;
}
