#include <python3.8/Python.h>

#include "bur_related/urdf_planner.h"
#include <Python.h>

// Wrapper for the constructor
static PyObject *URDFPlanner_new(PyTypeObject *type, PyObject *args, PyObject *kwds)
{
    std::string urdf_file;
    int max_iters;
    double d_crit, delta_q, epsilon_q;
    int num_spikes;

    if (!PyArg_ParseTuple(args, "sidddi", &urdf_file, &max_iters, &d_crit, &delta_q, &epsilon_q, &num_spikes))
    {
        return NULL;
    }

    Burs::URDFPlanner *self = new Burs::URDFPlanner(urdf_file, max_iters, d_crit, delta_q, epsilon_q, num_spikes);

    return PyCapsule_New(self, "URDFPlanner", NULL);
}

// Wrapper for GetNrOfJoints
static PyObject *URDFPlanner_GetNrOfJoints(PyObject *self)
{
    Burs::URDFPlanner *planner = reinterpret_cast<Burs::URDFPlanner *>(PyCapsule_GetPointer(self, "URDFPlanner"));

    if (!planner)
    {
        PyErr_SetString(PyExc_TypeError, "Invalid URDFPlanner object");
        return NULL;
    }

    unsigned int joints = planner->GetNrOfJoints();
    return PyLong_FromUnsignedLong(joints);
}

static PyMethodDef URDFPlanner_methods[] = {
    {"GetNrOfJoints", (PyCFunction)URDFPlanner_GetNrOfJoints, METH_NOARGS, "Get number of joints."},
    // Add other methods here
    {NULL} /* Sentinel */
};

// static PyMethodDef Methods[] = {
//     {"begin_model", begin_model_wrap, METH_VARARGS, "Python interface for starting a RAPID model"},
//     {"add_to_model", add_to_model_wrap, METH_VARARGS, "Python interface for adding objects a RAPID model"},
//     {"end_model", end_model_wrap, METH_VARARGS, "Python interface for ending a RAPID model"},
//     {"delete_model", delete_model_wrap, METH_VARARGS, "Python interface for deleting the RAPID model"},
//     {"check_col", check_col_wrap, METH_VARARGS, "Python interface for checking collisions with RAPID"},
//     {NULL, NULL, 0, NULL}
// };

// static struct PyModuleDef module = {
//     PyModuleDef_HEAD_INIT,
//     "MACE", // Model maker And Collision Engine
//     "Python interface for the RAPID C library",
//     -1,
//     Methods
// };

// // WARNING!!! this has to have the name PyInit_<module_name>
// PyMODINIT_FUNC PyInit_MACE(void)
// {
//     return PyModule_Create(&module);
// }

// static PyTypeObject URDFPlannerType = {
//     PyVarObject_HEAD_INIT(NULL, 0)
//         .tp_name = "mymodule.URDFPlanner",
//     .tp_basicsize = sizeof(PyTypeObject),
//     .tp_itemsize = 0,
//     .tp_flags = Py_TPFLAGS_DEFAULT,
//     .tp_new = URDFPlanner_new,
//     .tp_methods = URDFPlanner_methods,
//     // ... additional type members ...
// };
