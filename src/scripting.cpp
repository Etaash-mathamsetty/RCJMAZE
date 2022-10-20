#include "scripting.h"
#include "globals.h"
#include <iostream>
using namespace boost;

std::map<std::string, std::vector<float>> data_values;

namespace Bridge
{
    // could expose later
    void set_data_value(const std::string key, python::list value)
    {
        for(int i = 0; i < len(value); i++)
        {
            data_values[key].push_back(python::extract<float>(value[i]));
        }
    }

    std::vector<float> get_data_value(const std::string key)
    {
        return data_values[key];
    }
}

bool f_is_simulation()
{
    return is_simulation;
}

BOOST_PYTHON_MODULE(Robot)
{
    using namespace boost::python;
    def("SetDataValue", &Bridge::set_data_value);
    def("isSimulation", &f_is_simulation);
}

void PythonScript::initPython()
{
    //one time python init
    PyImport_AppendInittab("Robot", PyInit_Robot);
    Py_Initialize();
    main_module = python::import("__main__");
    main_namespace = main_module.attr("__dict__");
}

// Execute the python script
void PythonScript::Exec(std::string file)
{
    try
    {
        python::exec_file(file.c_str(), main_namespace, main_namespace);
    }
    catch(python::error_already_set const&)
    {
        PyErr_Print();
    }
    catch(...)
    {
        std::cout << "Unknown exception" << std::endl;
    }
}
