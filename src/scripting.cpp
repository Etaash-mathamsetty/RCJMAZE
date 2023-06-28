#include "scripting.h"
#include "globals.h"
#include <iostream>
#include <fstream>
#include <sstream>
using namespace boost;

std::map<std::string, std::vector<float>> data_values;

namespace Bridge
{
    // could expose later
    void set_data_value(const std::string& key, python::list value)
    {
        data_values[key].clear();
        for(int i = 0; i < len(value); i++)
        {
            data_values[key].push_back(python::extract<float>(value[i]));
        }
    }

    void remove_data_value(const std::string& key)
    {
        if(data_values.count(key) != 0)
            data_values.erase(key);
    }

    std::optional<std::vector<float>> get_data_value(const std::string& key)
    {
        if(data_values.count(key) == 0)
            return {};
        return data_values[key];
    }
}

static bool f_is_simulation()
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

std::map<std::string, std::string> file_data;

// Execute the python script
void PythonScript::Exec(std::string file)
{
    try
    {
        //works around a weird bug with reading files over and over with python::exec_file
        if(file_data.count(file) == 0)
        {
            std::ifstream t(file);
            std::stringstream buffer;
            buffer << t.rdbuf();
            file_data[file] = buffer.str();
        }
        python::exec(file_data[file].c_str(), main_namespace, main_namespace);
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
