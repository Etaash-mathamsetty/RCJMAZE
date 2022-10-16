#include "scripting.h"
#include <iostream>
using namespace boost;

std::map<std::string, std::vector<float>> data_values;
std::mutex mutex;
std::unique_lock<std::mutex> lock = {mutex, std::defer_lock};

void set_data_value(const std::string key, python::list value)
{
    lock.lock();
    for(int i = 0; i < len(value); i++)
    {
        data_values[key].push_back(python::extract<float>(value[i]));
    }
    lock.unlock();
}

namespace Bridge
{
    std::vector<float> get_data_value(const std::string key)
    {
        lock.lock();
        std::vector<float> value = data_values[key];
        lock.unlock();
        return value;
    }

    void send_serial_command(PythonScript& script, std::string command)
    {
        script.main.attr("WriteSerialCommand")(command);
    }
}

BOOST_PYTHON_MODULE(Robot)
{
    using namespace boost::python;
    void(*SetDataValue)(const std::string, python::list) = &set_data_value;
    def("SetDataValue", SetDataValue);
}

// Get the python module ready
PythonScript::PythonScript(std::string file)
{
    PyImport_AppendInittab("Robot", PyInit_Robot);
    Py_Initialize();
    main = python::import("__main__");
    global = python::object(main.attr("__dict__"));
    this->file = file;
}

// Execute the python script
void PythonScript::Exec()
{
    try
    {
        python::exec_file(file.c_str(), global, global);
    }
    catch(python::error_already_set const&)
    {
        PyErr_Print();
    }
}

PythonScript::~PythonScript()
{
   // Py_Finalize();
}