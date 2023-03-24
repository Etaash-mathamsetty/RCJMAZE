#ifndef SCRIPTING_H_INCLUDED
#define SCRIPTING_H_INCLUDED

#include <boost/python.hpp>
#include <map>
#include <mutex>
#include <optional>
#include <iostream>

class PythonScript
{
public:
    static void initPython();

    static void Exec(std::string);

    template<typename ReturnType, typename... Args>
    static std::optional<ReturnType> CallPythonFunction(std::string function, Args... args)
    {
        using namespace boost;
        if(PyObject_HasAttrString(main_module.ptr(), function.c_str()))
        {
            try
            {
                return python::extract<ReturnType>(main_module.attr(function.c_str())(args...));
            }
            catch(python::error_already_set const &)
            {
                PyErr_Print();
                return {};
            }
        }
        else
        {
            std::cerr << "ERR: Function " << function << " not found!" << std::endl;
            return {};
        }
    }
private: 
    static inline boost::python::object main_module;
    static inline boost::python::object main_namespace;
};

namespace Bridge
{
    std::optional<std::vector<float>> get_data_value(const std::string& key);

    void remove_data_value(const std::string& key);
}

#endif // SCRIPTING_H_INCLUDED
