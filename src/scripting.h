#include <boost/python.hpp>
#include <map>
#include <mutex>
#include <optional>
#include <iostream>

#ifndef _PY_SCRIPT
#define _PY_SCRIPT
class PythonScript
{
public:
    static void initPython();

    static void Exec(std::string);

    template<typename T, typename... Args>
    static std::optional<T> CallPythonFunction(std::string function, Args... args)
    {
        using namespace boost;
        if(PyObject_HasAttrString(main_module.ptr(), function.c_str()))
        {
            try
            {
                return python::extract<T>(main_module.attr(function.c_str())(args...));
            }
            catch(python::error_already_set const &)
            {
                PyErr_Print();
                return {};
            }
        }
        else
        {
            std::cout << "Function " << function << " not found" << std::endl;
            return {};
        }
    }
private: 
    static inline boost::python::object main_module;
    static inline boost::python::object main_namespace;
};

namespace Bridge
{
    std::vector<float> get_data_value(const std::string key);
}

#endif