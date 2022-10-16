#include <boost/python.hpp>
#include <atomic>
#include <future>
#include <vector>
#include <algorithm>
#include <map>
#include <mutex>

class PythonScript
{
public:

    boost::python::object main;
    boost::python::object global;
    std::string file;

    PythonScript(std::string file);

    void Exec();

    ~PythonScript();

};

namespace Bridge
{
    std::vector<float> get_data_value(const std::string key);

    void send_serial_command(PythonScript& script, std::string command);
}