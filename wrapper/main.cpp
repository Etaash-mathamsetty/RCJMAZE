#include <iostream>
#include <filesystem>
#include <stdlib.h>
#include <unistd.h>
#include <sys/signal.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/wait.h>
//#define DEBUG_LOGIC
#ifndef DEBUG_LOGIC
#include <wiringPi.h>
#endif

namespace fs = std::filesystem;

const int pin = 7;


bool button_pressed()
{
#ifndef DEBUG_LOGIC
    const int thresh_count = 50;
    for(int i = 0; i < thresh_count; i++)
    {
        if(!digitalRead(pin))
            return false;
    }
#endif
    return true;
}

bool button_released()
{
#ifndef DEBUG_LOGIC
    const int thresh_count = 50;
    for(int i = 0; i < thresh_count; i++)
    {
        if(digitalRead(pin))
            return false;
    }
#endif
    return true;
}

bool has_child_exited(pid_t pid)
{
    int status = 0;
    waitpid(pid, &status, WNOHANG);

    if(WIFEXITED(status) && WEXITSTATUS(status) == 0)
    {
        return true;
    }

    return false;
}

//pass in arg to BFS path
int main(int argc, char **argv)
{
    std::string path_to_bfs = "";
    if(argc >= 2)
    {
        path_to_bfs = std::string(argv[1]);
        path_to_bfs = fs::canonical(fs::absolute(path_to_bfs)).c_str();
        std::cout << "executing: " << path_to_bfs << std::endl;
    }
    else
    {
        std::cout << "pass in path to BFS exec" << std::endl;
        return -1;
    }

#ifndef DEBUG_LOGIC
    wiringPiSetup();
    pinMode(pin, INPUT);
#endif

    fs::path parent_path = path_to_bfs;
    if(!parent_path.has_parent_path())
    {
        std::cout << "no parent path" << std::endl;
        return -1;
    }
    parent_path = parent_path.parent_path();
    std::cout << "parent path: " << parent_path.c_str() << std::endl;

    std::string parent_path_str = parent_path.c_str();

    pid_t child_pid = 0;

    while(true)
    {
        child_pid = fork();
        if(child_pid > 0)
        {
            std::cout << "parent process starting: " << getpid() << std::endl;
            while(!button_pressed())
            {
                if(has_child_exited(child_pid))
                {
                    std::cout << "child process exited (successfully)!" << std::endl;
                    goto natural_exit;
                }
            }
            while(!button_released())
            {
                if(has_child_exited(child_pid))
                {
                    std::cout << "child process exited (successfully)!" << std::endl;
                    goto natural_exit;
                }
            }
#ifdef DEBUG_LOGIC
            if(has_child_exited(child_pid))
            {
                std::cout << "child process exited (successfully)!" << std::endl;
                goto natural_exit;
            }
#endif
            std::cout << "killing child process: " << child_pid << std::endl;
            kill(child_pid, SIGKILL);
        }
        else
        {
            std::cout << "child process starting: " << getpid() << std::endl;
            int ret = execl(path_to_bfs.c_str(), path_to_bfs.c_str(), NULL);
            std::cout << "child ret (failed to run): " << ret << std::endl;
            return ret;
        }
    }

natural_exit:

    std::cout << "Parent process ending!" << std::endl;

    if(child_pid > 0)
    {
        std::string save_path = parent_path_str + "/save.txt";

        if(fs::exists(save_path))
        {
            fs::remove(save_path);
            std::cout << "removed save.txt" << std::endl;
        }
    }

    
    return 0;
}