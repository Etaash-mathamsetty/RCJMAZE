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

void run_parent_and_child(const fs::path& path_to_bfs, const fs::path& parent_path)
{
    pid_t child_pid = fork();
    if(child_pid > 0)
    {
        std::cout << "parent process starting: " << getpid() << std::endl;
        //wait for child to start
        sleep(2);
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
        //kill parent if we can't execute the inputted path
        kill(getppid(), SIGKILL);
        exit(ret);
    }

    return;

natural_exit:

    std::cout << "Parent process ending!" << std::endl;

    if(child_pid > 0)
    {
        std::string save_path = parent_path.string() + "/save.txt";

        if(fs::exists(save_path))
        {
            fs::remove(save_path);
            std::cout << "removed save.txt" << std::endl;
        }
    }

    exit(EXIT_SUCCESS);
}

//pass in arg to BFS path
int main(int argc, char **argv)
{
    fs::path path_to_bfs;
    if(argc >= 2)
    {
        std::string path = std::string(argv[1]);
        if(!fs::exists(path))
        {
            std::cout << "pls pass in path that exists" << std::endl;
            return -1;
        }
        path_to_bfs = fs::canonical(fs::absolute(path));
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

    if(!path_to_bfs.has_parent_path())
    {
        std::cout << "no parent path" << std::endl;
        return -1;
    }
    fs::path parent_path = path_to_bfs.parent_path();
    std::cout << "parent path: " << parent_path << std::endl;


    while(true)
    {
        run_parent_and_child(path_to_bfs, parent_path);
    }

    
    return 0;
}