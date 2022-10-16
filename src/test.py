#used for testing python integration
import Robot as Rb

def WriteSerialCommand(command):
    print(command)

arr =  [1.412, 2.718, 3.1415926]
print("hello world")
Rb.SetDataValue("test", arr)  