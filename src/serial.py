#import Robot as Rb

if not simulation:
    # code here
    # keep reading until serial buffer is empty and store to Rb.SetDataValue()
    if(ser.in_waiting >= 5):
        data = ser.readline().decode('ascii', 'ignore').split("::")
        data[1] = data[1][0:-2]
        split_data = [float (x) for x in data[1].split(',')]
        Rb.SetDataValue(data[0], split_data)

