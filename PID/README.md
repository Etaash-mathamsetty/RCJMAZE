PID Turn function details: 

turn(angle, motorspeed, L or R) 

add this: 

static char L = 'L'; 
static char R = 'R'; 

Warning: left turn does not work with angle greater than 180.... fix on the way...  

PID straight drive 

straightDrive(encoders, motorspeed, tolerance) 
