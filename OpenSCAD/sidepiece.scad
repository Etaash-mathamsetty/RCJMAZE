include <roundedcube.scad>

$fn = 30;
CYLINDER_X = 6.1;
CYLINDER_Y = 6;
CYLINDER_Z = 25;
BASE_WIDTH = 70.25;
BASE_LENGTH = 62;
PICASE_X = 45.5;
PICASE_Y = 31;
PICASE_Z = 6;

COLOR_WIDTH = 22;

TOF_LEN = 26;
TOF_WIDTH = 18;

module cylinders2(h, r1, r2, LEN, WIDTH) {
     for (i = [0:WIDTH:WIDTH]) {
         for (n = [0:LEN:LEN]) {
             translate([i,n,0])cylinder(h, r1, r2);
         }
     }    
} 

module cylinders3(h, r1, r2, LEN, WIDTH, xNum, yNum) {
     for (i = [0:WIDTH/(xNum-1):WIDTH]) {
         for (n = [0:LEN/(yNum-1):LEN]) {
             translate([i,n,0])cylinder(h, r1, r2);
         }
     }    
} 

module tof(LEN, WIDTH, radius) {
        translate([0,0,2]) cube([WIDTH, LEN, 1]);
        translate([radius+1,radius+1,0]) cylinders2(6, 2, 2, LEN-(radius+1)*2,WIDTH-(radius+1)*2);
        translate([WIDTH/2-10/2,-3,1]) cube([10,3,4]);
        translate([WIDTH/2-10/2,LEN,1]) cube([10,3,4]);
}

module sidePiece(WIDTH,LEN,radius) {
    difference() {
        translate([0,0,2]) roundedcube([WIDTH,2,LEN],false,1,"y");
        translate([5,3,30]) rotate([90,90,0])tof(TOF_LEN,TOF_WIDTH,2);
        
        
        //rotate([90,0,0]) translate([radius + 1 + WIDTH/2 - 24/2,radius+1+2,-2]) cylinders3(6,radius,radius,24-(radius+1)*2,24-(radius+1)*2,3,1);
    }
    difference() {
        translate([0,1,2]) translate([WIDTH/2 - 30/2,0,0]) roundedcube([30,27,2],false,1,"z");
        translate([radius+1 + WIDTH/2 - 24/2,radius+1+2,0]) cylinders3(6,radius,radius,24-(radius+1)*2,24-(radius+1)*2,3,3);
    }
}


sidePiece(36,36,2.5);