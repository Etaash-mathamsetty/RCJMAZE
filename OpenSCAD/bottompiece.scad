include <roundedcube.scad>

$fn = 32;

BASE_WIDTH = 56;
BASE_LENGTH = 85;
HEIGHT = 4;

HOLES_DIST = 58;
CYLINDER_OFFSET = 3;

HANGER_LEN = 10;
HANGER_WIDTH = 3;
HANGER_HEIGHT = 13;

module cylinders3(h, r1, r2, LEN, WIDTH, xNum, yNum) {
     for (i = [0:LEN/(xNum-1):LEN]){
         for (n = [0:WIDTH/(yNum-1):WIDTH]){    
             translate([i,n,0])cylinder(h, r1, r2);
            }
        }    
    } 

module hanger() {
    difference() {
        translate([0,0,-HANGER_HEIGHT+1]) roundedcube(size=[HANGER_LEN,HANGER_WIDTH,HANGER_HEIGHT],center=false,radius=1,apply_to="zmin");
        translate([0,0,-HANGER_HEIGHT]) translate([HANGER_LEN/2,4,4]) rotate([90,0,0]) cylinder(10,2,2);
        }
    }

module base() {
    difference() {

        union() {
        roundedcube([BASE_LENGTH, BASE_WIDTH, HEIGHT],false,1,"z");
            translate([0,0,0]) hanger();
            translate([0,BASE_WIDTH - HANGER_WIDTH,0]) hanger();
            translate([BASE_LENGTH - HANGER_LEN,0,0]) hanger();
            translate([BASE_LENGTH - HANGER_LEN,BASE_WIDTH - HANGER_WIDTH,0]) hanger();
            }
            translate([CYLINDER_OFFSET + 10, CYLINDER_OFFSET, -1]) cylinders3(10, 2, 2, HOLES_DIST, BASE_WIDTH - CYLINDER_OFFSET * 2, 2, 2);
            translate([CYLINDER_OFFSET + HOLES_DIST, CYLINDER_OFFSET, -1]) cylinders3(10, 2, 2, HOLES_DIST, BASE_WIDTH - CYLINDER_OFFSET * 2, 1, 2);
            translate([-30,-17,3.7]) linear_extrude(height=3) import("/home/alawn/Documents/GitHub/RCJMAZE/OpenSCAD/REA1.svg");
        }
    }
base();