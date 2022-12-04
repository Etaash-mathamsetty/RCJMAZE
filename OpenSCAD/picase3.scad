$fn = 32;

BASE_WIDTH = 56;
BASE_LENGTH = 85;
HEIGHT = 4;

HOLES_DIST = 58;
CYLINDER_OFFSET = 3;

HANGER_LEN = 10;
HANGAR_WIDTH = 2;
HANGER_HEIGHT = 20;



module cylinders3(h, r1, r2, LEN, WIDTH, xNum, yNum) {
     for (i = [0:LEN/(xNum-1):LEN]){
         for (n = [0:WIDTH/(yNum-1):WIDTH]){
             translate([i,n,0])cylinder(h, r1, r2);
            }
        }    
    } 

module hanger() {
    translate([0,0,-HANGER_HEIGHT + HEIGHT]) cube([HANGER_LEN,HANGER_WIDTH,HANGER_HEIGHT]);
    }

module base() {
    difference() {
        cube([BASE_LENGTH, BASE_WIDTH, HEIGHT]);
            translate([CYLINDER_OFFSET, CYLINDER_OFFSET, -1]) cylinders3(10, 2, 2, HOLES_DIST, BASE_WIDTH - CYLINDER_OFFSET * 2, 2, 2);
        }
    }
    
    translate([-HANGER_LEN/2 + CYLINDER_OFFSET + HOLES_DIST,0,0]) hanger();
    translate([-HANGER_LEN/2 + CYLINDER_OFFSET + HOLES_DIST,BASE_WIDTH - HANGER_WIDTH,0]) hanger();
    
base();