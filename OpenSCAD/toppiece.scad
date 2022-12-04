include <roundedcube.scad>

$fn = 32;

BRACKET_WIDTH = 72;
BRACKET_LEN = 24;
HEIGHT = 3;
PI_WIDTH = 56;
PI_LEN = 88;
HOLES_DIST = 58;
EDGE = 0;
HOLE_RADIUS = 1.5;
HOLE_OFFSET = 3;

module cylinders2(h, r1, r2, LEN, WIDTH) {
     for (i = [0:WIDTH:WIDTH]){
         for (n = [0:LEN:LEN]){
             translate([i,n,0])cylinder(h, r1, r2);
         }
     }    
} 

module bracket() {
import("/home/alawn/Documents/GitHub/RCJMAZE/OpenSCAD/plate.stl");
}

module squarehole() {
    difference() {
        roundedcube(size=[BRACKET_WIDTH + EDGE * 2, PI_LEN, HEIGHT],center=false,radius=1,apply_to="z");
        translate([EDGE+1,PI_LEN/2-BRACKET_LEN/2+1,-1]) cube([BRACKET_WIDTH-2,BRACKET_LEN-2,HEIGHT+2]);
        translate([(BRACKET_WIDTH + EDGE * 2)/2-(PI_WIDTH - HOLE_OFFSET * 2)/2,3, -1]) cylinders2(5,HOLE_RADIUS,HOLE_RADIUS,HOLES_DIST,PI_WIDTH - HOLE_OFFSET * 2);
        
        echo((BRACKET_WIDTH + EDGE * 2)/2-(PI_WIDTH - HOLE_RADIUS * 2)/2);
        }
    }

module toppiece() {
    squarehole();
    translate([EDGE,PI_LEN/2,1.5]) rotate([90,0,0]) bracket();

    }

toppiece();