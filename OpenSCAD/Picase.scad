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

module cylinders(h, r1, r2){
     for (i = [0:BASE_WIDTH - CYLINDER_X*2:BASE_WIDTH - CYLINDER_X*2]){
         for (n = [0:BASE_LENGTH - CYLINDER_Y*2:BASE_LENGTH - CYLINDER_Y*2]){
            translate([CYLINDER_X + i,CYLINDER_Y + n, CYLINDER_Z]) cylinder(h, r1, r2);
         }
     }
}

module cylinders2(h, r1, r2, LEN, WIDTH) {
     for (i = [0:WIDTH:WIDTH]){
         for (n = [0:LEN:LEN]){
             translate([i,n,0])cylinder(h, r1, r2);
         }
     }    
} 

module cylinders3(h, r1, r2, LEN, WIDTH, xNum, yNum) {
     for (i = [0:WIDTH/(xNum-1):WIDTH]){
         for (n = [0:LEN/(yNum-1):LEN]){
             translate([i,n,0])cylinder(h, r1, r2);
         }
     }    
} 

module base(){
    
    difference(){
        translate([PICASE_X, PICASE_Y, PICASE_Z]) rotate(90, [-1,0,0]) {
                //import("/home/alawn/Downloads/files/v3_upper.stl");
                //import("/home/alawn/Downloads/files/Pi_Case_Upper_30mm_fan.stl");
                import("/home/alawn/Downloads/files/Pi_case_Upper_all_cutouts.stl");
        }
        cylinders(10,2,2);
        scale([0.65, 0.55, 0.55]) translate([3, 130, -15]) rotate([90,0,270]) linear_extrude(height=4) import("/home/alawn/Downloads/REA1.svg");
    }
    
    
    translate([PICASE_X, PICASE_Y, -10]) rotate(90, [-1,0,0]) import("/home/alawn/Downloads/files/V3_lower.stl");
}

module tof(LEN, WIDTH, radius){
        translate([0,0,2]) cube([WIDTH, LEN, 1]);
        translate([radius+1,radius+1,0]) cylinders2(6, 2, 2, LEN-(radius+1)*2,WIDTH-(radius+1)*2);
        translate([WIDTH/2-10/2,-3,1]) cube([10,3,4]);
        translate([WIDTH/2-10/2,LEN,1]) cube([10,3,4]);
}

module camera(LEN,WIDTH,radius){
        translate([0,0,2]) cube([WIDTH+6, LEN, 1]);
        translate([radius+2,radius+2,0]) cylinders2(6, 2, 2, LEN-(radius+2)*2,WIDTH-(radius+2)*2);
        translate([0,2,-20]) cube([7,LEN,2]);
       
}
module lPiece(WIDTH,LEN,radius){
    
    difference(){
        translate([0,0,2]) cube([WIDTH,2,LEN]);
        translate([5,3,30]) rotate([90,90,0])tof(TOF_LEN,TOF_WIDTH,2);
    }
    
    difference(){
        translate([0,2,2]) cube([WIDTH,LEN,2]);
        translate([radius+1,radius+1+2,0]) cylinders3(6,radius,radius,LEN-(radius+1)*2,WIDTH-(radius+1)*2,3,3);
    }
    
    difference(){
        translate([0,0,-30]) cube([WIDTH,2,LEN]);
        translate([5,3,0]) rotate([90,90,0]) camera(TOF_LEN,TOF_WIDTH,1);
    }
    //translate([5,3,]) rotate([90,90,0]) camera(TOF_LEN,TOF_WIDTH,1);
}

base();
translate([-40,-40,0]) lPiece(36,36,2.5);

//cylinders(6, 2, 2, TOF_LEN-6,TOF_WIDTH-5);
