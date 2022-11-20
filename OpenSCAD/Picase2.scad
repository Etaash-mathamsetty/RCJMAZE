$fn = 30;
CYLINDER_X = 6.1;
CYLINDER_Y = 6;
CYLINDER_Z = 25;
BASE_WIDTH = 70.25;
BASE_LENGTH = 62;
PICASE_X = 45.5;
PICASE_Y = 31;
PICASE_Z = 6;

module cylinders(h, r1, r2){
     for (i = [0:BASE_WIDTH - CYLINDER_X*2:BASE_WIDTH - CYLINDER_X*2]){
         for (n = [0:BASE_LENGTH - CYLINDER_Y*2:BASE_LENGTH - CYLINDER_Y*2]){
            translate([CYLINDER_X + i,CYLINDER_Y + n, CYLINDER_Z]) cylinder(h, r1, r2);
         }
     }
}

difference() {
    translate([PICASE_X, PICASE_Y, PICASE_Z]) rotate(90, [-1,0,0]) import(
    "/home/alawn/Downloads/files/v3_upper.stl");
        cylinders(10, 2, 2);



    scale([0.65, 0.55, 0.55]) translate([3, 130, -15]) rotate([90,0,270]) linear_extrude(height=4) import("/home/alawn/Downloads/REA1.svg");
}

translate([PICASE_X, PICASE_Y, PICASE_Z]) rotate(90, [-1,0,0]) {
import("/home/alawn/Downloads/files/v3_upper.stl");
import("/home/alawn/Downloads/files/Pi_Case_Upper_30mm_fan.stl");
import("/home/alawn/Downloads/files/V3_lower.stl");
import("/home/alawn/Downloads/files/Pi_Case_Upper_Tall.stl");
import("/home/alawn/Downloads/files/Pi_Case_Upper_solid.stl");
import("/home/alawn/Downloads/files/Pi_case_Upper_all_cutouts.stl");
}



//cylinder(10, 2, 2);

