
module Color(){
    color_width = 22;
    translate([0,0,-1])cube([color_width, color_width, 3]);
    translate([2,18,-3])cube([18,3,10]);
        for(i = [3:color_width-6:color_width - 3]){
            translate([i,3,-3]) cylinder(10,1.5,1.5);
        }   
}
