$fn=30;
BASE_LEN=110;
BASE_WIDTH=60;
PI_WIDTH=54;
PI_LEN=64;
OUTSIDE_WIDTH=57;
OUTSIDE_LEN=97;
CYLINDER=3;



module cylinders(h,r,width,length){
    for(i=[0:width:width]){
        for(n=[0:length:length]){
            translate([CYLINDER+n,CYLINDER+i,0])cylinder(h,r,r);
        }        
    }
}

difference(){
    cube([BASE_LEN,BASE_WIDTH,5]);
    translate([(BASE_LEN-PI_LEN)/2,(BASE_WIDTH-PI_WIDTH)/2,-1])cylinders(10,2,PI_WIDTH-CYLINDER*2,PI_LEN-CYLINDER*2);
    translate([(BASE_LEN-OUTSIDE_LEN)/2,(BASE_WIDTH-OUTSIDE_WIDTH)/2,-1])cylinders(10,2,OUTSIDE_WIDTH-CYLINDER*2,OUTSIDE_LEN-CYLINDER*2);
}
