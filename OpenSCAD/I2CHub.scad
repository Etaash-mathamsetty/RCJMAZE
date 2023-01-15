module I2C(){
    I2C_width = 27;
    I2C_length = 38;
     cube([I2C_width,I2C_length,3]);
     for(n = [3.5:I2C_width - 7:I2C_width - 3.5])
         for(i = [3.5:I2C_length - 7:I2C_length-3.5])
            translate([n,i,-3]) cylinder(10,1.5,1.5);
     translate([I2C_width/2-13/2,4,-3])cube([13,35,10]); 
}
