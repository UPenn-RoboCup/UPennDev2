local mot = {};
mot.servos = {
    1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,
};
mot.keyframes = {
   {	--Straight out limbs.
     angles = vector.new({
         0, 0, 
         109.8, 11.0, -88.9, -21.4, 
         -13.7, -0.3, 17.1, -5.6, 5.2, 0, 
         0.0, -1.8, 14.6, -5.6, 4.9, 0, 
         109.9, -10.2, 88.7, 19.9, 
     })*math.pi/180,
     duration = 0.400;
   },
   {	--Bend in knees, rotate wrists
     angles = vector.new({
         0, 0, 
         121.0, 7.8, -2.3, -3.4, 
         0, 0, 24.0, 65.8, 28.2, 0, 
         0, 0, 24.0, 65.8, 28.2, 0, 
         121.0, -9.4, -5.4, 9.5, 
     })*math.pi/180,
     duration = 0.400;
   },
   {	--Pull in arms under tailbone
     angles = vector.new({
         0, 0, 
         120.1, 11.4, 1.1, -81.7, 
         0, 0, 18.9, 64.9, 31.5, 0, 
         0, 0, 18.9, 64.9, 31.5, 0, 
         120.1, -14.4, -2.6, 82.8,
     })*math.pi/180,
     duration = 0.400;
   },
	 {	--Re-extend legs, rotate body up
		angles = vector.new({
         0, 0, 
         130.1, 24.4, 0, -81.7, 
         0, 0, -50, 0, 31.5, 0, 
         0, 0, -50, 0, 31.5, 0, 
         130.1, -24.4, 0, 82.8,
     })*math.pi/180,
     duration = 0.400;
	 },
	 {	--Spread stance, rotate forearms down, bend knees
		angles = vector.new({
         0, 0, 
         130.1, 0, 0, -20, 
         -60, 20, -60, 50, 40, 0, 
         -60, -20, -60, 50, 40, 0, 
         130.1, 0, 0, 20,
     })*math.pi/180,
     duration = 0.300;
	 },
   {	--Plant feet on ground
		angles = vector.new({
         0, 0, 
         130.1, 0, 0, -20, 
         -40, 20, -80, 95, 35, 0, 
         -40, -20, -80, 95, 35, 0, 
         130.1, 0, 0, 20,
     })*math.pi/180,
     duration = 0.200;
	 },
	 { --Lift left side; left arm out
		angles = vector.new({
         0, 60, 
         130.1, 20, 0, 0, 
         -40, 20, -30, 125, -20, -10, 
         -40, -20, -80, 95, 25, -20, 
         130.1, -10, 0, 10,
     })*math.pi/180,
     duration = 0.300;
	 },
	 { --Take weight onto left leg
		angles = vector.new({
         0, 60, 
         110.1, 20, 0, 0, 
         -40, 20, -25, 125, -30, -10, 
         -40, -30, -80, 95, 20, -10, 
         130.1, -10, 0, 10,
     })*math.pi/180,
     duration = 0.100;
	 },
	 { --Shift onto left ankle
		angles = vector.new({
         0, 60, 
         110.1, 20, 0, 0, 
         -40, 20, -15, 125, -45, -10, 
         -40, -30, -72, 90, 20, -10, 
         130.1, -25, 0, 10,
     })*math.pi/180,
     duration = 0.100;
	 },
	 { --Shift onto left ankle
		angles = vector.new({
         0, 60, 
         110.1, 20, 0, 0, 
         -40, 20, -15, 125, -45, -10, 
         -40, -30, -72, 90, 20, -10, 
         130.1, -25, 0, 10,
     })*math.pi/180,
     duration = 5.100;
	 },

};

return mot;
