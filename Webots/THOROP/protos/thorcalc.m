function thorcalc()

	%some matlab code to transform robotis com positions and inertia matrices into our format


%webots and robotis: x is left, y is up, z is front
%our code: x is front, y is left, z is up 

%those are based on our code coordinate
	cshoulder_to_neck=[0,0,117];
	cshoulder_to_lshoulder=[0,234,0];
	lshoulder_to_lelbow=[0,261,-30];
	lelbow_to_lwrist=[0,252,30];
	waist_to_cshoulder=[0,0,276];
	chip_to_waist=[0,0,180];
	chip_to_lhip = [0,105,0];
	chip_to_rhip = [0,-105,0];
	knee_to_hip=[0,0,300];
	ankle_to_knee=[0,0,300];


	function ret=conv_robotis(name,mass,com, origin) 
		

	
		acom_code=([com(3) com(1) com(2)]-origin)/1000;
		acom = [acom_code(2) acom_code(3) acom_code(1)];


		str=sprintf('%s: mass %.3f webots com[%.4f %.4f %.4f]', name, mass,acom(1),acom(2),acom(3));
		disp(str);

	end




	%robotis relative com positions (from chip)

	cshoulder = chip_to_waist+waist_to_cshoulder;


	conv_robotis('torso',9.802,[-0.7 417.8 -4.2], cshoulder );
	conv_robotis('waist', 0.490,  [0 237.9 0], chip_to_waist);	
	conv_robotis('pelvis', 4.540, [0.2 183.2 -21.2], chip_to_waist );

	conv_robotis('lhipyaw',0.935,[105.3 50.4 -28.0],chip_to_lhip);
	conv_robotis('lhiproll',0.911,[93.9 0 -0.2],chip_to_lhip);	
	conv_robotis('luleg',3.322,[92.8 -150 11.9],chip_to_lhip);
	conv_robotis('llleg',4.165,[89.3 -400.1 6.6],chip_to_lhip-knee_to_hip);
	conv_robotis('lankle',0.911,[105.2 -600 -11.1],chip_to_lhip-knee_to_hip-ankle_to_knee);
	conv_robotis('lfoot',1.616,[106.8 -682.8 11.3],chip_to_lhip-knee_to_hip-ankle_to_knee);


	conv_robotis('rhipyaw',0.935,[-105.3 50.4 -28.0],chip_to_rhip);
	conv_robotis('rhiproll',0.911,[-93.9 0 -0.2],chip_to_rhip);	
	conv_robotis('ruleg',3.322,[-92.8 -150 11.9],chip_to_rhip);
	conv_robotis('rlleg',4.165,[-89.3 -400.1 6.6],chip_to_rhip-knee_to_hip);
	conv_robotis('rankle',0.911,[-105.2 -600 -11.1],chip_to_rhip-knee_to_hip-ankle_to_knee);
	conv_robotis('rfoot',1.616,[-106.8 -682.8 11.3],chip_to_rhip-knee_to_hip-ankle_to_knee);



	conv_robotis('head1',0.373,[1.8 572.0 0],cshoulder+cshoulder_to_neck);
	conv_robotis('head2',0.515,[-0.5 652.8 0],cshoulder+cshoulder_to_neck);


	conv_robotis('lshoulderpitch',0.940,[233.3 456.0 0.8], cshoulder+cshoulder_to_lshoulder);
	conv_robotis('lshoulderroll',0.752,[262.2 510.9 0.1], cshoulder+cshoulder_to_lshoulder);
	conv_robotis('luarm',1.806,[414.8 443.1 -0.2], cshoulder+cshoulder_to_lshoulder);
	conv_robotis('llarm',1.124,[618.9 456.1 0.3], cshoulder+cshoulder_to_lshoulder+lshoulder_to_lelbow);
	conv_robotis('lwristpitch',0.441,[740 456 -1.5], cshoulder+cshoulder_to_lshoulder+lshoulder_to_lelbow+lelbow_to_lwrist);
%{
	



	
	




	



	
	conv_robotis('lwristroll',0.077,[776 456 3.5]);
	conv_robotis('lwristyaw',0.474,[842.3 460 -1.5]);

	conv_robotis('lhand',1.484,[985.5 456 0.2]);
%}
end