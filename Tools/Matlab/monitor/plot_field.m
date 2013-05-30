function h = plot_field(handle,type)
  % plots the robocup field on the current axis
  cla( handle );

  if type==0 % Kidsize
    fieldX = [-3.00  3.00 3.00 -3.00 -3.00];
    fieldY = [-2.00 -2.00 2.00  2.00 -2.00];
    goalX = [3.00 (3.00+0.40) (3.00+0.40) 3.00];
    goalY = [-0.70 -0.70 0.70 0.70];
    penaltyX = [3.00 (3.00-0.60) (3.00-0.60) 3.00];
    penaltyY = [-1.50 -1.50 1.50 1.50];
    spotX=1.2;
    circleR = .625;
    fieldB=[-3.5 3.5 -2.5 2.5];

  elseif type==1 %SPL
    %old SPL

    %new SPL field
    %{
    fieldX = [-4.50  4.50 4.50 -4.50 -4.50];
    fieldY = [-3.00 -3.00 3.00  3.00 -3.00];
    goalX = [4.50 (4.50+0.50) (4.50+0.50) 4.50];
    goalY = [-0.80 -0.80 0.80 0.80];
    penaltyX = [4.50 (4.50-0.60) (4.50-0.60) 4.50];
    penaltyY = [-1.10 -1.10 1.10 1.10];
    spotX=2.7;
    circleR = .6;
    fieldB=[-5 5 -3.5 3.5];
    %}
    
    %testing field in Grasp
    fieldX = [-3.825  3.825 3.825 -3.825 -3.825];
    fieldY = [-2.55 -2.55 2.55  2.55 -2.55];
    goalX = [3.825 (3.825+0.50) (3.825+0.50) 3.825];
    goalY = [-0.70 -0.70 0.70 0.70];
    penaltyX = [3.825 (3.825-0.60) (3.825-0.60) 3.825];
    penaltyY = [-1.10 -1.10 1.10 1.10];
    spotX=2.295;
    circleR = .6375;
    fieldB=[-5 5 -3.5 3.5];

  elseif type==2 % Teensize
    fieldX = [-4.50  4.50 4.50 -4.50 -4.50];
    fieldY = [-3.00 -3.00 3.00  3.00 -3.00];
    goalX = [4.50 (4.50+0.50) (4.50+0.50) 4.50];
    goalY = [-0.80 -0.80 0.80 0.80];
    penaltyX = [4.50 (4.50-0.60) (4.50-0.60) 4.50];
    penaltyY = [-1.10 -1.10 1.10 1.10];
    spotX=2.7;
    circleR = .6;
    fieldB=[-5 5 -3.5 3.5];

  end
%  set(handle,'YDir','reverse');
  hold on;
  plot(handle, fieldX, fieldY, 'g-');
  fill(goalX, goalY, 'y');
  plot(handle, goalX, goalY, 'g-');
  fill(-goalX, goalY, 'c');
  plot(handle, -goalX, goalY, 'g-');

  plot(handle, penaltyX, penaltyY, 'g-');
  plot(handle, -penaltyX, penaltyY, 'g-');

  plot(handle, spotX,0,'go');
  plot(handle, -spotX,0,'go');

  centerX = [0  0];
  centerY = [fieldY(1) fieldY(3)];
  plot(handle, centerX, centerY, 'g-');

  circleT = 2*pi*[0:.01:1];
  circleX = circleR*cos(circleT);
  circleY = circleR*sin(circleT);
  hcircle = plot(handle, circleX, circleY, 'g-');
  hold off;

  set(gca, 'XTickMode', 'auto');
  set(gca, 'YTickMode', 'auto');
  grid off;
  axis equal;
  axis(fieldB);

end
