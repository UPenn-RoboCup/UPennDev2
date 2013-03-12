%This function generates arm trajectory from hand drawing (using mouse)
%Usage: drag to draw, press any key to publish

function draw_gen(filename)
  global DATA;
  close all;
  clear DATA;
  DATA.button = 0;
  DATA.xpos=[];
  DATA.ypos=[];
  DATA.count = 0;
  DATA.strokecount = 0;
  DATA.strokenum=[];
  
  %Load background file
  rgb=imread(filename);
  image(rgb);
  size_x = size(rgb,2);
  size_y = size(rgb,1);
  size_max = max(size_x,size_y);   
  
%   figure;
  set (gcf, 'WindowButtonMotionFcn', @mouseMove);
  set (gcf, 'WindowButtonDownFcn', @buttonDown);
  set (gcf, 'WindowButtonUpFcn', @buttonUp);
  set (gcf, 'KeyPressFcn', @keyPressed);
    
 function mouseMove(object,eventdata)
   C=get(gca,'CurrentPoint');
   xpos_new = C(1,1);
   ypos_new = C(1,2);
             
   if DATA.button         
     mov = sqrt((DATA.xpos(DATA.count) - xpos_new)^2+ (DATA.ypos(DATA.count)-ypos_new)^2);                
     hold on;
     if mov>size_max*0.025 
       add_pos(C(1,1), C(1,2));                
       hold on;
       plot([DATA.xpos(DATA.count) DATA.xpos(DATA.count-1)],[DATA.ypos(DATA.count) DATA.ypos(DATA.count-1)]);
       axis([1,size_x,1,size_y]);
       drawnow;                
       hold off;                
     end                       
     title(gca,[num2str(DATA.strokecount), 'Xstrokes X,Y =(', num2str( C(1,1)), ',' , num2str(C(1,2)),')']);
   else
     title(gca,[num2str(DATA.strokecount), ' strokes X,Y =(', num2str( C(1,1)), ',' , num2str(C(1,2)),')']);
   end
 end
    
 function add_pos(x,y)
   DATA.count = DATA.count + 1;   
   DATA.xpos(DATA.count) = x;
   DATA.ypos(DATA.count) = y;             
 end
    function buttonDown(object,eventdata)
       if DATA.button==0 
%         disp('BUTTONDOWN')
         C=get(gca,'CurrentPoint');
         add_pos(C(1,1), C(1,2));
         DATA.strokecount = DATA.strokecount + 1;
         DATA.button = 1;
       end
    end
    function buttonUp(object,eventdata)
        DATA.strokenum(DATA.strokecount)=DATA.count ;
        DATA.button = 0;
    end
    function keyPressed(object,eventdata)
        disp('GENERATING.....')
        %Generate lua file for the drawing
        fid = fopen('strokedef.lua','w');
        fprintf(fid,'strokedef={\n');
        count = 1;
        
        %Normalize the positions to (-0.5, 0.5)
        size_max = max(size_x,size_y);
                
        for i=1:DATA.strokecount            
          fprintf(fid,'  {\n');
          fprintf(fid,'    {\n     ');          
          for j=count:DATA.strokenum(i)              
            fprintf(fid,'%.2f, ',(size_x/2-DATA.xpos(j))/size_max) ;  
          end
          fprintf(fid,'\n    },\n');          
          
          fprintf(fid,'    {\n     ');          
          for j=count:DATA.strokenum(i)              
            fprintf(fid,'%.2f, ',(size_y/2-DATA.ypos(j))/size_max);  
          end
          fprintf(fid,'\n    },\n');          
                              
          count = DATA.strokenum(i)+1;
          fprintf(fid,'  },\n');
        end             
        fprintf(fid,'}\n');
        fclose(fid);
    end
end


