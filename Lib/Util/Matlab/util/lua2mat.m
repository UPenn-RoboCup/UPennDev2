function st = lua2mat(msg)
% function that will convert serialized Lua data into
% a matlab struct

% msg package type define in monitorComm
% 11 non-Image data
% 18 Y small
% 19 U small
% 20 V small
imageid = [18,19,20];
	if (msg(1) == 11)
			str = char(msg(2:end));	
		%
		% NOTE: This is not a complete solution, some serialized 
		%       types are not supported
		
		  str = char(str);
		  str = regexprep(str, '\{', 'struct(');  
		  str = regexprep(str, '\}', ')');  
		  str = regexprep(str, '=', ',');  
		  str = regexprep(str, '\[\"', '''');  
		  str = regexprep(str, '\"\]', '''');  
		  str = regexprep(str, '\"', '''');  
		  str = regexprep(str, ',\)', ')');
		  
		  % Match for nested structs to support sending team states
		  %str = regexprep(str, '[', 'struct('); 
		  %str = regexprep(str, ']', ')'); 
		
		  % convert any arrays to array format (only 1D arrays are supported)
		  [matched, split] = regexp(str, 'struct\(\[.+?\)', 'match', 'split');
		  if ~isempty(matched)
		    str = split{1};
		    for i = 1:length(matched)
		      % convert table/array format to array
		      match = regexprep(matched{i}, 'struct\(', '[');
		      match = regexprep(match, '\[[0-9]*\],', '');
		      match = regexprep(match, '\)', ']');
		      str = strcat(str, match, split{i+1});
		    end
		  end
		  %st = str;
		  st = eval(str);
	elseif (sum(msg(1)==imageid)>0)
		str = {};
		str.image = {};
		str.image.type = double(msg(1));
		str.image.width = double(msg(2)) * 256 + double(msg(3));
		str.image.height = double(msg(4)) * 256 + double(msg(5));
		str.image.division = double(msg(8));
		str.image.section = double(msg(9)+1); % C array start from 0
		str.image.data = msg(10:end);
		str.team = {};
		str.team.number = double(msg(6));
		str.team.player_id = double(msg(7));
		 
		st = str;
		%disp('image update recived!');
	else
		str = {};
		str.team = {};
		str.team.number = -1;
		str.team.player_id = -1;
		st = str;
	end

end
