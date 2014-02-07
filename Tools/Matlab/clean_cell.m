function ret = clean_cell(i_vector)
%Sometimes message unpack generates a cell, not a matrix
%This cleans the cell into a proper matrix
  if iscell(i_vector) 
    ret = zeros(size(i_vector));
    for i=1:size(i_vector,2)
      ret(i) = i_vector{i};
    end
  else
    ret = i_vector;
  end
end
