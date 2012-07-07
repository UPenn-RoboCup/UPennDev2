require 'primesense'

while( true ) do
  ret = primesense.update_joints();
  if( ret ) then
    for i=1,2 do -- Loop through users
      pos, rot, confidence, active = primesense.get_jointtables( i, 1 );
      print('User: '..i..":"..active);
    end
  end
end
