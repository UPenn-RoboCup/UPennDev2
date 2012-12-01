require('Hokuyo')
require('signal')

function ShutDownFN()
  print("Proper shutdown")
  Hokuyo.shutdown()
  os.exit(1);
end

while (true) do
  signal.signal("SIGINT", ShutDownFN);
  signal.signal("SIGTERM", ShutDownFN);
end
