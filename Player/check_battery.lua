cwd = os.getenv('PWD')
require('init')

require('unix');
require('shm');
sensorShm = shm.open('dcmSensor');
print(string.format("Battery: %.1f V\n", sensorShm:get('battery')/10));
