cwd = os.getenv('PWD')
cwd = cwd..'/../../Player';
package.path = cwd..'/?.lua;'..package.path
require('init')

require('lfs')
