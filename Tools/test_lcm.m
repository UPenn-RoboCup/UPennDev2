% Include the classpath definitions
[status,result] = ...
    system('/usr/local/bin/pkg-config --variable=classpath lcm-java');
javaaddpath({result(1:end-1),'../Framework/Lib/lcm/thor_matlab.jar'})
p = javaclasspath

% Secret Sauce
lc = lcm.lcm.LCM.getSingleton();
aggregator = lcm.lcm.MessageAggregator();

% Start listening on the channel
lc.subscribe('EXAMPLE', aggregator);

while true
    disp waiting
    millis_to_wait = 1000;
    msg = aggregator.getNextMessage(millis_to_wait);
    if ~isempty(msg)
        break
    end
end

disp(sprintf('channel of received message: %s', char(msg.channel)))
disp(sprintf('raw bytes of received message:'))
disp(sprintf('%d ', msg.data'))

m = thor.rpc_request_t(msg.data);

disp(sprintf('decoded message:\n'))
disp([ 'client id:   ' sprintf('%s ', char(m.client_id) ) ])
disp([ 'request id:    ' sprintf('%d ', m.request_id) ])
disp([ 'eval_string: ' sprintf('%s ', char(m.eval_string) ) ])
disp([ 'synchronous:      ' sprintf('%d ', m.synchronous) ])
%disp([ 'name:        ' char(m.name) ])
%disp([ 'enabled:     ' sprintf('%d ', m.enabled) ])
