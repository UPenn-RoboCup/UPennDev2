clear all;
zmq('publish','matlab');
zmq('send','hello world!');
