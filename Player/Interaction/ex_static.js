var express = require('express');
var app = express();
var http = require('http')
app.use(express.static(__dirname + '/public'));

var server = http.createServer(app);
server.listen(8080);


