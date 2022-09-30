const dgram = require('dgram');
const server = dgram.createSocket('udp4');
UDP_LISTEN_PORT = 9005
client_connected = false
var socket_io;
server.on('error', (err) => {
  console.log(`server error:\n${err.stack}`);
  server.close();
});

server.on('message', (msg, rinfo) => {
  message =`${msg}`
  //=console.log(`server got: ${msg} from ${rinfo.address}:${rinfo.port}`);
  console.log(message)
  //msg=msg.split(',')
  if (client_connected)
  socket_io.emit("sensor_data",message);
});

server.on('listening', () => {
  const address = server.address();
  console.log(`server listening ${address.address}:${address.port}`);

});

server.bind(UDP_LISTEN_PORT);

module.exports = function(io){
  socket_io=io;
 }