const express = require("express");
const http = require("http");
const socketio = require("socket.io");
const app = express();
const HttpServer = http.createServer(app);
const io = socketio(HttpServer);
const HTTP_PORT = 5000;
const url = require("url");

require("../Data_Reciver/Node_reciver.js")(io);

app.get("/", (req, res) => {
    res.sendFile("./Vizualize.html", {root: __dirname });
});
io.on("connect", (socket) => {
    client_connected=true
    console.log("Client connected");
});
io.on("disconnect", (socket) => {
    client_connected=false
    console.log("Client disconnected");
});
HttpServer.listen(HTTP_PORT, () => {
    console.log(`listening on ${HTTP_PORT}`);
});