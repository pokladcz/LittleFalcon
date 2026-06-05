const http = require('http');
const fs = require('fs');
const path = require('path');
const crypto = require('crypto');
const dgram = require('dgram');

const HTTP_PORT = 3000;
const ROBOT_PORT = 4444;

const server = http.createServer((req, res) => {
    // Standard file serving for index.html
    let filePath = path.join(__dirname, 'index.html');
    if (req.url !== '/' && req.url !== '/index.html') {
        res.writeHead(404);
        res.end('Not Found');
        return;
    }
    
    fs.readFile(filePath, (err, data) => {
        if (err) {
            res.writeHead(500);
            res.end('Error loading index.html');
        } else {
            res.writeHead(200, { 'Content-Type': 'text/html; charset=utf-8' });
            res.end(data);
        }
    });
});

// WebSocket Server (Zero-Dependency Implementation)
server.on('upgrade', (req, socket, head) => {
    if (req.headers['upgrade'] && req.headers['upgrade'].toLowerCase() === 'websocket') {
        const key = req.headers['sec-websocket-key'];
        const acceptKey = crypto
            .createHash('sha1')
            .update(key + '258EAFA5-E914-47DA-95CA-C5AB0DC85B11')
            .digest('base64');
            
        socket.write(
            'HTTP/1.1 101 Switching Protocols\r\n' +
            'Upgrade: websocket\r\n' +
            'Connection: Upgrade\r\n' +
            `Sec-WebSocket-Accept: ${acceptKey}\r\n` +
            '\r\n'
        );
        
        console.log('Browser connected to WebSocket server.');
        let robotIp = '';
        const udpClient = dgram.createSocket('udp4');
        
        // Listen for incoming UDP messages (telemetry) from the robot and forward to Web UI
        udpClient.on('message', (msg) => {
            try {
                const text = msg.toString('utf8');
                sendWsText(socket, text);
            } catch (e) {
                // Ignore errors
            }
        });
        
        socket.on('data', (buffer) => {
            let offset = 0;
            while (offset < buffer.length) {
                if (offset + 2 > buffer.length) break;
                const byte1 = buffer[offset];
                const byte2 = buffer[offset + 1];
                const opcode = byte1 & 0x0f;
                
                if (opcode === 8) { // WebSocket Connection Close
                    socket.end();
                    break;
                }
                
                const isMasked = (byte2 & 0x80) !== 0;
                let payloadLength = byte2 & 0x7f;
                
                offset += 2;
                if (payloadLength === 126) {
                    if (offset + 2 > buffer.length) break;
                    payloadLength = buffer.readUInt16BE(offset);
                    offset += 2;
                } else if (payloadLength === 127) {
                    if (offset + 8 > buffer.length) break;
                    payloadLength = Number(buffer.readBigUInt64BE(offset));
                    offset += 8;
                }
                
                let maskingKey;
                if (isMasked) {
                    if (offset + 4 > buffer.length) break;
                    maskingKey = buffer.slice(offset, offset + 4);
                    offset += 4;
                }
                
                if (offset + payloadLength > buffer.length) break;
                const payload = buffer.slice(offset, offset + payloadLength);
                offset += payloadLength;
                
                if (opcode === 1) { // Text frame
                    const decoded = Buffer.alloc(payload.length);
                    for (let i = 0; i < payload.length; i++) {
                        decoded[i] = payload[i] ^ maskingKey[i % 4];
                    }
                    try {
                        const msg = JSON.parse(decoded.toString('utf8'));
                        if (msg.type === 'connect') {
                            robotIp = msg.ip;
                            console.log(`Dynamic robot IP set to: ${robotIp}`);
                        } else if (msg.type === 'control') {
                            if (robotIp) {
                                const packet = JSON.stringify({
                                    speed: msg.speed,
                                    steer: msg.steer
                                });
                                udpClient.send(packet, 0, packet.length, ROBOT_PORT, robotIp);
                            }
                        }
                    } catch (e) {
                        // Ignore JSON parsing errors for safety
                    }
                }
            }
        });
        
        socket.on('close', () => {
            udpClient.close();
            console.log('Browser disconnected.');
        });
        
        socket.on('error', (err) => {
            console.log('WebSocket connection error:', err.message);
            udpClient.close();
        });
    }
});

server.listen(HTTP_PORT, () => {
    console.log(`=== LOCAL CONTROL SERVER RUNNING ===`);
    console.log(`Otevřete v prohlížeči: http://localhost:${HTTP_PORT}`);
});

// Helper function to send unmasked WebSocket text frame from server to browser
function sendWsText(socket, text) {
    try {
        const payload = Buffer.from(text, 'utf8');
        if (payload.length >= 126) return; // Only support small frames (<126 bytes) for telemetry
        const header = Buffer.alloc(2);
        header[0] = 0x81; // FIN + Text frame
        header[1] = payload.length;
        socket.write(Buffer.concat([header, payload]));
    } catch (e) {
        // Ignore socket write errors
    }
}
