const http = require('http');
const fs = require('fs');
const path = require('path');
const crypto = require('crypto');
const dgram = require('dgram');

const HTTP_PORT = 3000, ROBOT_PORT = 4444;

const server = http.createServer((req, res) => {
    fs.readFile(path.join(__dirname, 'index.html'), (err, data) => {
        if (err) { res.writeHead(500); res.end('Error loading index.html'); }
        else { res.writeHead(200, { 'Content-Type': 'text/html; charset=utf-8' }); res.end(data); }
    });
});

server.on('upgrade', (req, socket) => {
    if (req.headers['upgrade']?.toLowerCase() === 'websocket') {
        const key = req.headers['sec-websocket-key'];
        const acceptKey = crypto.createHash('sha1').update(key + '258EAFA5-E914-47DA-95CA-C5AB0DC85B11').digest('base64');
        socket.write(`HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Accept: ${acceptKey}\r\n\r\n`);
        
        console.log('Prohlížeč připojen.');
        let robotIp = '';
        const udpClient = dgram.createSocket('udp4');
        
        udpClient.on('message', (msg) => {
            try { sendWsText(socket, msg.toString('utf8')); } catch (e) {}
        });
        
        socket.on('data', (buf) => {
            try {
                let offset = 0;
                while (offset < buf.length) {
                    const byte1 = buf[offset], byte2 = buf[offset + 1], opcode = byte1 & 0x0f;
                    if (opcode === 8) { socket.end(); break; }
                    
                    let len = byte2 & 0x7f;
                    offset += 2;
                    if (len === 126) { len = buf.readUInt16BE(offset); offset += 2; }
                    
                    const mask = buf.slice(offset, offset + 4);
                    offset += 4;
                    const payload = buf.slice(offset, offset + len);
                    offset += len;
                    
                    if (opcode === 1) {
                        for (let i = 0; i < payload.length; i++) payload[i] ^= mask[i % 4];
                        const msg = JSON.parse(payload.toString('utf8'));
                        
                        if (msg.type === 'connect') {
                            robotIp = msg.ip;
                            console.log(`Cílová IP robota: ${robotIp}`);
                        } else if (msg.type === 'control' && robotIp) {
                            const pkt = JSON.stringify({ speed: msg.speed, steer: msg.steer });
                            udpClient.send(pkt, 0, pkt.length, ROBOT_PORT, robotIp);
                        }
                    }
                }
            } catch (e) {}
        });
        
        socket.on('close', () => { udpClient.close(); console.log('Prohlížeč odpojen.'); });
        socket.on('error', () => udpClient.close());
    }
});

function sendWsText(socket, text) {
    try {
        const payload = Buffer.from(text, 'utf8');
        const header = Buffer.alloc(2);
        header[0] = 0x81;
        header[1] = payload.length;
        socket.write(Buffer.concat([header, payload]));
    } catch (e) {}
}

server.listen(HTTP_PORT, () => console.log(`=== SERVER BĚŽÍ NA http://localhost:${HTTP_PORT} ===`));
