import { getDevice } from "file:///C:/Users/pokla/AppData/Roaming/npm/node_modules/jaculus-tools/dist/commands/util.js";

async function main() {
    const port = "COM8";
    const baudrate = 921600;
    
    console.log(`Connecting to device...`);
    const device = await getDevice(port, baudrate, undefined, {});
    
    console.log("Waiting for boot...");
    await new Promise(resolve => setTimeout(resolve, 2000));
    
    console.log("--- Device Object Keys ---");
    console.log(Object.keys(device));
    
    console.log("--- Controller Object Keys ---");
    console.log(Object.keys(device.controller));
    
    if (device.fs) {
        console.log("--- FS Object Keys ---");
        console.log(Object.keys(device.fs));
    }
    
    await device.destroy();
}

main().catch(console.error);
