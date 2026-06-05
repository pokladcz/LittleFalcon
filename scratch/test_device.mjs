import { getDevice } from "file:///C:/Users/pokla/AppData/Roaming/npm/node_modules/jaculus-tools/dist/commands/util.js";

async function main() {
    const port = "COM8";
    const baudrate = 921600;
    
    console.log(`Connecting to device on ${port}...`);
    const device = await getDevice(port, baudrate, undefined, {});
    
    console.log("Waiting 2 seconds for boot...");
    await new Promise(resolve => setTimeout(resolve, 2000));
    
    console.log("Requesting status...");
    const status = await device.controller.status();
    console.log("Status response:", status);
    
    console.log("Requesting version...");
    const ver = await device.controller.version();
    console.log("Version response:", ver);
    
    await device.destroy();
}

main().catch(err => {
    console.error("Diagnostic error:", err);
    process.exit(1);
});
