import { getDevice } from "file:///C:/Users/pokla/AppData/Roaming/npm/node_modules/jaculus-tools/dist/commands/util.js";
import { WifiMode } from "file:///C:/Users/pokla/AppData/Roaming/npm/node_modules/jaculus-tools/node_modules/@jaculus/device/dist/device.js";

async function main() {
    const port = "COM17";
    const baudrate = 921600;
    const ssid = "LittleFalcon-AP";
    const password = "14141414";
    
    console.log(`Connecting to device on ${port}...`);
    const device = await getDevice(port, baudrate, undefined, {});
    
    console.log("Waiting 2.5 seconds for ESP32 boot to complete...");
    await new Promise(resolve => setTimeout(resolve, 2500));
    
    console.log("Stopping running program...");
    try {
        await device.controller.stop();
        console.log("Program stopped.");
    } catch (err) {
        console.log("Note (could be already stopped):", err);
    }
    
    console.log("Locking device...");
    await device.controller.lock();
    
    console.log("Setting WiFi mode to AP (Access Point)...");
    await device.controller.setWifiMode(WifiMode.AP);
    
    console.log(`Setting AP SSID to "${ssid}"...`);
    await device.controller.setWifiApSsid(ssid);
    
    console.log(`Setting AP Password to "${password}"...`);
    await device.controller.setWifiApPassword(password);
    
    console.log("Unlocking device...");
    await device.controller.unlock();
    
    console.log("Successfully configured WiFi AP Mode on the device!");
    await device.destroy();
}

main().catch(err => {
    console.error("Error:", err);
    process.exit(1);
});
