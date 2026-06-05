import { getDevice } from "file:///C:/Users/pokla/AppData/Roaming/npm/node_modules/jaculus-tools/dist/commands/util.js";
import { WifiMode, WifiStaMode } from "file:///C:/Users/pokla/AppData/Roaming/npm/node_modules/jaculus-tools/node_modules/@jaculus/device/dist/device.js";

async function main() {
    const port = "COM8";
    const baudrate = 921600;
    const ssid = "G14";
    const password = "14141414";
    
    console.log(`Connecting to device on ${port}...`);
    const device = await getDevice(port, baudrate, undefined, {});
    
    console.log("Waiting 2.5 seconds for ESP32 boot to complete...");
    await new Promise(resolve => setTimeout(resolve, 2500));
    
    console.log("Stopping running program to free up serial/CPU resources...");
    try {
        await device.controller.stop();
        console.log("Program stopped.");
    } catch (err) {
        console.log("Note (could be already stopped):", err);
    }
    
    console.log("Locking device...");
    await device.controller.lock();
    
    console.log(`Adding WiFi network: SSID="${ssid}"...`);
    await device.controller.addWifiNetwork(ssid, password);
    
    console.log("Setting WiFi mode to STATION...");
    await device.controller.setWifiMode(WifiMode.STATION);
    
    console.log(`Setting Station mode to connect specifically to "${ssid}"...`);
    await device.controller.setWifiStaMode(WifiStaMode.SPECIFIC_SSID);
    await device.controller.setWifiStaSpecific(ssid);
    await device.controller.setWifiStaApFallback(true);
    
    console.log("Unlocking device...");
    await device.controller.unlock();
    
    console.log("Successfully configured WiFi G14 on the device!");
}

main().catch(err => {
    console.error("Error:", err);
    process.exit(1);
});
