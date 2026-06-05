import { getDevice } from "file:///C:/Users/pokla/AppData/Roaming/npm/node_modules/jaculus-tools/dist/commands/util.js";
import { WifiMode, WifiStaMode } from "file:///C:/Users/pokla/AppData/Roaming/npm/node_modules/jaculus-tools/node_modules/@jaculus/device/dist/device.js";

async function main() {
    const port = "COM17";
    const baudrate = 921600;
    
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
    
    // Add Laptop WiFi Network
    console.log('Adding Laptop WiFi network: SSID="G14"...');
    await device.controller.addWifiNetwork("G14", "14141414");
    
    // Add Phone WiFi Network
    console.log('Adding Phone WiFi network: SSID="hugo boss"...');
    await device.controller.addWifiNetwork("hugo boss", "123456789");
    
    console.log("Setting WiFi mode to STATION...");
    await device.controller.setWifiMode(WifiMode.STATION);
    
    console.log("Setting Station mode to BEST_SIGNAL (scans for any saved network)...");
    await device.controller.setWifiStaMode(WifiStaMode.BEST_SIGNAL);
    
    console.log("Enabling AP fallback (in case neither network is found)...");
    await device.controller.setWifiStaApFallback(true);
    
    console.log("Unlocking device...");
    await device.controller.unlock();
    
    console.log("Successfully configured Dual WiFi (G14 & hugo boss) on the device!");
    await device.destroy();
}

main().catch(err => {
    console.error("Error:", err);
    process.exit(1);
});
