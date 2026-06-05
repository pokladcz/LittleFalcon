import * as keyvalue from "keyvalue";
import * as wifi from "wifi";
import { SmartLed, LED_WS2812B } from "smartled";
const leds = new SmartLed(36, 8, LED_WS2812B);
function sleep(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
}
async function main() {
    console.log("=== PROGRAM PRO NASTAVENÍ WIFI ROBOTA ===");
    // 1. Uložíme síť G14 do databáze sítí wifi_net (SSID: heslo)
    console.log("Ukládám heslo pro síť G14...");
    const net = keyvalue.open("wifi_net");
    net.set("G14", "14141414");
    net.commit();
    // 2. Nastavíme konfiguraci WiFi v wifi_cfg
    console.log("Nastavuji WiFi do režimu STATION (klientský režim)...");
    const cfg = keyvalue.open("wifi_cfg");
    cfg.set("mode", 1); // 1 = STATION (připojit se k hotspotu)
    cfg.set("sta_mode", 1); // 1 = SPECIFIC_SSID (připojit se ke konkrétní síti)
    cfg.set("sta_ssid", "G14"); // SSID cílové sítě
    cfg.set("sta_ap_fallback", 1); // Pokud se nepřipojí, vytvoří záložní AP
    cfg.commit();
    console.log("Konfigurace v NVS uložena. Čekám na připojení k hotspotu G14...");
    while (true) {
        const ip = wifi.currentIp();
        if (ip) {
            console.log(`PŘIPOJENO! IP adresa robota: ${ip}`);
            // Rozsvítíme zelenou barvu jako signalizaci úspěchu
            leds.clear();
            for (let i = 0; i < 8; i++) {
                leds.set(i, 0x002000); // Zelená
            }
            leds.show();
        }
        else {
            console.log("Připojuji se k G14... (zapněte hotspot na notebooku)");
            // Blikáme žlutě
            leds.clear();
            for (let i = 0; i < 8; i++) {
                leds.set(i, 0x202000); // Žlutá
            }
            leds.show();
            await sleep(500);
            leds.clear();
            leds.show();
        }
        await sleep(1500);
    }
}
main().catch(e => {
    console.log("Chyba v programu nastavení WiFi: " + e);
});
