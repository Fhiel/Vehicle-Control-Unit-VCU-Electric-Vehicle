#ifndef WEB_UI_H
#define WEB_UI_H

#include <Arduino.h>

// --- INSERT YOUR WIDE-FORMAT BASE64 STRING HERE ---
#define LOGO_BASE64 "data:image/png;base64,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"

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no, viewport-fit=cover">
    <title>X1/9e VCU Dashboard</title>
    <style>
    /* --- IMPORTS & TOKENS --- */
    @import url('https://fonts.googleapis.com/css2?family=Montserrat:wght@600;800&display=swap');    
    
    :root {
        --bg-color: #000000;
        --card-bg: #0d0d0d;
        --bertone-gold: #d4af37;
        --instrument-green: #00e68a; 
        --instrument-glow: rgba(0, 230, 138, 0.65);
        --warning-red: #ff1a1a; 
        --text-main: #ffffff;
        --text-dim: #888888;
        --border-ui: #1a1a1a;

        --tesla-blue: var(--bertone-gold);
        --tesla-green: var(--instrument-green);
        --tesla-red: var(--warning-red);
    }

    body { 
        font-family: 'Montserrat', sans-serif; 
        background: var(--bg-color); color: var(--text-main); 
        margin: 0; padding: 0; display: flex; justify-content: center; min-height: 100vh;
    }

    body:not(.show-expert) { overflow: hidden !important; position: fixed; width: 100%; height: 100%; }
    body.show-expert { overflow-y: auto !important; position: relative; }

    #page-home { display: block !important; }
    #page-expert { display: none !important; }
    body.show-expert #page-home { display: none !important; }
    body.show-expert #page-expert { display: block !important; }

    .page-section { width: 100%; min-height: 100vh; }

    .dashboard-wrapper {
        margin-top: 0; padding-top: 70px; display: flex; flex-direction: column; 
        width: 100%; max-width: 1300px; padding-left: 10px; padding-right: 10px; padding-bottom: 15px;
        box-sizing: border-box; gap: 5px;
    }

    .header-bar {
        position: fixed; top: 0; left: 0; right: 0; height: 64px; z-index: 10000;
        background-color: #000; border-bottom: 1px solid var(--border-ui);
        margin: 0 !important; padding: 0 10px; box-sizing: border-box; display: flex; align-items: flex-start;
    }

    .logo-area { z-index: 2; display: flex; margin: 0; padding: 0; }
    .logo-img { height: 50px; width: auto; display: block; margin-left: 0; cursor: pointer; }

    #status-tag { 
        cursor: pointer; position: absolute; left: 50%; top: 3px; transform: translateX(-50%); 
        height: 60px; display: flex; align-items: flex-start; padding-top: 5px;
        font-size: 1.0em; font-weight: bold; letter-spacing: 2px; color: var(--bertone-gold); 
        z-index: 1001; text-shadow: 0 1px 3px rgba(0,0,0,0.9);
    }
    #status-tag:active { color: #fff; }

    .range-box { 
        cursor: pointer; user-select: none; text-align: right; display: flex; flex-direction: column; 
        align-items: flex-end; position: absolute; right: 12px; top: 4px; height: 55px; z-index: 10; min-width: 120px;
    }
    .range-val { font-size: 3.2em; font-weight: 900; line-height: 0.8; } 
    .range-unit { font-size: 0.7em; color: var(--text-dim); text-transform: none; }

    .control-row { display: flex; justify-content: center; margin: 2px 0; }
    
    .mode-grid {
        display: grid; grid-template-columns: repeat(3, 1fr); gap: 15px; width: 100%; max-width: 100%; 
        margin: 0 auto 20px auto !important;
    }

    .mode-card {
        background: var(--card-bg); border: 2px solid var(--border-ui); border-radius: 8px; height: 55px; 
        display: flex; flex-direction: column; align-items: center; justify-content: center; cursor: pointer; transition: 0.2s;
    }

    .mode-label { 
        font-size: 0.55em; font-weight: 800; color: #222; text-transform: uppercase; letter-spacing: 0.8px;
        margin-top: 4px; transition: color 0.3s;
    }

    .mode-active { border-color: #555 !important; }
    .mode-active .icon-svg { fill: #ffffff !important; filter: drop-shadow(0 0 5px rgba(255,255,255,0.3)); }
    .mode-active .mode-label { color: #ffffff !important; }

    .mode-card:not(.mode-active):not(.bolt-card) .icon-svg { fill: #222; }
    .mode-card:not(.mode-active):not(.bolt-card) .mode-label { color: #222; }

    .bolt-card { cursor: pointer; border-color: var(--border-ui); }
    .bolt-card .icon-svg { fill: #444; }

    .icon-grid { display: grid; width: 100%; grid-template-columns: repeat(4, 1fr); gap: 15px; margin: 0 auto 20px !important; }

    .icon-card { 
        background: var(--card-bg); border: 2px solid var(--border-ui); border-radius: 8px; height: 55px; 
        display: flex; flex-direction: column; align-items: center; justify-content: center; 
        transition: 0.2s; cursor: default; padding: 0; box-sizing: border-box;
    }

    .icon-svg { width: 40px; height: 40px; display: flex; align-items: center; justify-content: center; fill: #ffffff; }
    .icon-svg svg { width: 100%; height: 100%; }

    .card-on-red { background-color: rgba(232, 18, 36, 0.2) !important; border-color: #e81224 !important; color: #ff4d4d; box-shadow: 0 0 10px rgba(232, 18, 36, 0.4); }
    .card-on-green { background-color: rgba(61, 219, 103, 0.15) !important; border-color: #3ddb67 !important; color: #3ddb67; box-shadow: 0 0 10px rgba(61, 219, 103, 0.3); }
    .card-on-red .icon-svg, .card-on-green .icon-svg { fill: #ffffff !important; opacity: 1; }

    .main-diag { display: grid; grid-template-columns: repeat(2, 1fr); gap: 10px; }
    .diag-box { background: rgba(255,255,255,0.02); border: 2px solid rgba(0, 230, 138, 0.35); padding: 8px 10px 10px 10px; border-radius: 6px; box-shadow: 0 0 14px rgba(0, 230, 138, 0.2); }
    .diag-header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 6px; min-height: 32px; }
    .diag-label { font-size: 0.68em; color: var(--bertone-gold); text-transform: uppercase; letter-spacing: 1.5px; font-weight: 800; line-height: 1; }

    .scale-container { position: relative; height: 27px; width: 100%; background: #000; border-bottom: 1px solid #222; display: flex; align-items: flex-end; justify-content: space-between; padding: 0 3px; box-sizing: border-box; margin: 0 auto !important; }
    .tick { background: #fff; width: 1px; height: 18px; opacity: 0.8; }
    .tick.thick { width: 3px; height: 22px; opacity: 1; }
    .tick.gold { background: var(--bertone-gold); }

    .red-zone { position: absolute; right: 6px; bottom: 0; width: 12%; height: 20px; background: var(--warning-red); z-index: 1; }
    .iso-yellow { position: absolute; right: 12%; bottom: 0; width: 12%; height: 20px; background: #ffcc00; z-index: 1; }
    .iso-red { position: absolute; right: 6px; bottom: 0; width: 12%; height: 20px; background: var(--warning-red); z-index: 1; }

    .scale-labels { position: relative; z-index: 5; display: flex; justify-content: space-between; padding: 0 4px; margin-top: 2px; }
    .scale-val { color: var(--bertone-gold); font-size: 1.0em; font-weight: 800; }
    .gauge-wrapper { position: relative; height: 52px; width: 100%; margin: 4px 0 6px 0; }
    
    .needle { position: absolute; top: 3px; left: 0; width: 5px; height: 43px; background: linear-gradient(to right, #eee 0%, #fff 50%, #eee 100%); border: 1px solid #ccc; border-radius: 1px; box-shadow: 0 0 10px rgba(255,255,255,0.7); transition: left 0.3s ease-out; z-index: 40; }
    .val-top { font-family: monospace; font-size: 1.22em; font-weight: 800; color: var(--text-main); line-height: 1; }

    .expert-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 12px; margin-top: 5px; border-top: 1px solid var(--border-ui); padding-top: 20px; }
    .expert-box { background: var(--card-bg); border: 1px solid #222; padding: 15px; border-radius: 15px; }
    .expert-box h2 { font-size: 0.75em; color: var(--bertone-gold); text-transform: uppercase; margin: 0 0 10px 0; border-bottom: 1px solid #1a1a1a; padding-bottom: 6px; }
    
    .data-row { display: flex; justify-content: space-between; align-items: center; font-size: 0.85em; margin-bottom: 6px; color: #ccc; }
    .val { font-weight: bold; color: #fff; }
    .unit { color: var(--text-dim); margin-left: 3px; font-size: 0.75em; }

    /* +50% HEIGHT BUTTONS FOR S10e */
    #page-expert button {
        background: #111; color: #eee; border: 1px solid #444;
        padding: 16px 8px !important; border-radius: 6px; font-size: 1.1em !important;
        font-weight: bold; cursor: pointer; transition: 0.2s;
        min-height: 52px; display: flex; align-items: center; justify-content: center;
    }
    #page-expert button:hover { background: #222; border-color: var(--instrument-green); }

    .fan-spin { display: inline-block; animation: spin 2.5s linear infinite; }
    @keyframes spin { 100% { transform: rotate(360deg); } }

    @keyframes pulse-green {
        0% { opacity: 0.3; transform: scale(1); filter: drop-shadow(0 0 2px var(--tesla-green)); }
        50% { opacity: 1; transform: scale(1.1); filter: drop-shadow(0 0 15px var(--tesla-green)); }
        100% { opacity: 0.3; transform: scale(1); filter: drop-shadow(0 0 2px var(--tesla-green)); }
    }
    @keyframes strobe-red {
        0% { color: var(--tesla-red); opacity: 1; }
        10% { color: #000; opacity: 0; }
        20% { color: var(--tesla-red); opacity: 1; }
        100% { color: var(--tesla-red); opacity: 1; }
    }

    .bolt-charging { animation: pulse-green 2s infinite ease-in-out; color: var(--tesla-green) !important; }
    .bolt-error { animation: strobe-red 0.8s infinite; color: var(--tesla-red) !important; }
    .bolt-warning { color: var(--tesla-red) !important; opacity: 1; }
    .bolt-limit { color: var(--tesla-blue) !important; opacity: 0.6; }
    .bolt-idle { color: #444 !important; opacity: 0.3; }
</style>
</head>
<body>
    <div class="dashboard-wrapper">
        <div class="header-bar">
            <div class="logo-area" onclick="showPage('home')">
                <img src=")rawliteral" LOGO_BASE64 R"rawliteral(" class="logo-img" alt="Logo">
            </div>
            <div id="status-tag" onclick="showPage('expert')">BERTONE</div>
            <div class="range-box" onclick="toggleRangeDisplay()">
                <span class="range-val" id="range-display">0</span>
                <span class="range-unit" id="range-unit">Range km</span>
            </div>
        </div>

        <div id="page-home" class="page-section">
            <div class="control-row">
                <div class="mode-grid">
                    <div id="mode-trip" class="mode-card" onclick="send('MODE_TRIP')">
                        <span class="icon-svg">
                            <svg viewBox="0 0 24 24"><path d="M11,2V4.07C7.38,4.53 4.53,7.38 4.07,11H2V13H4.07C4.53,16.62 7.38,19.47 11,19.93V22H13V19.93C16.62,19.47 19.47,16.62 19.93,13H22V11H19.93C19.47,7.38 16.62,4.53 13,4.07V2H11M12,6A6,6 0 0,1 18,12A6,6 0 0,1 12,18A6,6 0 0,1 6,12A6,6 0 0,1 12,6Z"/></svg>
                        </span>
                        <span class="mode-label">Trip</span>
                    </div>
                    <div class="mode-card bolt-card" onclick="if(confirm('Stop Charging?')) send('CHARGE_STOP_REQ')">
                        <span class="icon-svg">
                            <svg viewBox="0 0 24 24"><path d="M7,2V13H10V22L17,10H13L17,2H7Z"/></svg>
                        </span>
                        <span class="mode-label">Charge</span>
                    </div>
                    <div id="mode-daily" class="mode-card" onclick="send('MODE_DAILY')">
                        <span class="icon-svg">
                            <svg viewBox="0 0 24 24"><path d="M10,20V14H14V20H19V12H22L12,3L2,12H5V20H10Z"/></svg>
                        </span>
                        <span class="mode-label">Daily</span>
                    </div>
                </div>
            </div>

            <div class="icon-grid">
                <div class="icon-card" id="card-rel-1">
                    <span class="icon-svg">
                        <svg viewBox="0 0 32 32"><path d="M 11 9 L 11 11 L 13 11 L 13 13 L 7.5625 13 L 5.84375 10.4375 L 5.53125 10 L 1 10 L 1 15.6875 L 6 17.6875 L 6 25 L 20.53125 25 L 20.8125 24.5625 L 29.5 12 L 31 12 L 31 10 L 27.65625 10 L 27.40625 10.1875 L 21 15 L 21 13 L 15 13 L 15 11 L 17 11 L 17 9 Z M 3 12 L 4.4375 12 L 6 14.34375 L 6 15.5 L 3 14.3125 Z M 25.78125 13.9375 L 19.5 23 L 8 23 L 8 15 L 19 15 L 19 19 L 20.59375 17.8125 Z M 29.5 16 C 29.5 16 28 18.671875 28 19.5 C 28 20.328125 28.671875 21 29.5 21 C 30.328125 21 31 20.328125 31 19.5 C 31 18.671875 29.5 16 29.5 16 Z"/></svg>
                    </span>
                </div>
                <div class="icon-card" id="card-rel-2" onclick="send('PIEZO_TOGGLE')" style="cursor: pointer;">
                    <span class="icon-svg">
                        <svg viewBox="0 0 24 24"><path d="M12,2L1,21H23M12,6L19.53,19H4.47M11,10H13V14H11M11,16H13V18H11V16Z" /></svg>
                    </span>
                </div>
                <div class="icon-card" id="card-rel-3">
                    <span class="icon-svg">
                        <svg id="icon-fan" viewBox="0 0 24 24" fill="currentColor"><path d="M12,11a1,1,0,1,0,1,1,1,1,0,0,0-1-1m.5-9C17,2,17.1,5.57,14.73,6.75a3.36,3.36,0,0,0-1.62,2.47,3.17,3.17,0,0,1,1.23.91C18,8.13,22,8.92,22,12.5c0,4.5-3.58,4.6-4.75,2.23a3.44,3.44,0,0,0-2.5-1.62,3.24,3.24,0,0,1-.91,1.23c2,3.69,1.2,7.66-2.38,7.66C7,22,6.89,18.42,9.26,17.24a3.46,3.46,0,0,0,1.62-2.45,3,3,0,0,1-1.25-.92C5.94,15.85,2,15.07,2,11.5,2,7,5.54,6.89,6.72,9.26A3.39,3.39,0,0,0,9.2,10.87a2.91,2.91,0,0,1,.92-1.22C8.13,6,8.92,2,12.48,2Z"/></svg>
                    </span>
                </div>
                <div class="icon-card" id="card-rel-4">
                    <span class="icon-svg" style="transform: rotate(90deg);">
                        <svg viewBox="0 0 1000 1000"><path d="M500 177q-115 0 -215 58q-96 56 -152 152q-58 100 -58 215t58 215q56 96 152 152q100 58 215 58t215 -58q96 -56 152 -152q58 -100 58 -215t-58 -215q-56 -96 -152 -152q-100 -58 -215 -58zM500 259q93 0 173 46q78 46 124 124q46 80 46 173t-46 173q-46 78 -124 124 q-80 46 -173 46q-36 0 -72 -7l389 -336l-388 -336q35 -7 71 -7zM333 302l361 302l-358 300q-82 -45 -130 -124q-49 -82 -49 -177.5t48 -176.5q48 -79 128 -124z"/></svg>
                    </span>
                </div>
            </div>

            <div class="main-diag">
                <div class="diag-box ok" id="box-it">
                    <div class="diag-header">
                        <span class="diag-label">Inv</span>
                        <div class="val-top"><span id="i-temp">--</span><span class="unit">°C</span></div>
                    </div>
                    <div class="gauge-wrapper">
                        <div id="i-temp-needle" class="needle"></div>
                        <div class="scale-container">
                            <div class="red-zone"></div>
                            <div class="tick thick gold"></div>
                            <div class="tick"></div><div class="tick"></div><div class="tick"></div><div class="tick"></div>
                            <div class="tick thick"></div>
                            <div class="tick"></div><div class="tick"></div><div class="tick"></div><div class="tick"></div>
                            <div class="tick thick gold"></div>
                            <div class="tick"></div><div class="tick"></div><div class="tick"></div><div class="tick"></div>
                            <div class="tick thick"></div>
                            <div class="tick"></div><div class="tick"></div><div class="tick"></div><div class="tick"></div>
                            <div class="tick thick gold"></div>
                        </div>
                        <div class="scale-labels">
                            <span class="scale-val">0</span>
                            <span class="scale-val">50</span>
                            <span class="scale-val">95</span>
                        </div>
                    </div>
                </div> 

                <div class="diag-box" id="box-iso">
                    <div class="diag-header">
                        <span class="diag-label">ISO-R</span>
                        <div class="val-top"><span id="iso-val">--</span><span class="unit">kΩ</span></div>
                    </div>
                    <div class="gauge-wrapper">
                        <div id="iso-needle" class="needle"></div>
                        <div class="scale-container">
                            <div class="iso-yellow"></div>
                            <div class="iso-red"></div>
                            <div class="tick thick gold"></div>
                            <div class="tick"></div><div class="tick"></div><div class="tick"></div><div class="tick"></div>
                            <div class="tick thick"></div>
                            <div class="tick"></div><div class="tick"></div><div class="tick"></div><div class="tick"></div>
                            <div class="tick thick gold"></div>
                            <div class="tick"></div><div class="tick"></div><div class="tick"></div><div class="tick"></div>
                            <div class="tick thick"></div>
                            <div class="tick"></div><div class="tick"></div><div class="tick"></div><div class="tick"></div>
                            <div class="tick thick gold"></div>
                        </div>
                        <div class="scale-labels">
                            <span class="scale-val">50k</span>
                            <span class="scale-val">1k</span>
                            <span class="scale-val">0</span>
                        </div>
                    </div>
                </div>
            </div>
        </div>

        <div id="page-expert" class="page-section" style="display:none;">
            <div class="expert-grid">
                <div class="expert-box">
                    <h2>MCU / Motor (0x239)</h2>
                    <div class="data-row"><span>Fault Level:</span><span class="val" id="mcu_flt">0</span></div>
                    <div class="data-row"><span>RPM:</span><span><span class="val" id="mcu_rpm">0</span><span class="unit">RPM</span></span></div>
                    <div class="data-row"><span>Motor Temp:</span><span><span class="val" id="mcu_mT_exp">0</span><span class="unit">°C</span></span></div>
                    <div class="data-row"><span>MCU Temp:</span><span><span class="val" id="mcu_cT">0</span><span class="unit">°C</span></span></div>
                </div>

                <div class="expert-box">
                    <h2>BMS (0x355, 0x356)</h2>
                    <div class="data-row"><span>Status:</span><span class="val" id="bms_st">0</span></div>
                    <div class="data-row"><span>SoC:</span><span><span class="val" id="bms_soc">0</span><span class="unit">%</span></span></div>
                    <div class="data-row"><span>Current:</span><span><span class="val" id="bms_cur">0.0</span><span class="unit">A</span></span></div>
                </div>

                <div class="expert-box">
                    <h2>IMD / Isolation</h2>
                    <div class="data-row"><span>Internal Status:</span><span class="val" id="imd_st_val">--</span></div>            
                    <div class="data-row"><span>Resistance:</span><span><span class="val" id="imd_res">--</span><span class="unit">kΩ</span></span></div>
                    <div class="data-row"><span>HV1 Voltage:</span><span><span class="val" id="imd_hv1">--</span><span class="unit">V</span></span></div>
                    <div class="data-row"><span>Self Test Running:</span><span class="val" id="vcu_selftest">n/a</span></div>           
                </div>

                <div class="expert-box">
                    <h2>VCU &amp; LED+ALARM</h2>
                    <div class="data-row"><span>LED Check Oil:</span><span class="val" id="hw_ledoil">n/a</span></div>
                    <div class="data-row"><span>LED Battery:</span><span class="val" id="hw_ledbat">n/a</span></div>
                    <div class="data-row"><span>ALARM PIEZO:</span><span class="val" id="hw_PIEZO">n/a</span></div>
                    <div class="data-row"><span>ALARM ACTIVE (Log):</span><span class="val" id="hw_piezo_act">n/a</span></div>
                </div>

                <div class="expert-box">
                    <h2>Cooling &amp; Pumps</h2>
                    <div class="data-row"><span>Battery Pump Relay:</span><span class="val" id="hw_batrelay">n/a</span></div>           
                    <div class="data-row"><span>Battery Pump PWM:</span><span><span class="val" id="bat-pump">0</span><span class="unit">%</span></span></div>
                    <div class="data-row"><span>Inverter Pump Relay:</span><span class="val" id="hw_invrelay">n/a</span></div>
                    <div class="data-row"><span>Inverter Pump PWM:</span><span><span class="val" id="inv-pump">0</span><span class="unit">%</span></span></div>
                    <div class="data-row"><span>Fan Relay:</span><span class="val" id="hw_fan">n/a</span></div>
                </div>

                <div class="expert-box">
                    <h2>Type2 Lock &amp; WS2812</h2>
                    <div class="data-row"><span>Manual Unlock:</span><span class="val" id="hw_manual">n/a</span></div>
                    <div class="data-row"><span>Type2 Lock IN1:</span><span class="val" id="hw_lock1">n/a</span></div>
                    <div class="data-row"><span>Type2 Lock IN2:</span><span class="val" id="hw_lock2">n/a</span></div>
                    <div class="data-row"><span>Lock Feedback:</span><span class="val" id="hw_lockfb">n/a</span></div>
                    <div class="data-row"><span>Port Lock:</span><span class="val" id="d-lock">--</span></div>
                    <div class="data-row"><span>WS2812 Status:</span><span class="val" id="ws2812_status" style="font-weight:bold;">n/a</span></div>
                </div>

                <div class="expert-box">
                    <h2>Proxy BMS (Hyper9)</h2>
                    <div class="data-row"><span>Hyper9 SoC (raw):</span><span class="val" id="p-soc">0</span></div>
                    <div class="data-row"><span>Power Limit:</span><span><span class="val" id="p-lim">100</span><span class="unit">%</span></span></div>
                    <div class="data-row"><span>Drive Inhibit:</span><span class="val" id="p-inh">--</span></div>
                    <div class="data-row"><span>System Fault:</span><span class="val" id="p-flt">--</span></div>
                </div>

                <div class="expert-box">
                    <h2>System Tests & Alarm</h2>
                    <div style="display:grid; grid-template-columns:1fr 1fr; gap:8px;">
                        <button id="btn-piezo" onclick="send('PIEZO_TOGGLE')">ALARM</button>
                        <button onclick="send('PIEZO_AUTO')">AUTO</button>
                        <button onclick="if(confirm('IMD Test?')) send('IMD_TEST_START')" style="color:var(--tesla-red); border-color:var(--tesla-red);">IMD TEST</button>
                        <button onclick="send('IMD_AUTO')">AUTO</button>
                    </div>
                </div>

                <div class="expert-box">
                    <h2>Hardware Overrides</h2>
                    <div style="display:grid; grid-template-columns: repeat(2, 1fr); gap:6px; font-size:0.8em;">
                        <button id="btn-oil" onclick="send('LED_CHECK_OIL_TOGGLE')">OIL LED</button>
                        <button onclick="send('LED_CHECK_OIL_AUTO')">AUTO</button>
                        
                        <button id="btn-bat" onclick="send('LED_BATTERY_TOGGLE')">BAT LED</button>
                        <button onclick="send('LED_BATTERY_AUTO')">AUTO</button>

                        <button id="btn-bat-pump" onclick="send('BAT_PUMP_TOGGLE')">PUMP BAT</button>
                        <button onclick="send('BAT_PUMP_AUTO')">AUTO</button>

                        <button id="btn-inv-pump" onclick="send('INV_PUMP_TOGGLE')">PUMP INV</button>
                        <button onclick="send('INV_PUMP_AUTO')">AUTO</button>
                        
                        <button id="btn-fan" onclick="send('FAN_TOGGLE')">FAN</button>
                        <button onclick="send('FAN_AUTO')">AUTO</button>
                    </div>
                </div>    

                <div class="expert-box">
                    <h2>Aux Relays (Reserve)</h2>
                    <div style="display:grid; grid-template-columns: repeat(4, 1fr); gap:8px;">
                        <button id="btn-rel11" onclick="send('REL1_TOGGLE')">R11</button>
                        <button id="btn-rel12" onclick="send('REL2_TOGGLE')">R12</button>
                        <button id="btn-rel13" onclick="send('REL3_TOGGLE')">R13</button>
                        <button id="btn-rel14" onclick="send('REL4_TOGGLE')">R14</button>
                    </div>
                </div>
            </div>
        </div>
    </div>

    <script>
        var ws = new WebSocket('ws://' + location.host + '/ws');
        
        function send(cmd) { 
            if(ws.readyState === 1) ws.send(cmd); 
        }

        function check(val, isValid) {
            return (isValid === true && val !== 255 && val !== 0 && val !== undefined && val !== null) ? val : "n/a";
        }

        function setSafeText(id, text) {
            const el = document.getElementById(id);
            if (el) el.innerText = (text !== undefined && text !== null) ? text : "n/a";
        }

        function toggleButtonStyle(id, active) {
            const btn = document.getElementById(id);
            if (btn) {
                if (active) {
                    btn.style.borderColor = "var(--instrument-green)";
                    btn.style.color = "var(--instrument-green)";
                    btn.style.boxShadow = "0 0 8px var(--instrument-green)";
                    btn.style.background = "#0d0d0d"; 
                } else {
                    btn.style.borderColor = "";
                    btn.style.color = "";
                    btn.style.boxShadow = "";
                    btn.style.background = ""; 
                }
            }
        }

        function showPage(pageId) {
            if(pageId === 'home') document.body.classList.remove('show-expert');
            else document.body.classList.add('show-expert');
            window.scrollTo(0, 0);
        }

        let displaySoC = false;
        function toggleRangeDisplay() {
            displaySoC = !displaySoC;
            updateRangeUI();
        }

        function updateRangeUI() {
            if (!window.lastData?.vcu) return;
            const valEl = document.getElementById('range-display');
            const unitEl = document.getElementById('range-unit');
            if (valEl && unitEl) {
                if (displaySoC) {
                    valEl.innerText = window.lastData.vcu.soc ?? 0;
                    unitEl.innerText = "% SoC";
                } else {
                    valEl.innerText = window.lastData.vcu.range ?? 0;
                    unitEl.innerText = "Range km";
                }
            }
        }

        function setGauge(id, val, min, max) { 
            const needle = document.getElementById(id + '-needle');
            const display = document.getElementById(id); 
            if (!needle && !display) return; 
            
            let numVal = parseFloat(val);
            if (display) {
                if (isNaN(numVal) || numVal === 255 || numVal === 0) {
                    display.innerText = "n/a";
                    if (needle) needle.style.opacity = "0.3";
                    return;
                } else {
                    display.innerText = numVal;
                    if (needle) needle.style.opacity = "1";
                }
            }
            if (needle) {
                let pct = ((numVal - min) / (max - min)) * 100;
                pct = Math.min(Math.max(pct, 0), 98); 
                needle.style.left = pct + "%";
            }
        }
        
        function setIsoGauge(val) { 
            const needle = document.getElementById('iso-needle');
            const display = document.getElementById('iso-val'); 
            if (!needle && !display) return;
            
            let numVal = parseFloat(val);
            if (display) {
                if (isNaN(numVal) || numVal === 255 || numVal === 0) {
                    display.innerText = "n/a";
                    if (needle) needle.style.opacity = "0.3";
                    return; 
                } else {
                    display.innerText = numVal;
                    if (needle) needle.style.opacity = "1";
                }
            }
            if (!needle) return;
            let safeVal = Math.max(numVal, 0.1); 
            let pct = ((Math.log(safeVal) - Math.log(0.1)) / (Math.log(50000) - Math.log(0.1))) * 100;
            let invertedPct = Math.min(Math.max(100 - pct, 0), 98);
            needle.style.left = invertedPct + "%";
        }

        ws.onmessage = function(e) {
            try {
                var d = JSON.parse(e.data);
                window.lastData = d;

                updateRangeUI();

                if (d.mcu) {
                    setGauge('m-temp', d.mcu.mt, 50, 150);
                    setGauge('i-temp', d.mcu.it, 0, 95);
                    setSafeText('mcu_rpm', check(d.mcu.rpm, d.mcu.rpmV));
                    setSafeText('mcu_mT_exp', check(d.mcu.mt, d.mcu.mtV));
                    setSafeText('mcu_cT', check(d.mcu.it, d.mcu.itV));
                    setSafeText('mcu_flt', check(d.mcu.flt, d.mcu.fltV));
                }

                if (d.imd) {
                    setIsoGauge(d.imd.r);
                    setSafeText('imd_res', check(d.imd.r, d.imd.rV));
                    let stText = check(d.imd.st, d.imd.stV);
                    if (stText !== "n/a" && d.imd.stV) {
                        if (d.imd.st === 1) stText += " (Warn)";
                        else if (d.imd.st >= 2) stText += " (Crit)";
                    }
                    setSafeText('imd_st_val', stText);
                }

                if (d.bms) {
                    setSafeText('bms_st',   d.bms.st);
                    setSafeText('bms_soc',  d.bms.soc);
                    setSafeText('bms_cur',  d.bms.a);
                    setSafeText('imd_hv1',  d.bms.v);
                }
                if (d.proxy) {
                    setSafeText('p-soc',    d.proxy.soc);
                    setSafeText('p-lim',    d.proxy.lim);
                    setSafeText('p-inh',    d.proxy.inh ? "INHIBIT" : "GO");
                    setSafeText('p-flt',    d.proxy.flt ? "FAULT" : "OK");
                }

                if (d.vcu) {
                    const dailyBtn = document.getElementById('mode-daily');
                    const tripBtn = document.getElementById('mode-trip');
                    if (dailyBtn && tripBtn) {
                        tripBtn.classList.toggle('mode-active', !!d.vcu.trip);
                        dailyBtn.classList.toggle('mode-active', !d.vcu.trip);
                    }

                    const lockEl = document.getElementById('d-lock');
                    if (lockEl) {
                        lockEl.innerText = d.vcu.unl ? "UNLOCKING" : (d.vcu.err ? "LOCK ERROR" : (d.vcu.run ? "SELFTEST" : "STABLE"));
                        lockEl.style.color = d.vcu.err ? "var(--tesla-red)" : "var(--instrument-green)";
                    }
                }

                if (d.hw) {
                    const c1 = document.getElementById('card-rel-1'); if (c1) c1.className = "icon-card " + (d.hw.led_oil ? "card-on-red" : "");
                    const c3 = document.getElementById('card-rel-3'); if (c3) c3.className = "icon-card " + (d.hw.fan_relay ? "card-on-green" : "");
                    const c4 = document.getElementById('card-rel-4'); if (c4) c4.className = "icon-card " + (d.hw.bat_pump_relay ? "card-on-green" : "");
                    
                    const c2 = document.getElementById('card-rel-2'); 
                    if (c2) {
                        const isAlarmOn = !!d.hw.piezo || !!d.hw.piezo_act;
                        c2.className = "icon-card " + (isAlarmOn ? "card-on-red" : "");
                    }

                    const fanIcon = document.getElementById('icon-fan');
                    if (fanIcon) fanIcon.classList.toggle("fan-spin", !!d.hw.fan_relay);

                    setSafeText('hw_ledoil',    d.hw.led_oil ? "ON" : "OFF");
                    setSafeText('hw_ledbat',    d.hw.led_battery ? "ON" : "OFF");
                    setSafeText('hw_PIEZO',     d.hw.piezo ? "ON" : "OFF");
                    setSafeText('hw_piezo_act', d.hw.piezo_act ? "ON" : "OFF");
                    setSafeText('hw_fan',       d.hw.fan_relay ? "ON" : "OFF");
                    setSafeText('hw_batrelay',  d.hw.bat_pump_relay ? "ON" : "OFF");
                    setSafeText('bat-pump',     d.hw.bat_pump_pwm ?? 0); 
                    setSafeText('hw_invrelay',  d.hw.inv_pump_relay ? "ON" : "OFF");
                    setSafeText('inv-pump',     d.hw.inv_pump_pwm ?? 0);
                    
                    setSafeText('hw_lock1',     d.hw.lock_in1 ? "ON" : "OFF");
                    setSafeText('hw_lock2',     d.hw.lock_in2 ? "ON" : "OFF");
                    setSafeText('hw_lockfb',    d.hw.lock_fb ? "CLOSED" : "OPEN");
                    setSafeText('hw_manual',    d.hw.manual_unlock ? "PRESSED" : "IDLE");

                    setSafeText('aux_rel11_val', d.hw.aux_rel11 ? "ON" : "OFF");
                    setSafeText('aux_rel12_val', d.hw.aux_rel12 ? "ON" : "OFF");
                    setSafeText('aux_rel13_val', d.hw.aux_rel13 ? "ON" : "OFF");
                    setSafeText('aux_rel14_val', d.hw.aux_rel14 ? "ON" : "OFF");
                    setSafeText('aux_in13_val',  d.hw.aux_in13 ? "HIGH" : "LOW");

                    toggleButtonStyle('btn-oil',      !!d.hw.led_oil);
                    toggleButtonStyle('btn-bat',      !!d.hw.led_battery);
                    toggleButtonStyle('btn-piezo',    (!!d.hw.piezo || !!d.hw.piezo_act));
                    toggleButtonStyle('btn-fan',      !!d.hw.fan_relay);
                    
                    toggleButtonStyle('btn-bat-pump', !!d.hw.bat_pump_relay);
                    toggleButtonStyle('btn-inv-pump', !!d.hw.inv_pump_relay);
                    
                    toggleButtonStyle('btn-rel11',    !!d.hw.aux_rel11);
                    toggleButtonStyle('btn-rel12',    !!d.hw.aux_rel12);
                    toggleButtonStyle('btn-rel13',    !!d.hw.aux_rel13);
                    toggleButtonStyle('btn-rel14',    !!d.hw.aux_rel14);
                }

                const wsEl = document.getElementById('ws2812_status');
                if (wsEl && d.hw) {
                    if (d.hw.ws2812 === 1) {
                        wsEl.innerText = "GREEN (CHARGING/READY)";
                        wsEl.style.color = "var(--instrument-green)";
                    } else if (d.hw.ws2812 === 2) {
                        wsEl.innerText = "RED (IMD FAULT)";
                        wsEl.style.color = "var(--tesla-red)";
                    } else {
                        wsEl.innerText = "OFF";
                        wsEl.style.color = "";
                    }
                }

            } catch (err) {
                console.error("WebSocket data error:", err);
            }
        };

        ws.onclose = function() { setTimeout(() => location.reload(), 2000); };
    </script>
</body>
</html>
)rawliteral";
#endif