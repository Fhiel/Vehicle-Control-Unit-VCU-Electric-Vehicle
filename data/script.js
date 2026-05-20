var ws = new WebSocket('ws://' + location.host + '/ws');

// =====================================================================
// DEFINITIVE UI CHANNEL SYNC ENGINE (STATE-PER-CHANNEL MAPPING)
// =====================================================================
function syncChannelUI(channelData, switchId, btnOnId, btnOffId, btnEcoId = null, btnBoostId = null, mainCardId = null) {
    if (!channelData) return;

    const isManual = (channelData.m === 1);
    const stage    = channelData.s; // 0 = OFF, 1 = ON/ECO, 2 = BOOST

    // Auto/Manual Schieberegler synchronisieren
    const sw = document.getElementById(switchId);
    if (sw) sw.checked = isManual;

    const btnOn    = document.getElementById(btnOnId);
    const btnOff   = document.getElementById(btnOffId);
    const btnEco   = document.getElementById(btnEcoId);
    const btnBoost = document.getElementById(btnBoostId);
    const mainCard = document.getElementById(mainCardId);

    // --- 1. EXPERTEN-PAGE BUTTON RENDERING ---
    if (isManual) {
        // Handbetrieb: Buttons leuchten hell, Rahmen folgt der gewählten Stufe
        if (btnOn)    toggleButtonStyle(btnOnId,    stage === 1);
        if (btnOff)   toggleButtonStyle(btnOffId,   stage === 0);
        if (btnEco)   toggleButtonStyle(btnEcoId,   stage === 1);   
        if (btnBoost) toggleButtonStyle(btnBoostId, stage === 2);   

        if (btnOn)    btnOn.style.opacity    = (stage === 1) ? "1.0" : "0.4";
        if (btnOff)   btnOff.style.opacity   = (stage === 0) ? "1.0" : "0.4";
        if (btnEco)   btnEco.style.opacity   = (stage === 1) ? "1.0" : "0.4";
        if (btnBoost) btnBoost.style.opacity = (stage === 2) ? "1.0" : "0.4";
    } else {
        // Automatik-Modus: Buttons blassen ab (0.25), zeigen aber den AKTUELLEN Auto-Zustand an!
        if (btnOn)    btnOn.style.opacity    = "0.25";
        if (btnOff)   btnOff.style.opacity   = "0.25";
        if (btnEco)   btnEco.style.opacity   = "0.25";
        if (btnBoost) btnBoost.style.opacity = "0.25";
        
        // Rahmen folgt dem automatischen Ist-Zustand aus der VCU
        if (btnOn)    toggleButtonStyle(btnOnId,    stage === 1);
        if (btnOff)   toggleButtonStyle(btnOffId,   stage === 0);
        if (btnEco)   toggleButtonStyle(btnEcoId,   stage === 1);
        if (btnBoost) toggleButtonStyle(btnBoostId, stage === 2);
    }

    // --- 2. MAIN COCKPIT DASHBOARD COUPLING (PAGE-HOME) ---
    if (mainCard) {
        if (mainCardId === 'card-rel-2') {
            // Master Visual Alarm Card: Follows clean stage state integer unswayed by auto/manual bounds
            mainCard.className = "icon-card " + (stage === 1 ? "card-on-red" : "");
        } else if (mainCardId === 'card-rel-3') {
            // Radiator Fan home-tile tracking
            if (stage === 1) mainCard.classList.add('card-active-fan');
            else             mainCard.classList.remove('card-active-fan');
        } else {
            // Standard digital visual tracking (Oil LED & Coolant Loop lines)
            if (stage > 0) {
                mainCard.style.borderColor = "var(--instrument-green)";
                mainCard.style.boxShadow    = "0 0 10px var(--instrument-green)";
                mainCard.style.background   = "#0d0d0d";
            } else {
                mainCard.style.borderColor = "";
                mainCard.style.boxShadow    = "";
                mainCard.style.background   = "";
            }
        }
    }

    // --- 2. MAIN COCKPIT DASHBOARD COUPLING (PAGE-HOME) ---
    if (mainCard) {
        const isActive = (stage === 1);

        if (mainCardId === 'card-rel-1' || mainCardId === 'card-rel-2') {
            // Red tiles (OIL LED & ALARM)
            if (isActive) {
                mainCard.className = "icon-card card-on-red";
            } else {
                mainCard.className = "icon-card";
            }
        } 
        else if (mainCardId === 'card-rel-3') {
            // RADIATOR FAN: turns green and & SVG-Icon spins
            const svgIcon = mainCard.querySelector('svg');
            
            if (isActive) {
                mainCard.className = "icon-card card-on-green"; 
                if (svgIcon) svgIcon.classList.add('fan-spin'); 
            } else {
                mainCard.className = "icon-card";
                if (svgIcon) svgIcon.classList.remove('fan-spin');
            }
        } 
        else if (mainCardId === 'card-rel-4') {
            // INVERTER PUMP (tile 4)
            const pumpActive = (stage > 0);
            if (pumpActive) {
                mainCard.className = "icon-card card-on-green";
            } else {
                mainCard.className = "icon-card";
            }
        }
    }
}
        
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
            unitEl.innerText = "%"; // Clean, tight percentage sign inline
        } else {
            valEl.innerText = window.lastData.vcu.range ?? 0;
            unitEl.innerText = " km"; // Stripped of text, matching standard dashboard design
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
            if (needle) {
                needle.style.opacity = "1";
                needle.style.left = "1%";
            }
            return;
        } else {
            display.innerText = numVal;
            if (needle) needle.style.opacity = "1";
        }
    }
    if (needle) {
        let pct = ((numVal - min) / (max - min)) * 88;
        let finalLeft = 6 + pct;
        finalLeft = Math.min(Math.max(finalLeft, 6), 94); 
        needle.style.left = finalLeft + "%";
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
            if (needle) {
                needle.style.opacity = "1";
                needle.style.left = "1%";
            }
            return; 
        } else {
            display.innerText = numVal;
            if (needle) needle.style.opacity = "1";
        }
    }
    if (!needle) return;
    let safeVal = Math.max(numVal, 0.1); 
    let pct = ((Math.log(safeVal) - Math.log(0.1)) / (Math.log(50000) - Math.log(0.1))) * 88;
    let invertedPct = 88 - pct;
    let finalLeft = 6 + invertedPct;
    finalLeft = Math.min(Math.max(finalLeft, 6), 94);
    needle.style.left = finalLeft + "%";
}

function triggerOta(type) {
    const label = type === 'firmware' ? 'the firmware (C++)' : 'the filesystem (UI)';
    if (!confirm(`Do you want to update ${label} wirelessly from GitHub Cloud?`)) return;

    const btnFw = document.getElementById('btn-ota-fw');
    const btnFs = document.getElementById('btn-ota-fs');
    const status = document.getElementById('ota-status');

    btnFw.disabled = true; btnFs.disabled = true;
    btnFw.style.opacity = "0.4"; btnFs.style.opacity = "0.4";
    
    status.innerText = `Requesting ${type} from GitHub Pages...`;
    status.style.color = "var(--bertone-gold)";

    const baseUrl = "https://fhiel.github.io/Vehicle-Control-Unit-VCU-Electric-Vehicle/";
    const fileName = type === 'firmware' ? 'firmware.bin' : 'littlefs.bin';
    const fileUrl = `${baseUrl}${fileName}`;

    let bodyData = `url=${encodeURIComponent(fileUrl)}`;
    if (type === 'filesystem') {
        bodyData += `&mode=fs`;
    }

    fetch('/update', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/x-www-form-urlencoded',
        },
        body: bodyData
    })
    .then(response => {
        if (response.ok) {
            status.innerText = `Success! VCU is downloading ${type} and rebooting...`;
            status.style.color = "#3ddb67";
            setTimeout(() => { window.location.reload(); }, 6000);
        } else {
            throw new Error('VCU rejected the OTA command.');
        }
    })
    .catch(error => {
        console.error('OTA Error:', error);
        status.innerText = `Failed: Cloud file missing or VCU timeout!`;
        status.style.color = "var(--warning-red)";
        btnFw.disabled = false; btnFs.disabled = false;
        btnFw.style.opacity = "1"; btnFs.style.opacity = "1";
    });
}

// =====================================================================
// MAIN WEBSOCKET FRAMES INTERCEPTOR ROUTINE
// =====================================================================
ws.onmessage = function(e) {
    if (!e.data.startsWith('{')) {
        console.log("WebSocket text info received:", e.data);
        return; 
    }

    try {
        var d = JSON.parse(e.data);
        window.lastData = d;

        console.log("%c[DEBUG] JSON received - Top-Level Keys:", "color:cyan", Object.keys(d));

        updateRangeUI();

        // =====================================================================
        // DIRECT DECOUPLED COUPLING ENGINE (UPDATED TO REMOVE MAPPING CLASHES)
        // =====================================================================
        if (d.ovr) {
            // Binary Indicators & Relays 
            syncChannelUI(d.ovr.oil,   'switch-oil-auto',   'btn-oil-on',     'btn-oil-off',     null,         null,          'card-rel-1');
            syncChannelUI(d.ovr.bat,   'switch-bat-auto',   'btn-batled-on',  'btn-batled-off',  null,         null,          null);
            syncChannelUI(d.ovr.fan,   'switch-fan-auto',   'btn-fan-on',     'btn-fan-off',     null,         null,          'card-rel-3');
            // --- ISOLATED ACOUSTIC HORN OVERRIDES ---
            syncChannelUI(d.ovr.buzz,  'switch-piezo-auto', 'btn-piezo-on',   'btn-piezo-off',   null,         null,          null);
            // --- ISOLATED VISUAL ALARM WATCHDOG (Coupled cleanly onto Main Dashboard Card 2!) ---
            syncChannelUI(d.ovr.alarm, 'switch-alarm-auto', 'btn-alarm-on',   'btn-alarm-off',   null,         null,          'card-rel-2');

            // Multi-Stage PWM Coolant Pumps
            syncChannelUI(d.ovr.invp,  'switch-invp-auto',  null,             'btn-inv-off',     'btn-inv-20', 'btn-inv-80',  'card-rel-4');
            syncChannelUI(d.ovr.batp,  'switch-batp-auto',  null,             'btn-bat-off',     'btn-bat-20', 'btn-bat-80',  null);

            // Hut V3 Expansion Board Aux Channels
            syncChannelUI(d.ovr.r11,   'switch-rel11-auto', 'btn-rel11-on',   'btn-rel11-off',   null,         null,          null);
            syncChannelUI(d.ovr.r12,   'switch-rel12-auto', 'btn-rel12-on',   'btn-rel12-off',   null,         null,          null);
            syncChannelUI(d.ovr.r13,   'switch-rel13-auto', 'btn-rel13-on',   'btn-rel13-off',   null,         null,          null);
            syncChannelUI(d.ovr.r14,   'switch-rel14-auto', 'btn-rel14-on',   'btn-rel14-off',   null,         null,          null);
        }

        // =====================================================================
        // MCU
        // =====================================================================
        if (d.mcu) {
            setGauge('m-temp', d.mcu.mt, 50, 150);
            setGauge('i-temp', d.mcu.it, 0, 95);
            setSafeText('mcu_rpm', check(d.mcu.rpm, d.mcu.rpmV));
            setSafeText('mcu_trq', d.mcu.trq !== undefined ? d.mcu.trq.toFixed(1) : "n/a");
            setSafeText('mcu_mT_exp', check(d.mcu.mt, d.mcu.mtV));
            setSafeText('mcu_cT', check(d.mcu.it, d.mcu.itV));
            setSafeText('mcu_flt', check(d.mcu.flt, d.mcu.fltV));
            
            const trqEl = document.getElementById("mcu_trq");
            if (trqEl && d.mcu.trq !== undefined) {
                trqEl.style.color = d.mcu.trq < 0 ? "var(--instrument-green)" : "";
            }

            if (d.mcu.trq !== undefined) {
                const needle = document.getElementById('mcu-trq-needle');
                if (needle) {
                    let minScale = -20.0;
                    let maxScale = 100.0;
                    let pct = ((d.mcu.trq - minScale) / (maxScale - minScale)) * 88;
                    let finalLeft = 6 + pct;
                    finalLeft = Math.min(Math.max(finalLeft, 6), 94);
                    needle.style.left = finalLeft + "%";
                }
            }
        }

        // =====================================================================
        // IMD + BMS + PROXY + VCU STATUS
        // =====================================================================
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
            setSafeText('bms_cur',  d.bms.a !== undefined ? d.bms.a.toFixed(1) : "0.0");
            setSafeText('imd_hv1',  d.bms.v);
            
            const curEl = document.getElementById("bms_cur");
            if (curEl && d.bms.a !== undefined) {
                curEl.style.color = d.bms.a < -0.5 ? "var(--instrument-green)" : "";
            }

            if (d.bms.a !== undefined) {
                const needle = document.getElementById('bms-cur-needle');
                if (needle) {
                    let minScale = -40.0;
                    let maxScale = 200.0;
                    let pct = ((d.bms.a - minScale) / (maxScale - minScale)) * 88; 
                    let finalLeft = 6 + pct;
                    finalLeft = Math.min(Math.max(finalLeft, 6), 94);
                    needle.style.left = finalLeft + "%";
                }
            }
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

            setSafeText('vcu_uptime', d.vcu.ontm !== undefined ? d.vcu.ontm : "0");
        }

        // =====================================================================
        // HARDWARE STATUS (Absturzsicher & rein informativ gemappt)
        // =====================================================================
        if (d.hw) {
            // --- NATIVE FAN INJECTION FOR AUTOMATION LOOP ---
            const c3 = document.getElementById('card-rel-3');
            if (c3 && d.ovr && d.ovr.fan.m === 0) {
                // When Fan is in AUTO mode, animate home-tile purely off physical relay state
                if (d.hw.fan_relay) c3.classList.add('card-active-fan');
                else                c3.classList.remove('card-active-fan');
            }

            // Standard status label
            setSafeText('hw_ledoil',    d.hw.led_oil ? "ON" : "OFF");
            setSafeText('hw_ledbat',    d.hw.led_battery ? "ON" : "OFF");
            setSafeText('hw_PIEZO',     d.hw.piezo ? "ON" : "OFF");
            setSafeText('hw_piezo_act', d.hw.is_alarm ? "ALARM ACTIVE" : "NOMINAL");
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
        }

        // =====================================================================
        // KINETIC PROFILES PANEL INTERLOCK & LABOR LIVE SWITCH
        // =====================================================================
        let isLaborDemoActive = false;

        // 1. Lese das dedizierte Demo-Flag aus Group 4 aus (C++: vcu["demo_active"])
        if (d.vcu && d.vcu.hasOwnProperty('demo_active')) {
            isLaborDemoActive = (d.vcu.demo_active === true);
            
            // Experten-Schalter auf der Rückseite synchron halten
            const swDemo = document.getElementById('switch-demo-mode');
            const txtDemo = document.getElementById('txt-demo-status');
            if (swDemo) swDemo.checked = isLaborDemoActive;
            if (txtDemo) {
                txtDemo.innerText = isLaborDemoActive ? "TEST MAX" : "OFF";
                txtDemo.style.color = isLaborDemoActive ? "var(--instrument-green)" : "#888";
            }
        }

        // Viewport Visibility Interlock: Toggles profiles vs kinetic tracking layout
        if (d.bms && d.bms.st !== undefined) {
            const profileManager = document.getElementById("ui-profile-manager");
            const kineticGauges  = document.getElementById("ui-kinetic-gauges");
            
            // Render kinetic dials if BMS is in DRIVE (2) or CHARGE (3) mode
            if (d.bms.st === 2 || d.bms.st === 3) {
                if (profileManager) profileManager.style.display = "none";
                if (kineticGauges)  kineticGauges.style.display = "grid"; // Swapped to grid for horizontal pairing
            } else {
                if (profileManager) profileManager.style.display = "flex";
                if (kineticGauges)  kineticGauges.style.display = "none";
            }
        }

        // =====================================================================
        // MATHEMATICAL NEEDLE TRACKING & DATA SANITIZATION
        // =====================================================================
        const curEl = document.getElementById("live-bms-cur");
        const trqEl = document.getElementById("live-mcu-trq");
        const curNeedle = document.getElementById("current-needle");
        const trqNeedle = document.getElementById("torque-needle");

        // --- A. CURRENT GAUGE (Spanne: -250 bis +400 = 650A) ---
        let rawCurrent = (d.bms && d.bms.current !== undefined) ? d.bms.current : null;
        
        if (isLaborDemoActive) rawCurrent = 400; // Im Testmodus auf Anschlag jagen

        if (rawCurrent === null) {
            if (curEl) curEl.innerText = "n/a";
            if (curNeedle) curNeedle.style.left = "-5%"; // Sicher ausblenden vor der Skala!
        } else {
            if (curEl) curEl.innerText = Math.round(rawCurrent);
            if (curNeedle) {
                // Begrenzen auf Skalen-Eckwerte
                let clampedCur = Math.max(-250, Math.min(400, rawCurrent));
                // Prozentualer Anteil am Messbereich ermitteln
                let pct = (clampedCur - (-250)) / 650;
                // Skalieren auf den physischen Viewport (6% bis 94% = 88% Breite)
                let leftPos = 6 + (pct * 88);
                curNeedle.style.left = leftPos + "%";
            }
        }

        // --- B. TORQUE GAUGE (Spanne: -100% bis +100% = 200% Spanne) ---
        let rawTorque = (d.mcu && d.mcu.torque !== undefined) ? d.mcu.torque : null;
        
        if (isLaborDemoActive) rawTorque = 100; // Im Testmodus auf Anschlag jagen

        if (rawTorque === null) {
            if (trqEl) trqEl.innerText = "n/a";
            if (trqNeedle) trqNeedle.style.left = "-5%"; // Sicher ausblenden vor der Skala!
        } else {
            if (trqEl) trqEl.innerText = Math.round(rawTorque);
            if (trqNeedle) {
                let clampedTrq = Math.max(-100, Math.min(100, rawTorque));
                let pct = (clampedTrq - (-100)) / 200;
                let leftPos = 6 + (pct * 88);
                trqNeedle.style.left = leftPos + "%";
            }
        }
        // =====================================================================
        // TESLA-STYLE WS2812 PORT LOCK STATE DISPLAY MATRIX
        // =====================================================================
        const wsEl = document.getElementById('ws2812_status');
        if (wsEl && d.hw) {
            const stateId = d.hw.ws2812;
            
            if (stateId === 1) {
                wsEl.innerText = "GREEN (DRIVE READY / CHARGING)";
                wsEl.style.color = "var(--instrument-green)";
            } else if (stateId === 2) {
                wsEl.innerText = "RED (IMD FAULT / LOCK ERROR)";
                wsEl.style.color = "var(--tesla-red)";
            } else if (stateId === 3) {
                wsEl.innerText = "CYAN (CHARGER COOLDOWN RAMP-DOWN)";
                wsEl.style.color = "#00ffff";
            } else if (stateId === 4) {
                wsEl.innerText = "YELLOW (ISO SELFTEST RUNNING)";
                wsEl.style.color = "var(--bertone-gold)";
            } else if (stateId === 5) {
                wsEl.innerText = "BLUE (DAILY LIMIT REACHED)";
                wsEl.style.color = "#3897ff";
            } else {
                wsEl.innerText = "AMBER (STANDBY / SYSTEM IDLE)";
                wsEl.style.color = "#ffaa00";
            }
        }

    } catch (err) {
        console.error("WebSocket data parsing exception caught:", err);
    }
};                     

ws.onclose = function() { setTimeout(() => location.reload(), 2000); };