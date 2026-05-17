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
            if (needle) {
                needle.style.opacity = "1"; // Schön dezent im Fehlerfall
                needle.style.left = "1%";     /* Der "Veglia-Ruheanschlag" ganz links außerhalb! */
            }
            return;
        } else {
            display.innerText = numVal;
            if (needle) needle.style.opacity = "1";
        }
    }
    if (needle) {
        // Berechnet die Prozent innerhalb der 6% bis 94% Skala
        let pct = ((numVal - min) / (max - min)) * 88; // 88% ist der nutzbare Bereich (100 - 6 - 6)
        let finalLeft = 6 + pct; // Versatz um das linke Padding
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
                needle.style.left = "1%";     /* Auch hier: Abseits der Skala parken */
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
    if (!confirm(`Do you want to update ${label} wirelessly?`)) return;

    const btnFw = document.getElementById('btn-ota-fw');
    const btnFs = document.getElementById('btn-ota-fs');
    const status = document.getElementById('ota-status');

    // Lock both buttons during upload
    btnFw.disabled = true; btnFs.disabled = true;
    btnFw.style.opacity = "0.4"; btnFs.style.opacity = "0.4";
    
    status.innerText = `Sending download path for ${type}...`;
    status.style.color = "var(--bertone-gold)";

    // IP of your development PC in your home/garage network
    const pcIp = "192.168.188.57"; 
    
    const fileName = type === 'firmware' ? 'firmware.bin' : 'littlefs.bin';
    const fileUrl = `http://${pcIp}:8080/${fileName}`;

    // ElegantOTA v3 URL-Update payload
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
            status.innerText = `Success! VCU is flashing ${type} and rebooting...`;
            status.style.color = "#3ddb67";
            // Reload dashboard after 6 seconds
            setTimeout(() => { window.location.reload(); }, 6000);
        } else {
            throw new Error('VCU rejected the OTA download command.');
        }
    })
    .catch(error => {
        console.error('OTA Error:', error);
        status.innerText = `Failed: Server unreachable or binary missing!`;
        status.style.color = "var(--warning-red)";
        // Release buttons on error
        btnFw.disabled = false; btnFs.disabled = false;
        btnFw.style.opacity = "1"; btnFs.style.opacity = "1";
    });
}

ws.onmessage = function(e) {
    // FEHLERABFANG: Wenn es kein JSON ist, gar nicht erst parsen
    if (!e.data.startsWith('{')) {
        console.log("WebSocket Text-Info:", e.data);
        return; 
    }

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