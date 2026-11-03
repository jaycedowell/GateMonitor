#include <time.h>
#include <Adafruit_ADS1X15.h>

#include "WiFi_private.h"
/* WiFi_private.h includes:
    * ssid, pass - WiFi setup
    * staticIP, subnet - static IP address and network submask
    * gateway - gateway address
    * primaryDNS, secondaryDNS - DNS servers
*/

#include "Auth_private.h"
/* Auth_private.h includes:
    * auth_username
    * auth_password
*/

#include "Icon.h"
/* Icon.h includes:
    * icon_raw
*/


#include "SimpleHome/src/SimpleHome.h"
#include "SimpleHome/src/SimpleHome.cpp"

// Pin assignments - ESP32
#define PIN_SDA        D4
#define PIN_SCL        D5
#define PIN_LED_RED    D1 
#define PIN_LED_GREEN  D2
#define PIN_LED_BLUE   D3
#define PIN_NTC_EXC    D0
#define PIN_EXIT       D10

// Pin assignements - ADC
#define VBAT_ADC_PIN   0
#define VBAT_DIV       ((100.0f + 27.0f) / 27.0f)

#define VSOL_ADC_PIN   1
#define VSOL_DIV       ((100.0f + 15.0f) / 15.0f)

#define EXC_ADC_PIN    3
#define NTC_ADC_PIN    2
#define NTC_R_FIXED    10000.0f
#define NTC_R25        10000.0f
#define NTC_BETA       3950.0f

#define SUN_MARGIN_V        0.30f   /* Vsol > Vbat + margin => sun */
#define TEMPCO_V_PER_C      0.024f  /* 12 V SLA, -4 mV/C/cell */
#define FULL_V_25C          13.80f  /* held this long under sun => full */
#define OVERCHARGE_V_25C    14.80f  /* status-page warning only */

#define REST_DARK_MS        (2LL * 60 * 60 * 1000)  /* dark this long ... */
#define REST_FLAT_V         0.010f                  /* ... and flat over window */
#define REST_MAX_AGE_MS     (36LL * 60 * 60 * 1000)

#define TIME_FACTOR  1

Adafruit_ADS1115 ads1115;

volatile int gate_state = 0;
volatile int led_state = 0;

SimpleHome sh("GateMonitor", icon_raw, sizeof(icon_raw));

float read_battery_voltage() {
  int16_t adc = ads1115.readADC_SingleEnded(VBAT_ADC_PIN);
  float voltage = ads1115.computeVolts(adc);
  return voltage * VBAT_DIV;
}

float read_solar_voltage() {
  int16_t adc = ads1115.readADC_SingleEnded(VSOL_ADC_PIN);
  float voltage = ads1115.computeVolts(adc);
  return voltage * VSOL_DIV;
}

float read_battery_temperature() {
  digitalWrite(PIN_NTC_EXC, HIGH);
  delay(10);

  int16_t adc_exc, adc_ntc;
  adc_exc = ads1115.readADC_SingleEnded(EXC_ADC_PIN);
  adc_ntc = ads1115.readADC_SingleEnded(NTC_ADC_PIN);
  digitalWrite(PIN_NTC_EXC, LOW);

  float v_top, v_mid;
  v_top = ads1115.computeVolts(adc_exc);
  v_mid = ads1115.computeVolts(adc_ntc);
  float d = v_top - v_mid;
  if (v_top < 2.5f || d < 0.005f || v_mid < 0.005f) {
    return NAN;
  }

  float r   = NTC_R_FIXED * v_mid / d;
  float inv = 1.0f / 298.15f + logf(r / NTC_R25) / NTC_BETA;
  return 1.0f / inv - 273.15f;
}

typedef struct _ComputedState {
  float soc;
  bool soc_measured;
  bool in_sun;
  bool is_charging;
  float full_threshold;
  float over_threshold;
  unsigned long time;
} ComputedState;

typedef struct _MeasurementWindow {
  int count;
  float vbat[20];
  float vsol[20];
  float tbat[20];
  unsigned long time[20];
} MeasurementWindow;

typedef struct _HourlyWindow {
  int count;
  float vbat_min[6];
  float vbat_max[6];
  float vsol_min[6];
  float vsol_max[6];
  float tbat_min[6];
  float tbat_max[6];
  unsigned long time[6];
} HourlyWindow;

typedef struct _DailyWindow {
  int count;
  float vbat_min[24];
  float vbat_max[24];
  float vsol_min[24];
  float vsol_max[24];
  float tbat_min[24];
  float tbat_max[24];
  unsigned long time[24];
} DailyWindow;

ComputedState State = {0};
MeasurementWindow Last10Min = {0};
HourlyWindow LastHour = {0};
DailyWindow LastDay = {0};

void record_measurements() {
  float vbat, vsol, tbat;
  vbat = read_battery_voltage();
  vsol = read_solar_voltage();
  tbat = read_battery_temperature();
  Serial.print("Vbat: ");
  Serial.print(vbat);
  Serial.print("V, Vsol: ");
  Serial.print(vsol);
  Serial.print("V, Tbat: ");
  Serial.print(tbat);
  Serial.println("C");

  int idx = Last10Min.count % 20;
  Last10Min.vbat[idx] = vbat;
  Last10Min.vsol[idx] = vsol;
  Last10Min.tbat[idx] = tbat;
  Last10Min.time[idx] = time(nullptr);
  Last10Min.count++;
  Last10Min.count %= 20;
  Serial.print("Updated 10 minute history, count is now ");
  Serial.println(Last10Min.count);
  
  if( (Last10Min.count % 20) == 0 ) {
    idx = LastHour.count;
    LastHour.vbat_min[idx] = LastHour.vbat_max[idx] = Last10Min.vbat[0];
    LastHour.vsol_min[idx] = LastHour.vsol_max[idx] = Last10Min.vsol[0];
    LastHour.tbat_min[idx] = LastHour.tbat_max[idx] = Last10Min.tbat[0];
    LastHour.time[idx] = Last10Min.time[0];
    for(int i=1; i<20; i++) {
      LastHour.vbat_min[idx] = fminf(LastHour.vbat_min[idx], Last10Min.vbat[i]);
      LastHour.vbat_max[idx] = fmaxf(LastHour.vbat_max[idx], Last10Min.vbat[i]);
      LastHour.vsol_min[idx] = fminf(LastHour.vsol_min[idx], Last10Min.vsol[i]);
      LastHour.vsol_max[idx] = fmaxf(LastHour.vsol_max[idx], Last10Min.vsol[i]);
      LastHour.tbat_min[idx] = fminf(LastHour.tbat_min[idx], Last10Min.tbat[i]);
      LastHour.tbat_max[idx] = fmaxf(LastHour.tbat_max[idx], Last10Min.tbat[i]);
    }
    LastHour.count++;
    LastHour.count %= 6;
    Serial.print("Updated hour history, count is now ");
    Serial.println(LastHour.count);

    if( (LastHour.count % 6) == 0 ) {
      idx = LastDay.count;
      LastDay.vbat_min[idx] = LastHour.vbat_min[0];
      LastDay.vbat_max[idx] = LastHour.vbat_max[0];
      LastDay.vsol_min[idx] = LastHour.vsol_min[0];
      LastDay.vsol_max[idx] = LastHour.vsol_max[0];
      LastDay.tbat_min[idx] = LastHour.tbat_min[0];
      LastDay.tbat_max[idx] = LastHour.tbat_max[0];
      LastDay.time[idx] = LastHour.time[0];
      for(int i=1; i<6; i++) {
        LastDay.vbat_min[idx] = fminf(LastDay.vbat_min[idx], LastHour.vbat_min[i]);
        LastDay.vbat_max[idx] = fmaxf(LastDay.vbat_max[idx], LastHour.vbat_max[i]);
        LastDay.vsol_min[idx] = fminf(LastDay.vsol_min[idx], LastHour.vsol_min[i]);
        LastDay.vsol_max[idx] = fmaxf(LastDay.vsol_max[idx], LastHour.vsol_max[i]);
        LastDay.tbat_min[idx] = fminf(LastDay.tbat_min[idx], LastHour.tbat_min[i]);
        LastDay.tbat_max[idx] = fmaxf(LastDay.tbat_max[idx], LastHour.tbat_max[i]);
      }
      LastDay.count++;
      LastDay.count %= 24;
      Serial.print("Updated day history, count is now ");
      Serial.println(LastDay.count);
    }
  }
}

float get_estimated_soc() {
  float vbat = NAN;
  if( LastHour.count == 0 ) {
    vbat = Last10Min.vbat[(Last10Min.count+20-1) % 20];
  } else {
    vbat = LastHour.vbat_min[(LastHour.count+6-1) % 6];
  }

  return sla_estimate_soc(vbat);
}

float get_measured_soc() {
  float vbat = NAN;

  int idx0 = (LastDay.count + 24 - 1) % 24;
  int idx1 = (LastDay.count + 24 - 2) % 24;
  if( (LastDay.time[idx0] != 0) && (LastDay.time[idx1] != 0) ) {
    // If we have two valid hours....
    float vbat_min = fminf(LastDay.vbat_min[idx0], LastDay.vbat_min[idx1]);
    float vbat_max = fmaxf(LastDay.vbat_max[idx0], LastDay.vbat_max[idx1]);

    if( (vbat_max - vbat_min) < 0.01 ) {
      // If the battery voltage has been flat over those two hours...
      float vsol_max = fmaxf(LastDay.vsol_max[idx0], LastDay.vsol_max[idx1]);

      if( vsol_max < (vbat_min + 0.3) ) {
        // If we haven't charged the battery in those two hours...
        vbat = LastDay.vbat_min[idx0];
      }
    }
  }

  return sla_estimate_soc(vbat);
}

void update_current_state() {
  int idx = (Last10Min.count + 20 - 1) % 20;
  float vbat = Last10Min.vbat[idx];
  float vsol = Last10Min.vsol[idx];
  float tbat = Last10Min.tbat[idx];
  if( tbat != tbat ) {
    tbat = 25.0;
  }
  State.time = Last10Min.time[idx];

  State.in_sun = (vsol > (vbat + 0.3));
  State.full_threshold = sla_estimate_full_volts(tbat);
  State.over_threshold = sla_estimate_over_volts(tbat);

  State.is_charging = false;  
  if( State.in_sun ) {
    State.is_charging = true;
    if( vbat >= State.full_threshold ) {
      State.is_charging = false;
    }
  }

  State.soc = get_measured_soc();
  State.soc_measured = true;
  if( State.soc != State.soc ) {
    State.soc = get_estimated_soc();
    State.soc_measured = false;
  }
}

void handle_history_json() {
    String body = String("{\n");
    body += " \"updated\": " +  String(State.time) + ",\n";
    body += " \"history\": [\n";
    for(int i=0; i<24; i++) {
        int idx = (LastDay.count - 24 + i) % 24;
        if( LastDay.time[idx] == 0 ) {
          continue;
        }
        body += "  {\n";
        body += "   \"time\": " + String(LastDay.time[idx]) + ",\n";
        body += "   \"vbat_min\": " + String(LastDay.vbat_min[idx]) + ",\n";
        body += "   \"vbat_max\": " + String(LastDay.vbat_max[idx]) + ",\n";
        body += "   \"vsol_min\": " + String(LastDay.vsol_min[idx]) + ",\n";
        body += "   \"vsol_max\": " + String(LastDay.vsol_max[idx]) + ",\n";
        body += "   \"temp_bat_min\": " + String(LastDay.tbat_min[idx]) + ",\n";
        body += "   \"temp_bat_max\": " + String(LastDay.tbat_max[idx]) + ",\n";
        body += "   \"vbat_full\": " + String(sla_estimate_full_volts(LastDay.tbat_max[idx])) + ",\n";
        body += "   \"vbat_over\": " + String(sla_estimate_over_volts(LastDay.tbat_max[idx])) + ",\n";
        body += "  }";
        if( i != 23 ) {
            body += ",\n";
        } else {
            body += "\n";
        }
  }
  body += "  ]\n";
  body += "}\n";
  
  sh.server.send(200, "application/json", body);
}

void handle_root() {
    String body = String(PAGE_HEAD);
    body.replace("%TITLE%", sh.name());
    
    int idx = (Last10Min.count + 20 - 1) % 20;
    body += "Battery Voltage: " + String(Last10Min.vbat[idx]) + " V<br>\n";
    body += "Battery Temperature: " + String(temp_c_to_f(Last10Min.tbat[idx])) + " F<br>\n";
    body += "Solar Voltage: " + String(Last10Min.vsol[idx]) + " V<br>\n";
    body += "Battery is charging? " + String(State.is_charging ? "yes" : "no") + "<br>\n";
    body += "Battery SoC: " + String(State.soc) + "%" + String(State.soc_measured ? "" : " (estimated)") + "<br>\n";
    body += "<br>\n";
    
    float vbat_min = 1000, vbat_max = -1000, tbat_min = 1000, tbat_max = -1000;
    for(int i=0; i<20; i++) {
        // 30 s chunks within the last 10 minutes
        if( Last10Min.time[i] == 0 ) {
            continue;
        }
        vbat_min = fminf(vbat_min, Last10Min.vbat[i]);
        vbat_max = fmaxf(vbat_max, Last10Min.vbat[i]);
    }
    for(int i=0; i<6; i++) {
        // 10 min chunks within the last hour
        if( LastHour.time[i] == 0 ) {
            continue;
        }
        vbat_min = fminf(vbat_min, LastHour.vbat_min[i]);
        vbat_max = fmaxf(vbat_max, LastHour.vbat_max[i]);
        tbat_min = fminf(tbat_min, LastHour.tbat_min[i]);
        tbat_max = fmaxf(tbat_max, LastHour.tbat_max[i]);
    }
    for(int i=0; i<24; i++) {
        // 1 hr chunks within the last day
        if( LastDay.time[i] == 0 ) {
            continue;
        }
        vbat_min = fminf(vbat_min, LastDay.vbat_min[i]);
        vbat_max = fmaxf(vbat_max, LastDay.vbat_max[i]);
        tbat_min = fminf(tbat_min, LastDay.tbat_min[i]);
        tbat_max = fmaxf(tbat_max, LastDay.tbat_max[i]);
    }
    if( vbat_max >= 0 ) {
        body += "24 hr Ranges:<br>\n";
        body += "  Battery Voltage: " + String(vbat_min) + " to " + String(vbat_max) + " V<br>\n";
        body += "  Battery Temperature: " + String(temp_c_to_f(tbat_min)) + " to " + String(temp_c_to_f(tbat_max)) + " F<br>\n";
        body += "<br>\n";
    }
    body += "<br>\n";
    
    body += "Gate: ";
    switch(gate_state) {
        case 3: // fall through
        case 2: body += "hold open"; break;
        case 1: body += "cycle open"; break;
        case 0: body += "cycle close"; break;
        default: body += "unknown";
    }
    body += "<br>\n";
    
    body += "Updated: " + format_now("%Y/%m/%d %H:%M:%S") + "<br>\n";
    body += String(PAGE_FOOT);
    
    sh.server.send(200, "text/html", body);
}

void handle_root_json() {
    int idx = (Last10Min.count + 20 - 1) % 20;
    
    String body = String("{\n");
    body += " \"vbat\": " + String(Last10Min.vbat[idx]) + ",\n";
    body += " \"temp_bat\": " + String(Last10Min.tbat[idx]) + ",\n";
    body += " \"vsolar\": " + String(Last10Min.vsol[idx]) + ",\n";
    body += " \"battery_charging\": " + String(State.is_charging ? "true" : "false") + ",\n";
    body += " \"battery_soc\": " + String(State.soc) + ",\n";
    body += " \"battery_soc_type\": \"" + String(State.soc_measured ? "measured" : " (estimated)") + "\",\n";
    body += " \"gate\": \"";
    switch(gate_state) {
        case 3: // fall through
        case 2: body += "hold open"; break;
        case 1: body += "cycle open"; break;
        case 0: body += "cycle close"; break;
        default: body += "unknown";
    }
    body += "\",\n";
    body += " \"updated\": \"" + format_now("%Y/%m/%d %H:%M:%S") + "\"\n";
    body += "}\n";
    
    sh.server.send(200, "application/json", body);
}

void handle_open() {
    if( sh.server.method() != HTTP_POST ) {
        // GET (or anything else): show the confirmation button
        String body = String(PAGE_HEAD);
        body += "<form method=\"post\" action=\"/open\">\n";
        body += " <button type=\"submit\">Open Gate</button>\n";
        body += "</form>\n";
        body += String(PAGE_FOOT);
        body.replace("%TITLE%", sh.name());
        
        sh.server.send(200, "text/html", body);
    } else {
        if( !sh.server.authenticate(auth_username, auth_password)) {
            return sh.server.requestAuthentication(BASIC_AUTH, sh.name());
        }
        
        gate_state = (gate_state > 1 ? gate_state : 1);
        
        String body = String(PAGE_HEAD);
        body.replace("%TITLE%", sh.name());
        body += "Gate: cycle open<br>\n";
        body += String(PAGE_FOOT);
        
        sh.server.send(200, "text/html", body);
    }
}

void handle_hold() {
    if( sh.server.method() != HTTP_POST ) {
        // GET (or anything else): show the confirmation button
        String body = String(PAGE_HEAD);
        body += "<form method=\"post\" action=\"/hold\">\n";
        body += " <button type=\"submit\">Restart %TITLE%</button>\n";
        body += "</form>\n";
        body += String(PAGE_FOOT);
        body.replace("%TITLE%", sh.name());
        
        sh.server.send(200, "text/html", body);
    } else {
        if( !sh.server.authenticate(auth_username, auth_password)) {
            return sh.server.requestAuthentication(BASIC_AUTH, sh.name());
        }
        
        gate_state = (gate_state != 3 ? 3 : 0);
        
        String body = String(PAGE_HEAD);
        body.replace("%TITLE%", sh.name());
        body += "Gate: " + String(gate_state > 0 ? "hold open" : "cycle close") + "<br>\n";
        body += String(PAGE_FOOT);
        
        sh.server.send(200, "text/html", body);
    }
}

void process_measurements() {
    record_measurements();
    update_current_state();
}

volatile uint64_t _last_hold_pulse = 0;

void process_gate_state() {
    uint64_t now = now_millis();
    
    if( gate_state == 3 ) {
        if( (now - _last_hold_pulse) > 30000 ) {
            digitalWrite(PIN_EXIT, HIGH);
            Serial.println("pulse exit - hold");
            gate_state = 2;
            _last_hold_pulse = now;
        }
    } else if( gate_state == 2 ) {
        digitalWrite(PIN_EXIT, LOW);
        Serial.println("pulse exit - cycle");
        gate_state = 3;
    } else if( gate_state == 1 ) {
        digitalWrite(PIN_EXIT, HIGH);
        Serial.println("pulse exit - open");
        gate_state = 0;
    } else {
        digitalWrite(PIN_EXIT, LOW);
        Serial.println("pulse exit - cycle");
    }
}

volatile uint64_t _last_led_flash = 0;

void process_led_state() {
    uint64_t now = now_millis();
    bool any_flashed = false;
    
    if( State.is_charging ) {
        if( (led_state & 1) ) {
            if( !any_flashed && ((now - _last_led_flash) > 4000) ) {
                digitalWrite(PIN_LED_GREEN, HIGH);
                led_state &= 2;
                any_flashed = true;
            }
        }
    } else {
        digitalWrite(PIN_LED_GREEN, LOW);
        led_state != 1;
    }
    
    if( State.soc < 30 ) {
        if( (led_state >> 1) & 1 ) {
            if( !any_flashed && ((now - _last_led_flash) > 4000) ) {
                digitalWrite(PIN_LED_RED, HIGH);
                led_state &= 1;
                any_flashed = true;
            }
        }
    } else {
        digitalWrite(PIN_LED_RED, LOW);
        led_state |= 2;
    }
    
    if( any_flashed ) {
        _last_led_flash = now;
    }
}

void setup() {
  // Drop the CPU clock to 80 MHz (the minimum that still supports WiFi)
  setCpuFrequencyMhz(80);
  
  // Set the pins for the LEDs and other output
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_BLUE, OUTPUT);
  pinMode(PIN_NTC_EXC, OUTPUT);
  pinMode(PIN_EXIT, OUTPUT);
  
  digitalWrite(PIN_LED_RED, LOW);
  digitalWrite(PIN_LED_GREEN, LOW);
  digitalWrite(PIN_LED_BLUE, LOW);
  digitalWrite(PIN_NTC_EXC, LOW);
  digitalWrite(PIN_EXIT, LOW);

  // Start the ADC
  Wire.begin(PIN_SDA, PIN_SCL);
  ads1115.begin(0x48);
  ads1115.setGain(GAIN_ONE); 
  delay(10);
  
  // Setup Simple Home
  sh.set_wifi(ssid, pass);
  sh.set_static_ip(staticIP, gateway, subnet, primaryDNS, secondaryDNS);
  sh.set_auth(auth_username, auth_password);
  
  sh.begin();
  sh.server.on("/", handle_root);
  sh.server.on("/data.json", handle_root_json);
  sh.server.on("/history.json", handle_history_json);
  sh.server.on("/open", handle_open);
  sh.server.on("/hold", handle_hold);
  
  sh.every(30000, process_measurements);
  sh.every(1000, process_gate_state);
  sh.every(50, process_led_state);
}

void loop() {
    sh.loop();
}
