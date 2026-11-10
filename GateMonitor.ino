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

ComputedState State = {0};

typedef struct _Measurement {
  float vbat;
  float vsol;
  float tbat;
  time_t time;
} Measurement;

typedef struct _Range {
  float vbat_min, vbat_max;
  float vsol_min, vsol_max;
  float tbat_min, tbat_max;
  time_t time;
} Range;

RingBuffer<Measurement,20> Last10Min;
RingBuffer<Range,6> LastHour;
RingBuffer<Range,24> LastDay;

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

  Measurement latest;
  latest.vbat = vbat;
  latest.vsol = vsol;
  latest.tbat = tbat;
  latest.time = time(nullptr);
  
  if( Last10Min.push(latest) ) {
    float vbat_min, vbat_max;
    float vsol_min, vsol_max;
    float tbat_min, tbat_max;
    vbat_min = vbat_max = NAN;
    vsol_min = vsol_max = NAN;
    tbat_min = tbat_max = NAN;
    for(const Measurement &entry : Last10Min) {
      vbat_min = fminf(vbat_min, entry.vbat);
      vbat_max = fmaxf(vbat_max, entry.vbat);
      vsol_min = fminf(vsol_min, entry.vsol);
      vsol_max = fmaxf(vsol_max, entry.vsol);
      tbat_min = fminf(tbat_min, entry.tbat);
      tbat_max = fmaxf(tbat_max, entry.tbat);
    }
    
    Range aggregate;
    aggregate.vbat_min = vbat_min;
    aggregate.vbat_max = vbat_max;
    aggregate.vsol_min = vsol_min;
    aggregate.vsol_max = vsol_max;
    aggregate.tbat_min = tbat_min;
    aggregate.tbat_max = tbat_max;
    aggregate.time = Last10Min[0].time;
    
    if( LastHour.push(aggregate) ) {
      vbat_min = vbat_max = NAN;
      vsol_min = vsol_max = NAN;
      tbat_min = tbat_max = NAN;
      for(const Range &entry : LastHour) {
        vbat_min = fminf(vbat_min, entry.vbat_min);
        vbat_max = fmaxf(vbat_max, entry.vbat_max);
        vsol_min = fminf(vsol_min, entry.vsol_min);
        vsol_max = fmaxf(vsol_max, entry.vsol_max);
        tbat_min = fminf(tbat_min, entry.tbat_min);
        tbat_max = fmaxf(tbat_max, entry.tbat_max);
      }
      
      aggregate.vbat_min = vbat_min;
      aggregate.vbat_max = vbat_max;
      aggregate.vsol_min = vsol_min;
      aggregate.vsol_max = vsol_max;
      aggregate.tbat_min = tbat_min;
      aggregate.tbat_max = tbat_max;
      aggregate.time = LastHour[0].time;
      
      LastDay.push(aggregate);
    }
  }
}

float get_estimated_soc() {
  float vbat = NAN;
  if( LastHour.empty() ) {
      const Measurement &latest = Last10Min.latest();
      vbat = latest.vbat;
  } else {
      const Range &aggregate = LastHour.latest();
      vbat = aggregate.vbat_min;
  }

  return sla_estimate_soc(vbat);
}

float get_measured_soc() {
  float vbat = NAN;

  if( LastDay.size() >= 2 ) {
    // If we have two valid hours....
    const Range &aggregate0 = LastDay[LastDay.size() - 1];
    const Range &aggregate1 = LastDay[LastDay.size() - 2];
    float vbat_min = fminf(aggregate0.vbat_min, aggregate1.vbat_min);
    float vbat_max = fmaxf(aggregate0.vbat_max, aggregate1.vbat_max);

    if( (vbat_max - vbat_min) < 0.01 ) {
      // If the battery voltage has been flat over those two hours...
      float vsol_max = fmaxf(aggregate0.vsol_max, aggregate1.vsol_max);

      if( vsol_max < (vbat_min + 0.3) ) {
        // If we haven't charged the battery in those two hours...
        vbat = aggregate0.vbat_min;
      }
    }
  }

  return sla_estimate_soc(vbat);
}

void update_current_state() {
  const Measurement &latest = Last10Min.latest();
  float vbat = latest.vbat;
  float vsol = latest.vsol;
  float tbat = latest.tbat;
  if( tbat != tbat ) {
    tbat = 25.0;
  }
  State.time = latest.time;

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
  JSONObject body;
  JSONArray history;
  for(const Range &aggregate : LastDay) {
    JSONObject entry;
    entry.add("time", aggregate.time);
    entry.add("vbat_min", aggregate.vbat_min);
    entry.add("vbat_max", aggregate.vbat_max);
    entry.add("vsol_min", aggregate.vsol_min);
    entry.add("vsol_max", aggregate.vsol_max);
    entry.add("temp_bat_min", aggregate.tbat_min);
    entry.add("temp_bat_max", aggregate.tbat_max);
    entry.add("vbat_full", sla_estimate_full_volts(aggregate.tbat_max));
    entry.add("vbat_over", sla_estimate_over_volts(aggregate.tbat_max));
    history.add(entry);
  }
  body.add("history", history);
  body.add("updated", format_epoch(State.time, "%Y/%m/%d %H:%M:%S"));
    
  sh.server.send(200, "application/json", body.render());
}

const char* decode_gate_state(uint8_t state) {
    switch(gate_state) {
        case 3: // fall through
        case 2: return "hold open";
        case 1: return "cycle open";
        case 0: return "cycle close";
        default: return "unknown";
    }
}

void handle_root() {
    const Measurement &latest = Last10Min.latest();
    
    TwoColumnTable table;
    table.add_row("Battery Voltage", latest.vbat, "V");
    table.add_row("Battery Temperature", temp_c_to_f(latest.tbat), "F");
    table.add_row("Solar Voltage", latest.vsol, "V"); 
    table.add_row("Battery is charging? ", State.is_charging);
    table.add_row("Battery SoC", State.soc, "%" + String(State.soc_measured ? "" : " (estimated)"));
    table.add_row("Gate", decode_gate_state(gate_state));
    table.add_row("Updated", format_now("%Y/%m/%d %H:%M:%S"));
    
    HTMLPage page = sh.get_base_html_page();
    page.add_content(table);
    
    float vbat_min, vbat_max;
    float tbat_min, tbat_max;
    vbat_min = vbat_max = NAN;
    tbat_min = tbat_max = NAN;
    for(const Measurement &entry : Last10Min) {
        vbat_min = fminf(vbat_min, entry.vbat);
        vbat_max = fmaxf(vbat_max, entry.vbat);
        tbat_min = fminf(tbat_min, entry.tbat);
        tbat_max = fmaxf(tbat_max, entry.tbat);
    }
    for(const Range &entry : LastHour) {
        vbat_min = fminf(vbat_min, entry.vbat_min);
        vbat_max = fmaxf(vbat_max, entry.vbat_max);
        tbat_min = fminf(tbat_min, entry.tbat_min);
        tbat_max = fmaxf(tbat_max, entry.tbat_max);
    }
    for(const Range &entry : LastDay) {
        vbat_min = fminf(vbat_min, entry.vbat_min);
        vbat_max = fmaxf(vbat_max, entry.vbat_max);
        tbat_min = fminf(tbat_min, entry.tbat_min);
        tbat_max = fmaxf(tbat_max, entry.tbat_max);
    }
    if( vbat_max == vbat_max ) {
        TwoColumnTable ranges;
        ranges.add_header("24 hr Ranges");
        ranges.add_row("Battery Voltage", String(vbat_min) + " to " + String(vbat_max) + " V");
        if( tbat_max == tbat_max ) {
            ranges.add_row("Battery Temperature", String(temp_c_to_f(tbat_min)) + " to " + String(temp_c_to_f(tbat_max)) + " F");
        }
        page.add_content("<br><br>\n");
        page.add_content(ranges);
    }
    
    sh.server.send(200, "text/html", page.render());
}

void handle_root_json() {
    const Measurement &latest = Last10Min.latest();
    
    JSONObject body;
    body.add("vbat", latest.vbat);
    body.add("temp_bat", latest.tbat);
    body.add("vsolar", latest.vsol);
    body.add("battery_charging", State.is_charging);
    body.add("battery_soc", State.soc);
    body.add("battery_soc_type", (State.soc_measured ? "measured" : "estimated"));
    body.add("gate", decode_gate_state(gate_state));
    body.add("updated", format_now("%Y/%m/%d %H:%M:%S"));
    
    sh.server.send(200, "application/json", body.render());
}

void handle_open() {
    if( sh.server.method() != HTTP_POST ) {
        // GET (or anything else): show the confirmation button
        String contents = String();
        contents += "<form method=\"post\" action=\"/open\">\n";
        contents += " <button type=\"submit\">Open Gate</button>\n";
        contents += "</form>\n";
        
        HTMLPage page = sh.get_base_html_page();
        page.add_content(contents);
        
        sh.server.send(200, "text/html", page.render());
    } else {
        if( !sh.server.authenticate(auth_username, auth_password)) {
            return sh.server.requestAuthentication(BASIC_AUTH, sh.name());
        }
        
        gate_state = (gate_state > 1 ? gate_state : 1);
        
        String contents = String();
        contents += "Gate: cycle open<br>\n";
        
        HTMLPage page = sh.get_base_html_page();
        page.add_content(contents);
        
        sh.server.send(200, "text/html", page.render());
    }
}

void handle_hold() {
    if( sh.server.method() != HTTP_POST ) {
        // GET (or anything else): show the confirmation button
        String contents = String();
        contents += "<form method=\"post\" action=\"/hold\">\n";
        contents += " <button type=\"submit\">Toggel Gate Hold State</button>\n";
        contents += "</form>\n";
        
        HTMLPage page = sh.get_base_html_page();
        page.add_content(contents);
        
        sh.server.send(200, "text/html", page.render());
    } else {
        if( !sh.server.authenticate(auth_username, auth_password)) {
            return sh.server.requestAuthentication(BASIC_AUTH, sh.name());
        }
        
        gate_state = (gate_state != 3 ? 3 : 0);
        
        String contents = String();
        contents += "Gate: " + String(gate_state > 0 ? "hold open" : "cycle close") + "<br>\n";
        
        HTMLPage page = sh.get_base_html_page();
        page.add_content(contents);
        
        sh.server.send(200, "text/html", page.render());
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
        led_state |= 1;
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
  
  // Preload the state
  process_measurements();
  
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
  
  sh.register_navigation_link("/", "Home");
  sh.register_navigation_link("/open", "Open");
  sh.register_navigation_link("/hold", "Hold");
  
  sh.every(30000, process_measurements);
  sh.every(1000, process_gate_state);
  sh.every(50, process_led_state);
}

void loop() {
    sh.loop();
}
