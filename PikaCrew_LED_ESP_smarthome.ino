
#define FASTLED_ESP32_I2S
#include <FastLED.h>
#include <type_traits>
#include "WiFi.h"
#include "AsyncUDP.h"
#include <PubSubClient.h>
#include <freertos/task.h>

// CONFIGURATION DEFINES

#define WIFI_SSID "YOUR_WIFI_NAME"
#define WIFI_PASS "YOUR_WIFI_PASS"

// Leave MQTT_HOST empty for the original automatic Art-Net behavior.
constexpr char MQTT_HOST[] = "YOUR_HUB_IP"; // Home Assistant / Mosquitto LAN address
constexpr uint16_t MQTT_PORT = 1883;
constexpr char MQTT_USER[] = "YOUR_HUB_ACCESS_USER";
constexpr char MQTT_PASSWORD[] = "HUB_ACCESS_PASSWORD";
constexpr bool MQTT_ENABLED = MQTT_HOST[0] != '\0';
// Use the SweetLight computer's LAN IP to restrict Show Control to that sender.
// 0.0.0.0 accepts any sender on the configured universe while Show Control is selected.
const IPAddress SHOW_CONTROLLER_IP(0, 0, 0, 0);
constexpr uint32_t SHOW_FRAME_TIMEOUT_MS = 1000;

// Set a display name and assign a different Art-Net port-address to each device.
// The hardware ID is derived from the factory MAC and survives a rename or DHCP change.
constexpr char DEVICE_NAME[] = "male-harness-v1";
constexpr uint16_t ARTNET_UNIVERSE = 0; // Wire value, 0 through 32767
constexpr uint16_t ARTNET_PORT = 6454;
constexpr uint16_t CONTROL_PORT = 6455;
static_assert(ARTNET_UNIVERSE <= 32767, "Invalid Art-Net port-address");

#define IS_MALE // comment out to make female

// only valid for males
#define NUM_LEDS_TOP 28
#define NUM_LEDS_BOTTOM 19
#define NUM_LEDS_STRAP 41

// only valid for females
#define NUM_LEDS_SHOULDER 25
#define NUM_LEDS_BRA_IN 11
#define NUM_LEDS_BRA_OUT 13
#define NUM_LEDS_TOP_RIB 65
#define NUM_LEDS_MIDDLE_RIB 58
#define NUM_LEDS_BOTTOM_RIB 68

// One row per output: slot alias, physical GPIO, male pixel count, female pixel count.
// Set the active layout's count to 0 for any unwired output.
// GPIO 1 and 3 are reserved for USB serial. Flash and input-only GPIOs are omitted.
// GPIO 0, 2, 5, 12 and 15 are boot strapping pins. Keep unused rows at 0.
#define LED_OUTPUTS(X) \
  X(0, 14, NUM_LEDS_STRAP , (NUM_LEDS_SHOULDER + NUM_LEDS_BRA_IN  * 2)) \
  X(1, 27, NUM_LEDS_TOP   , (NUM_LEDS_SHOULDER + NUM_LEDS_BRA_OUT * 2)) \
  X(2, 26, NUM_LEDS_TOP   , 0) \
  X(3, 25, NUM_LEDS_TOP   , (NUM_LEDS_SHOULDER + NUM_LEDS_BRA_OUT * 2)) \
  X(4, 33, 0              , (NUM_LEDS_SHOULDER + NUM_LEDS_BRA_IN  * 2)) \
  X(5, 32, 0              , 0) \
  X(6, 15, 0              , NUM_LEDS_BOTTOM_RIB) \
  X(7,  2, 0              , NUM_LEDS_BOTTOM_RIB) \
  X(8,  4, 0              , NUM_LEDS_BOTTOM_RIB) \
  X(9, 16, 0              , NUM_LEDS_MIDDLE_RIB) \
  X(A, 17, 0              , NUM_LEDS_MIDDLE_RIB) \
  X(B,  5, 0              , NUM_LEDS_MIDDLE_RIB) \
  X(C, 18, NUM_LEDS_BOTTOM   , NUM_LEDS_TOP_RIB) \
  X(D, 19, NUM_LEDS_BOTTOM   , NUM_LEDS_TOP_RIB) \
  X(E, 21, NUM_LEDS_BOTTOM   , NUM_LEDS_TOP_RIB) \
  X(F, 13, NUM_LEDS_STRAP , 0) \
  X(G, 12, NUM_LEDS_STRAP , 0) \
  X(H,  0, 0              , 0)

#ifdef IS_MALE
#define ACTIVE_COUNT(male, female) male
#else
#define ACTIVE_COUNT(male, female) female
#endif

// Keep this at least as large as the longest enabled strip.
constexpr int MAX_NUM_LEDS = 41;

// The UDP callbacks run separately from the render loop.
uint8_t received_colors[MAX_NUM_LEDS * 3];
uint8_t dmx_color_buffer[MAX_NUM_LEDS * 3];
uint32_t received_frames = 0;
bool standby_requested = false;
bool show_control_requested = false;
bool have_show_frame = false;
uint32_t last_show_frame_ms = 0;
enum class StatusAction { None, Show, Resume };
StatusAction pending_status_action = StatusAction::None;
constexpr char STATUS_REPORT_COMMAND[] = "LED1 STATUS_REPORT";
constexpr char STATUS_RESUME_COMMAND[] = "LED1 RESUME";
constexpr uint32_t STATUS_REPORT_MS = 5 * 60 * 1000UL;
portMUX_TYPE frame_lock = portMUX_INITIALIZER_UNLOCKED;

AsyncUDP artnet_udp;
AsyncUDP control_udp;
char device_id[13];
bool artnet_listening = false;
bool control_listening = false;
uint32_t listener_indicator_ticks = 0;
uint32_t wifi_indicator_ticks = 0;

void handle_packet(uint8_t* packet, size_t len, IPAddress source);

constexpr char PRESET_MODE[] = "Preset";
constexpr char SHOW_MODE[] = "Show Control";
constexpr char STANDBY_MODE[] = "Standby";

WiFiClient mqtt_transport;
PubSubClient mqtt(mqtt_transport);
char mqtt_command_topic[64];
char mqtt_state_topic[64];
char mqtt_availability_topic[64];
char mqtt_discovery_topic[80];

String mqtt_json_name() {
  String escaped;
  for (const unsigned char byte : DEVICE_NAME) {
    if (byte == 0) break;
    if (byte == '"' || byte == '\\') escaped += '\\';
    if (byte < 32) {
      char encoded[7];
      snprintf(encoded, sizeof(encoded), "\\u%04x", static_cast<unsigned>(byte));
      escaped += encoded;
    } else {
      escaped += static_cast<char>(byte);
    }
  }
  return escaped;
}

const char* current_mode_name() {
  portENTER_CRITICAL(&frame_lock);
  const bool standby = standby_requested;
  const bool show = show_control_requested;
  portEXIT_CRITICAL(&frame_lock);
  return standby ? STANDBY_MODE : (show ? SHOW_MODE : PRESET_MODE);
}

void handle_mqtt_mode(char* topic, uint8_t* payload, unsigned int length) {
  if (strcmp(topic, mqtt_command_topic) != 0) return;
  const bool preset = length == sizeof(PRESET_MODE) - 1 &&
                      memcmp(payload, PRESET_MODE, length) == 0;
  const bool show = length == sizeof(SHOW_MODE) - 1 &&
                    memcmp(payload, SHOW_MODE, length) == 0;
  const bool standby = length == sizeof(STANDBY_MODE) - 1 &&
                       memcmp(payload, STANDBY_MODE, length) == 0;
  if (!preset && !show && !standby) return;

  portENTER_CRITICAL(&frame_lock);
  if (standby_requested != standby || (!standby && show_control_requested != show)) {
    have_show_frame = false;
  }
  standby_requested = standby;
  if (!standby) show_control_requested = show;
  portEXIT_CRITICAL(&frame_lock);
}

void mqtt_task(void*) {
  snprintf(mqtt_command_topic, sizeof(mqtt_command_topic), "pikacrew/%s/mode/set", device_id);
  snprintf(mqtt_state_topic, sizeof(mqtt_state_topic), "pikacrew/%s/mode/state", device_id);
  snprintf(mqtt_availability_topic, sizeof(mqtt_availability_topic),
           "pikacrew/%s/availability", device_id);
  snprintf(mqtt_discovery_topic, sizeof(mqtt_discovery_topic),
           "homeassistant/select/pikacrew_%s/mode/config", device_id);

  String discovery = "{\"name\":\"Mode\",\"unique_id\":\"pikacrew_";
  discovery += device_id;
  discovery += "_mode\",\"options\":[\"Preset\",\"Show Control\",\"Standby\"],";
  discovery += "\"command_topic\":\"";
  discovery += mqtt_command_topic;
  discovery += "\",\"state_topic\":\"";
  discovery += mqtt_state_topic;
  discovery += "\",\"availability_topic\":\"";
  discovery += mqtt_availability_topic;
  discovery += "\",\"device\":{\"identifiers\":[\"pikacrew_";
  discovery += device_id;
  discovery += "\"],\"manufacturer\":\"PikaCrew\",\"model\":\"ESP32 LED Controller\",\"name\":\"";
  discovery += mqtt_json_name();
  discovery += "\"}}";

  const size_t buffer_size = discovery.length() + strlen(mqtt_discovery_topic) + 16;
  if (buffer_size > UINT16_MAX || !mqtt.setBufferSize(buffer_size)) {
    Serial.println("PikaCrew MQTT: could not allocate discovery buffer");
    vTaskDelete(nullptr);
    return;
  }
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(handle_mqtt_mode);
  mqtt.setSocketTimeout(3);

  // Broker connections can block briefly. Keep them off the LED render loop.
  const char* published_mode = nullptr;
  for (;;) {
    if (WiFi.status() != WL_CONNECTED) {
      mqtt_transport.stop();
      published_mode = nullptr;
    } else if (!mqtt.connected()) {
      published_mode = nullptr;
      if (mqtt.connect(device_id, MQTT_USER[0] ? MQTT_USER : nullptr,
                       MQTT_USER[0] ? MQTT_PASSWORD : nullptr,
                       mqtt_availability_topic, 0, true, "offline")) {
        const char* mode = current_mode_name();
        if (mqtt.subscribe(mqtt_command_topic) &&
            mqtt.publish(mqtt_discovery_topic, discovery.c_str(), true) &&
            mqtt.publish(mqtt_state_topic, mode, true) &&
            mqtt.publish(mqtt_availability_topic, "online", true)) {
          published_mode = mode;
          Serial.println("PikaCrew MQTT connected");
        } else {
          // Drop the socket so the broker can publish the offline will.
          mqtt_transport.stop();
        }
      } else {
        Serial.printf("PikaCrew MQTT connection failed: %d\n", mqtt.state());
      }
      delay(published_mode ? 20 : 5000);
      continue;
    } else if (mqtt.loop()) {
      const char* mode = current_mode_name();
      if (mode != published_mode && mqtt.publish(mqtt_state_topic, mode, true)) {
        published_mode = mode;
      }
    }
    delay(20);
  }
}

void handle_control(AsyncUDPPacket& packet) {
  const size_t length = packet.length();
  const uint8_t* data = packet.data();
  if (length == sizeof(STATUS_REPORT_COMMAND) - 1 &&
      memcmp(data, STATUS_REPORT_COMMAND, length) == 0) {
    // A device that initiates the broadcast already queues its own report.
    if (packet.remoteIP() != WiFi.localIP()) {
      portENTER_CRITICAL(&frame_lock);
      pending_status_action = StatusAction::Show;
      portEXIT_CRITICAL(&frame_lock);
    }
    return;
  }

  if (length == sizeof(STATUS_RESUME_COMMAND) - 1 &&
      memcmp(data, STATUS_RESUME_COMMAND, length) == 0) {
    portENTER_CRITICAL(&frame_lock);
    pending_status_action = StatusAction::Resume;
    portEXIT_CRITICAL(&frame_lock);
    return;
  }

  constexpr char discovery[] = "LED1 DISCOVER";
  if (length == sizeof(discovery) - 1 &&
      memcmp(data, discovery, length) == 0) {
    char reply[160];
    // Format: LED1 DEVICE <id> <universe> <name, which may contain spaces>
    snprintf(reply, sizeof(reply), "LED1 DEVICE %s %u %s",
             device_id, static_cast<unsigned>(ARTNET_UNIVERSE), DEVICE_NAME);
    packet.print(reply);
    return;
  }

  char sleep_command[32];
  char wake_command[32];
  snprintf(sleep_command, sizeof(sleep_command), "LED1 SLEEP %s", device_id);
  snprintf(wake_command, sizeof(wake_command), "LED1 WAKE %s", device_id);
  const bool sleep = length == strlen(sleep_command) &&
                     memcmp(data, sleep_command, length) == 0;
  const bool wake = length == strlen(wake_command) &&
                    memcmp(data, wake_command, length) == 0;
  if (sleep || wake) {
    portENTER_CRITICAL(&frame_lock);
    if (standby_requested != sleep) have_show_frame = false;
    standby_requested = sleep;
    portEXIT_CRITICAL(&frame_lock);
    packet.print(sleep ? "LED1 OK SLEEP" : "LED1 OK WAKE");
  }
}

void update_listeners(uint32_t now) {
  if (WiFi.status() != WL_CONNECTED) {
    if (artnet_listening) artnet_udp.close();
    if (control_listening) control_udp.close();
    artnet_listening = false;
    control_listening = false;
    listener_indicator_ticks = 0;
    return;
  }

  static uint32_t last_attempt = 0;
  static bool attempted = false;
  if (artnet_listening && control_listening) return;
  if (attempted && now - last_attempt < 1000) return;
  attempted = true;
  last_attempt = now;

  if (!artnet_listening && artnet_udp.listen(ARTNET_PORT)) {
    artnet_udp.onPacket([](AsyncUDPPacket packet) {
      handle_packet(packet.data(), packet.length(), packet.remoteIP());
    });
    artnet_listening = true;
    listener_indicator_ticks = 300;
    Serial.print("Art-Net listening at ");
    Serial.println(WiFi.localIP());
  }
  if (!control_listening && control_udp.listen(CONTROL_PORT)) {
    control_udp.onPacket([](AsyncUDPPacket packet) {
      handle_control(packet);
    });
    control_listening = true;
  }
}

// Derive static buffers, slot count and registration from the same table.
#define ALLOCATE_STRIP(slot, pin, male, female) \
  static_assert(ACTIVE_COUNT(male, female) >= 0 && \
                ACTIVE_COUNT(male, female) <= MAX_NUM_LEDS, \
                "Check LED count and MAX_NUM_LEDS for slot " #slot); \
  CRGB strip_##slot[ACTIVE_COUNT(male, female) > 0 ? ACTIVE_COUNT(male, female) : 1];
LED_OUTPUTS(ALLOCATE_STRIP)
#undef ALLOCATE_STRIP

#define STRIP_BUFFER(slot, pin, male, female) strip_##slot,
CRGB* const leds[] = { LED_OUTPUTS(STRIP_BUFFER) };
#undef STRIP_BUFFER

#define STRIP_LENGTH(slot, pin, male, female) ACTIVE_COUNT(male, female),
constexpr int strip_length[] = { LED_OUTPUTS(STRIP_LENGTH) };
#undef STRIP_LENGTH
constexpr int NUM_STRIPS = sizeof(leds) / sizeof(leds[0]);

template<int Pin, int Count>
typename std::enable_if<(Count > 0)>::type register_strip(CRGB* pixels) {
  FastLED.addLeds<NEOPIXEL, Pin>(pixels, Count);
}

template<int Pin, int Count>
typename std::enable_if<(Count == 0)>::type register_strip(CRGB*) {}

void set_indicator_pixels(CRGB color) {
  for (int s = 0; s < NUM_STRIPS; s++)
    if (strip_length[s] > 0)
      leds[s][0] = color;
  FastLED.show();
}
  

void setup() {

  Serial.begin(115200);

#define REGISTER_STRIP(slot, pin, male, female) \
  register_strip<pin, ACTIVE_COUNT(male, female)>(strip_##slot);
  LED_OUTPUTS(REGISTER_STRIP)
#undef REGISTER_STRIP

  FastLED.setBrightness(128);
  FastLED.setMaxPowerInVoltsAndMilliamps(5, 2000);

  set_indicator_pixels(CRGB(32,0,0));

  snprintf(device_id, sizeof(device_id), "%012llX",
           static_cast<unsigned long long>(ESP.getEfuseMac() & 0xFFFFFFFFFFFFULL));
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(true); // Keep the radio associated while using Wi-Fi power saving.
    WiFi.onEvent([](WiFiEvent_t, WiFiEventInfo_t info) {
    Serial.printf("Wi-Fi disconnect reason: %u\n",
                  static_cast<unsigned>(info.wifi_sta_disconnected.reason));
  }, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  WiFi.begin(WIFI_SSID, WIFI_PASS); // DHCP supplies the address.
  if (MQTT_ENABLED &&
      xTaskCreate(mqtt_task, "pikacrew-mqtt", 6144, nullptr, 1, nullptr) != pdPASS) {
    Serial.println("PikaCrew MQTT: could not start network task");
  }
  wifi_indicator_ticks = 300;
  Serial.printf("LED device %s (%s)\n", device_id, DEVICE_NAME);
}

constexpr size_t ARTDMX_HEADER_SIZE = 18;

#define WAVE_SCALE 256
const int wave_up = 5 * WAVE_SCALE;
const int wave_down = 25 * WAVE_SCALE;
const int wave_speed = 100; // in 1 / WAVE_SCALE of an led per cycle
int wave_position = 0x0FFFFFFF; // start high to force reset
int hue = 0;

void automatic_pattern() {
  wave_position += wave_speed;
  if (wave_position > MAX_NUM_LEDS * WAVE_SCALE + wave_down) {
    wave_position = -wave_up;
    hue = esp_random() % 256;
  }

#ifdef IS_MALE

  for (int i = 0; i < MAX_NUM_LEDS; i++) {
    
    int scaled_i = i * WAVE_SCALE;
    int b = 
      (scaled_i > wave_position + wave_up  ) ? 0 :
      (scaled_i < wave_position - wave_down) ? 0 : 
      (scaled_i > wave_position)             ? ((wave_up   - (scaled_i - wave_position)) *  255) / wave_up   : 
                                               ((wave_down + (scaled_i - wave_position)) *  255) / wave_down ;
      
    CRGB c = CHSV(hue, 255, b);

    for(int s = 0; s < NUM_STRIPS; s++) 
      if (i < strip_length[s]) 
        leds[s][strip_length[s] - 1 - i] = c;
  }

#else // FEMALE

  for (int i = 0; i < MAX_NUM_LEDS; i++) {
    
    int scaled_i = i * WAVE_SCALE;
    int b = 
      (scaled_i > wave_position + wave_up  ) ? 0 :
      (scaled_i < wave_position - wave_down) ? 0 : 
      (scaled_i > wave_position)             ? ((wave_up   - (scaled_i - wave_position)) *  255) / wave_up   : 
                                               ((wave_down + (scaled_i - wave_position)) *  255) / wave_down ;
      
    CRGB c = CHSV(hue, 255, b);

    for(int s = 0; s < NUM_STRIPS; s++) if (i < strip_length[s]) {
      if (s < 6) { // bra
        if (i < strip_length[s]) {
          leds[s][i] = c;
        }
      } else {  // ribs
        int ii = i + 1 - strip_length[s] / 2;
        int a = strip_length[s] / 2 - ii;
        int b = strip_length[s] / 2 + ii;
        if (a >= 0 && a < strip_length[s]) leds[s][a] = c;
        if (b >= 0 && b < strip_length[s]) leds[s][b] = c;
      }
    }
  }
#endif
}

void dmx_symmetric_pattern() {

#ifdef IS_MALE

  for (int i = 0; i < MAX_NUM_LEDS; i++) {
    CRGB c = CRGB(
      dmx_color_buffer[3 * i + 0],
      dmx_color_buffer[3 * i + 1],
      dmx_color_buffer[3 * i + 2]
    );

    for(int s = 0; s < NUM_STRIPS; s++) 
      if (i < strip_length[s]) 
        leds[s][strip_length[s] - 1 - i] = c;
  }

#else // FEMALE

  for (int i = 0; i < MAX_NUM_LEDS; i++) {
    CRGB c = CRGB(
      dmx_color_buffer[3 * i + 0],
      dmx_color_buffer[3 * i + 1],
      dmx_color_buffer[3 * i + 2]
    );

    for(int s = 0; s < NUM_STRIPS; s++) 
      if (i < strip_length[s])
        switch (s) {
          case 0: case 1: case 3: case 4: // bra
            if (i < strip_length[s]) {
              leds[s][i] = c;
            } 
            break;
          default: // ribs
            if (i <= strip_length[s] / 2) {
              leds[s][i] = c;
              leds[s][strip_length[s] - 1 - i] = c;
            }
            break;
        }

  }
#endif

}

constexpr uint32_t FRAME_MODE_TICKS = 300;
constexpr uint32_t MIN_LIVE_FRAMES = 20;
constexpr uint32_t PROBE_INTERVAL_MS = 60000;
constexpr uint32_t PROBE_WINDOW_MS = 2000;

// Passive activity check. The sender already streams ArtDMX, so no probe packet is needed.
void check_frame_activity(uint32_t now, uint32_t total_frames) {
  static uint32_t probe_started = 0;
  static uint32_t starting_frame_count = 0;
  static bool observing = false;

  if (!observing && now - probe_started >= PROBE_INTERVAL_MS) {
    probe_started = now;
    starting_frame_count = total_frames;
    observing = true;
  } else if (observing && now - probe_started >= PROBE_WINDOW_MS) {
    Serial.printf("Art-Net activity in last 2 seconds: %lu accepted frames\n",
                  static_cast<unsigned long>(total_frames - starting_frame_count));
    observing = false;
  }
}

// Called locally from the serial console. The hub can send the same UDP payload.
void status_report() {
  portENTER_CRITICAL(&frame_lock);
  pending_status_action = StatusAction::Show;
  portEXIT_CRITICAL(&frame_lock);

  if (WiFi.status() == WL_CONNECTED) {
    if (control_udp.broadcastTo(STATUS_REPORT_COMMAND, CONTROL_PORT) == 0) {
      Serial.println("Could not send status report broadcast");
    }
  }
}

void check_serial_status_command() {
  static char command[sizeof("status_report")];
  static size_t used = 0;
  static bool overflow = false;
  while (Serial.available()) {
    const int byte = Serial.read();
    if (byte == '\r' || byte == '\n') {
      command[used] = '\0';
      if (!overflow && strcmp(command, "status_report") == 0) status_report();
      used = 0;
      overflow = false;
    } else if (used < sizeof(command) - 1) {
      command[used++] = static_cast<char>(byte);
    } else {
      overflow = true;
    }
  }
}

void show_status_report(bool live, bool standby) {
  const bool connected = WiFi.status() == WL_CONNECTED;
  const CRGB black(0, 0, 0);
  const CRGB red(128, 0, 0);
  const CRGB orange(128, 25, 0);
  const CRGB yellow(120, 100, 0);
  const CRGB green(0, 128, 0);
  const CRGB blue(0, 0, 128);
  const CRGB purple(100, 0, 128);
  const CRGB digits[10][2] = {
    {red, red}, {red, orange}, {orange, orange}, {orange, yellow},
    {yellow, yellow}, {yellow, green}, {green, green}, {green, blue},
    {blue, blue}, {blue, purple}
  };
  CRGB status_pixels[10] = {
    connected ? blue : red,
    connected && artnet_listening ? green : red,
    standby ? black : (live ? purple : yellow),
    black, black, black, black, black, black, black
  };

  // The display has two digits. Larger Art-Net universes remain usable and appear in Serial.
  if (ARTNET_UNIVERSE < 100) {
    const auto& tens = digits[ARTNET_UNIVERSE / 10];
    const auto& ones = digits[ARTNET_UNIVERSE % 10];
    status_pixels[6] = tens[0];
    status_pixels[7] = tens[1];
    status_pixels[8] = ones[0];
    status_pixels[9] = ones[1];
  }

  Serial.printf("Status report | %s (%s) | Wi-Fi: %s | Art-Net listener: %s | mode: %s | universe: %02u\n",
                DEVICE_NAME, device_id, connected ? "connected" : "disconnected",
                connected && artnet_listening ? "on" : "off",
                standby ? "standby" : (live ? "network packets" : "default animation"),
                static_cast<unsigned>(ARTNET_UNIVERSE));
  for (int s = 0; s < NUM_STRIPS; ++s) {
    for (int i = 0; i < strip_length[s]; ++i) {
      leds[s][i] = i < 10 ? status_pixels[i] : black;
    }
  }
  FastLED.show();
}

void loop() {
  const uint32_t now = millis();
      static uint32_t last_wifi_report_ms = 0;
  static bool wifi_was_connected = false;
  const auto wifi_status = WiFi.status();

  if (wifi_status == WL_CONNECTED) {
    if (!wifi_was_connected) {
      Serial.print("PikaCrew Wi-Fi connected at ");
      Serial.println(WiFi.localIP());
    }
    wifi_was_connected = true;
  } else if (wifi_was_connected || now - last_wifi_report_ms >= 20000UL) {
    Serial.printf("PikaCrew Wi-Fi not connected (status %d). Still retrying.\n",
                  static_cast<int>(wifi_status));
    wifi_was_connected = false;
    last_wifi_report_ms = now;
  }
  update_listeners(now);
  check_serial_status_command();

  portENTER_CRITICAL(&frame_lock);
  const uint32_t total_frames = received_frames;
  const bool standby = standby_requested;
  const bool show_feed_live = show_control_requested && have_show_frame &&
                             millis() - last_show_frame_ms < SHOW_FRAME_TIMEOUT_MS;
  const StatusAction status_action = pending_status_action;
  pending_status_action = StatusAction::None;
  if (!standby) {
    memcpy(dmx_color_buffer, received_colors, sizeof(dmx_color_buffer));
  }
  portEXIT_CRITICAL(&frame_lock);

  check_frame_activity(now, total_frames);

  static bool in_standby = false;
  static bool legacy_live = true;
  static uint32_t ticks = 0;
  static uint32_t previous_frame_count = 0;
  if (++ticks >= FRAME_MODE_TICKS) {
    legacy_live = total_frames - previous_frame_count >= MIN_LIVE_FRAMES;
    previous_frame_count = total_frames;
    ticks = 0;
  }

  const bool live = MQTT_ENABLED ? (artnet_listening && show_feed_live) : legacy_live;
  static bool report_visible = false;
  static uint32_t report_started = 0;
  if (status_action == StatusAction::Show) {
    show_status_report(live, standby);
    if (!report_visible || now - report_started >= STATUS_REPORT_MS) report_started = now;
    report_visible = true;
  }
  if (report_visible && status_action != StatusAction::Resume &&
      now - report_started < STATUS_REPORT_MS) {
    delay(10);
    return;
  }
  const bool report_ended = report_visible;
  report_visible = false;

  if (standby) {
    if (report_ended || !in_standby) FastLED.clear(true);
  } else {
    if (live) dmx_symmetric_pattern();
    else automatic_pattern();

    for (int s = 0; s < NUM_STRIPS; s++) {
      if (wifi_indicator_ticks && WiFi.status() == WL_CONNECTED && strip_length[s] > 0) {
        leds[s][0] = CRGB(0, 0, 32);
      }
      if (listener_indicator_ticks && strip_length[s] > 1) {
        leds[s][1] = CRGB(0, 32, 0);
      }
    }
    FastLED.show();
  }
  in_standby = standby;

  if (wifi_indicator_ticks) --wifi_indicator_ticks;
  if (listener_indicator_ticks) --listener_indicator_ticks;
  delay(10);
}

void handle_packet(uint8_t* packet, size_t len, IPAddress source) {
  if (len < ARTDMX_HEADER_SIZE || memcmp(packet, "Art-Net", 8) != 0) return;
  if (packet[8] != 0x00 || packet[9] != 0x50) return; // ArtDMX opcode
  const uint16_t version = (uint16_t(packet[10]) << 8) | packet[11];
  if (version < 14) return;

  const uint16_t universe = (uint16_t(packet[15]) << 8) | packet[14];
  if (universe != ARTNET_UNIVERSE) return;
  const size_t length = (size_t(packet[16]) << 8) | packet[17];
  if (length < 2 || length > 512 || length % 2 ||
      length > len - ARTDMX_HEADER_SIZE) return;

  const size_t copied = length < sizeof(received_colors) ? length : sizeof(received_colors);
  portENTER_CRITICAL(&frame_lock);
  if (!MQTT_ENABLED || SHOW_CONTROLLER_IP == IPAddress(0, 0, 0, 0) ||
      source == SHOW_CONTROLLER_IP) {
    ++received_frames;
    if (!MQTT_ENABLED || (show_control_requested && !standby_requested)) {
      if (MQTT_ENABLED && !have_show_frame) memset(received_colors, 0, sizeof(received_colors));
      memcpy(received_colors, packet + ARTDMX_HEADER_SIZE, copied);
      last_show_frame_ms = millis();
      have_show_frame = true;
    }
  }
  portEXIT_CRITICAL(&frame_lock);
}
