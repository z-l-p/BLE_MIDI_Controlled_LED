////////////////////////////////////////
//   BLE MIDI to HSV LED controller   //
////////////////////////////////////////
// Zach Poff - 2026 (with help from Claude AI for control-surface library hacking)

// ESP32-S3 board advertises itself as BLE MIDI Device. It accepts 3 MIDI CC values (for Hue, Sat, Value)
// If steup button is held during boot, it creates a Captive Portal webpage where user can input settings

// NOTES:
// in Arduino Tools menu / USB Mode select OTG (even though we don't use USB-OTG)
// uses 14bit MIDI, with fallback to 7bit if no "LSB" messages are received.
// (LED dimming is 8bit, but includes FastLED hardware gamma for supported LEDs, so low range is much better than normal 8bit)

// Code Summary:
//   1. Load settings from NVRAM (via Preferences library)
//   2. If settings are missing, or portalPin button pressed, run captive portal (WiFiManager library) (builtin LED blinks fast)
//   3. User connects to captive portal WiFi, changes settings for BLE device name, etc.
//   4. User submits form. Form values are saved to NVRAM, then BLE starts advertising (control-surface library) (builtin LED glows dimly)
//   5. Computer discovers BLE MIDI device, user connects via MacOS Audio MIDI Setup app
//   6. In DAW, user selects BLE MIDI device as a track output
//   7. In DAW, user draws 14bit MIDI CC envelopes for Hue, Saturation, Value
//   8. As DAW plays, ESP uses FastLED library to update addressable LED color each time a new MIDI CC message arrives
//   (many ESP devices with different BLE names can connect to the same computer)

// TODO:
// If FastLED ever supports 16bit LEDs, use them! (We are wasting most of our 14bit CC resolution on 8bit LEDs, even with FastLED 13bit gamma)

// --- INCLUDE LIBRARIES ---
// First a workaround because FastLED & Control_Surface libs on ESP32 both have members called "Selectable"!
#define Selectable FastLED_Selectable
#include <FastLED.h>
#undef Selectable
#include <Control_Surface.h>  // ** YOU MUST manually install forked version of the control-surface library **
                              // https://github.com/z-l-p/Control-Surface-with-BLE-mods
                              // (because normal lib can't set BLE manufacturer/model names!)
#include <WiFiManager.h>      // https://github.com/tzapu/WiFiManager
#include <Preferences.h>      // ESP32 built-in NVRAM key-value store

// --- Custom BLE DIS strings (for modified Control_Surface library) ---
namespace cs::midi_ble_nimble {
extern void setDISManufacturer(const char *name);
extern void setDISModel(const char *model);
}

// --- Hardware Setup variables (not exposed via config portal)  ---

#define LED_PIN_DATA 2              // (pin 8 on Xiao) Adressable LED pins
#define LED_PIN_CLOCK 3             // (pin 7 on Xiao) Adressable LED pins
// APA102HD = APA102 with 13-bit hardware dimming
// HD107HD = APA102-compatible with 13-bit hardware dimming
// SK9822HD = APA102-compatible with 13-bit hardware dimming
#define LED_TYPE SK9822HD           // see above for common 13-bit types. See FastLED src/fl/chipsets/spi_chipsets.h for others
#define LED_ORDER BGR               // Some chipsets are RGB, some BGR, etc
// see this for color correction / color temp info: https://github.com/FastLED/FastLED/wiki/FastLED-Color-Correction
#define LED_CC CRGB(255, 180, 210)  // RGB Color correction values (255, 180, 210 seems to produce a nuetral white fpr SK9822)
#define LED_TEMP CRGB(255, 255, 244)// color temperature applied after color correction (RGB or keywords: Tungsten40W, Halogen, CarbonArc, DirectSunlight)
// scaling curve of each color channel (> 0 boosts the low end, < 0 attenuates the low end)
// Adjust these to correct the color shift that happens accross the brightness range (due to LED curren/brightness nonlinearity)
#define LED_CURVE_R 0.6
#define LED_CURVE_G 0.0
#define LED_CURVE_B 0.0
#define dimFloor 25      // FastLED hardware gamma clips off lowest dim values. dimFloor maps outputs to reduce the problem.
#define portalPin 1  // (pin 9 on Xiao) Pull this GPIO low at boot to run config portal even if NVRAM settings exist

// --- NVRAM namespace ---
#define PREFS_NS "midiled"

// --- Max string lengths (including null terminator) ---
#define BLE_STR_LEN 64

// --- Settings variables (updated later from NVRAM or captive portal) ---
// The values below become the defaults for the first run of the portal
int hueCC = 16;
int satCC = 17;
int valCC = 18;
int NUM_LEDS = 4;
char BLEdevice[BLE_STR_LEN] = "ESP MIDI LED XX";
char BLEmanu[BLE_STR_LEN] = "ESP";
char BLEmodel[BLE_STR_LEN] = "MIDI LED XX";

// CCValue objects are heap-allocated after settings are loaded,
// because their CC numbers depend on hueCC / satCC / valCC.
CCValue *hueMSB = nullptr, *hueLSB = nullptr;
CCValue *satMSB = nullptr, *satLSB = nullptr;
CCValue *valMSB = nullptr, *valLSB = nullptr;

// --- LED pixel Runtime state ---
CRGB *leds = nullptr;
uint8_t g_hue = 0, g_sat = 255, g_val = 255;

// instantiate MIDI interface in Control-Surface library
BluetoothMIDI_Interface midi;

// instantiate Preferences library
Preferences prefs;

// ---------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  delay(1000);
  Serial.println("MIDI LED SKETCH START");

  // Check portal override pin before anything else
  pinMode(portalPin, INPUT_PULLUP);
  delay(10);  // let pin settle
  bool pinForced = (digitalRead(portalPin) == LOW);
  if (pinForced) Serial.println("Portal pin LOW — captive portal forced.");

  if (nvramHasSettings()) loadSettings();  // always load NVRAM settings if present

  if (pinForced || !nvramHasSettings()) {
    runCaptivePortal();  // if no settings or portal button pressed, then load captive portal
                         // When it's finished, the settings variables will be populated
  }

  Serial.printf(
    "Settings: hueCC=%d satCC=%d valCC=%d NUM_LEDS=%d\n",
    hueCC, satCC, valCC, NUM_LEDS);
  Serial.printf(
    "BLE: device='%s' manu='%s' model='%s'\n",
    BLEdevice, BLEmanu, BLEmodel);

  // --- Initialise subsystems with the current settings ---
  initLEDs();
  initCCValues();
  midi.setName(BLEdevice);
  cs::midi_ble_nimble::setDISManufacturer(BLEmanu);
  cs::midi_ble_nimble::setDISModel(BLEmodel);
  testLEDs();  // run LED test pattern
  delay(500);
  Control_Surface.begin();
  Serial.println("Control Surface started");
  digitalWrite(LED_BUILTIN, HIGH);  // LED on to confirm that control surface started OK
  delay(250);
  digitalWrite(LED_BUILTIN, LOW);  // LED off
  Serial.println("(Setup complete)");
  rgbLedWrite(LED_BUILTIN, 1, 1, 1);  // leave LED on dimly to indicate power good
}

// ---------------------------------------------------------------
// LOOP
// ---------------------------------------------------------------

void loop() {
  Control_Surface.loop();

  bool changed = false;  // default

  // This pattern combines 2 incoming 7bit CC messages to make one 14bit message
  if (hueMSB->getDirty() || hueLSB->getDirty()) {
    uint16_t val14 = (hueMSB->getValue() << 7) | hueLSB->getValue();
    g_hue = val14 >> 6;
    hueMSB->clearDirty();
    hueLSB->clearDirty();
    changed = true;
  }

  if (satMSB->getDirty() || satLSB->getDirty()) {
    uint16_t val14 = (satMSB->getValue() << 7) | satLSB->getValue();
    g_sat = val14 >> 6;
    satMSB->clearDirty();
    satLSB->clearDirty();
    changed = true;
  }

  if (valMSB->getDirty() || valLSB->getDirty()) {
    uint16_t val14 = (valMSB->getValue() << 7) | valLSB->getValue();
    // g_val = val14 >> 6; // regular bitshift option
    g_val = map(val14, 0, 16383, dimFloor, 255);  // manual scaling with dimFloor
    //Serial.printf("14bit: %d = %d (scaled 8bit)\n", val14, g_val);
    valMSB->clearDirty();
    valLSB->clearDirty();
    changed = true;
  }

  if (changed) {
    updateLEDs();
  }
}

// ---------------------------------------------------------------
// NVRAM helpers
// ---------------------------------------------------------------

// Returns true if all expected keys are present in NVRAM.
// We use "hueCC" as a sentinel — if it's absent, assume first boot.
bool nvramHasSettings() {
  prefs.begin(PREFS_NS, /*readOnly=*/true);
  bool exists = prefs.isKey("hueCC");
  prefs.end();
  return exists;
}

// load settings from NVRAM into global variables
void loadSettings() {
  prefs.begin(PREFS_NS, /*readOnly=*/true);
  hueCC = prefs.getInt("hueCC", hueCC);
  satCC = prefs.getInt("satCC", satCC);
  valCC = prefs.getInt("valCC", valCC);
  NUM_LEDS = prefs.getInt("numLEDs", NUM_LEDS);
  prefs.getString("BLEdevice", BLEdevice, BLE_STR_LEN);
  prefs.getString("BLEmanu", BLEmanu, BLE_STR_LEN);
  prefs.getString("BLEmodel", BLEmodel, BLE_STR_LEN);
  prefs.end();
  Serial.println("Settings loaded from NVRAM.");
}

// save settings from global variables into VRAM
void saveSettings() {
  prefs.begin(PREFS_NS, /*readOnly=*/false);
  prefs.putInt("hueCC", hueCC);
  prefs.putInt("satCC", satCC);
  prefs.putInt("valCC", valCC);
  prefs.putInt("numLEDs", NUM_LEDS);
  prefs.putString("BLEdevice", BLEdevice);
  prefs.putString("BLEmanu", BLEmanu);
  prefs.putString("BLEmodel", BLEmodel);
  prefs.end();
  Serial.println("Settings saved to NVRAM.");
}

// ---------------------------------------------------------------
// Captive portal
// ---------------------------------------------------------------

/**
 * Start a WiFiManager captive portal, block until the user submits
 * the settings form, then save to NVRAM.
 *
 * The portal AP name is "MIDI-LED-Setup". Connect to it, and a
 * browser should open automatically (or navigate to 192.168.4.1).
 * The form shows the standard WiFi credential fields plus our
 * custom parameters below.
 */


void runCaptivePortal() {
  Serial.println("Starting captive portal...");

  WiFiManager wm;



  // --- Custom parameters ---
  // WiFiManagerParameter(id, label, defaultValue, length)
  char defHueCC[8], defSatCC[8], defValCC[8], defNumLEDs[8];
  snprintf(defHueCC, sizeof(defHueCC), "%d", hueCC);
  snprintf(defSatCC, sizeof(defSatCC), "%d", satCC);
  snprintf(defValCC, sizeof(defValCC), "%d", valCC);
  snprintf(defNumLEDs, sizeof(defNumLEDs), "%d", NUM_LEDS);

  WiFiManagerParameter p_hueCC("hueCC", "MIDI CC# for Hue", defHueCC, 4);
  WiFiManagerParameter p_satCC("satCC", "MIDI CC# for Saturation", defSatCC, 4);
  WiFiManagerParameter p_valCC("valCC", "MIDI CC# for Brightness", defValCC, 4);
  WiFiManagerParameter p_numLEDs("numLEDs", "Number of Addressable LEDs", defNumLEDs, 6);
  WiFiManagerParameter p_BLEdevice("BLEdevice", "BLE Device Name (used by OS)", BLEdevice, BLE_STR_LEN - 1);
  WiFiManagerParameter p_BLEmanu("BLEmanu", "BLE Manufacturer (used by DAW)", BLEmanu, BLE_STR_LEN - 1);
  WiFiManagerParameter p_BLEmodel("BLEmodel", "BLE Model (used by DAW)", BLEmodel, BLE_STR_LEN - 1);

  wm.addParameter(&p_BLEdevice);
  wm.addParameter(&p_BLEmanu);
  wm.addParameter(&p_BLEmodel);
  wm.addParameter(&p_numLEDs);
  wm.addParameter(&p_hueCC);
  wm.addParameter(&p_satCC);
  wm.addParameter(&p_valCC);

  wm.setConfigPortalTimeout(0);       // 0 = wait forever
  wm.setConfigPortalBlocking(false);  // we drive the loop ourselves

  // Start the AP + web portal (non-blocking on ESP32)

  // Use CSS to hide some things in the portal that we don't need!
  wm.setCustomHeadElement(
    "<style>"
    "input[name=s],input[name=p]{display:none!important;}"
    "label[for=s],label[for=p]{display:none!important;}"
    "form[action='/wifi']{display:none!important;}"  // Wifi Settings button
    "form[action='/info']{display:none!important;}"  // Info button
    "div.msg{display:none!important;}"               // "No AP set" text
    "div.wrap h3{display:none!important;}"           // the redundant H3 subheadline
    "</style>");

  wm.setTitle("MIDI LED Setup");
  wm.setParamsPage(1);

  bool portalDone = false;

  // this callback makes it so that when user saves the param page, the portal exits without needing to
  // go back to main page and exit manually.
  wm.setSaveParamsCallback([&]() {
    hueCC = atoi(p_hueCC.getValue());
    satCC = atoi(p_satCC.getValue());
    valCC = atoi(p_valCC.getValue());
    NUM_LEDS = atoi(p_numLEDs.getValue());
    strlcpy(BLEdevice, p_BLEdevice.getValue(), BLE_STR_LEN);
    strlcpy(BLEmanu, p_BLEmanu.getValue(), BLE_STR_LEN);
    strlcpy(BLEmodel, p_BLEmodel.getValue(), BLE_STR_LEN);
    saveSettings();
    portalDone = true;  // signal the loop to exit cleanly
  });
  wm.startConfigPortal("MIDI-LED-Setup");
  Serial.println("Portal AP started — connect to MIDI-LED-Setup");

  // Spin until the user saves settings and the portal closes
  while (wm.getConfigPortalActive() && !portalDone) {
    digitalWrite(LED_BUILTIN, HIGH);  // LED off
    wm.process();
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);  // LED on
    delay(50);
  }

  wm.stopConfigPortal();  // safe to call here, outside the HTTP handler

  bool connected = WiFi.isConnected();

  // Retrieve values regardless of whether WiFi connected —
  // we only need the custom fields.
  // hueCC    = atoi(p_hueCC.getValue());
  // satCC    = atoi(p_satCC.getValue());
  // valCC    = atoi(p_valCC.getValue());
  // NUM_LEDS = atoi(p_numLEDs.getValue());
  // strlcpy(BLEdevice, p_BLEdevice.getValue(), BLE_STR_LEN);
  // strlcpy(BLEmanu,   p_BLEmanu.getValue(),   BLE_STR_LEN);
  // strlcpy(BLEmodel,  p_BLEmodel.getValue(),  BLE_STR_LEN);

  saveSettings();

  if (connected) {
    Serial.println("Portal closed — WiFi connected (not needed, ignoring).");
  } else {
    Serial.println("Portal closed — no WiFi connection (expected).");
  }

  // Shut WiFi down; we don't need it and it may interfere with BLE
  WiFi.mode(WIFI_OFF);
  Serial.println("WiFi off.");
}

// ---------------------------------------------------------------
// Post-settings initialisation
// ---------------------------------------------------------------

void initCCValues() {
  // Allocate CCValue objects now that CC numbers are known.
  hueMSB = new CCValue(MIDIAddress(hueCC, CHANNEL_1));
  hueLSB = new CCValue(MIDIAddress(hueCC + 32, CHANNEL_1));
  satMSB = new CCValue(MIDIAddress(satCC, CHANNEL_1));
  satLSB = new CCValue(MIDIAddress(satCC + 32, CHANNEL_1));
  valMSB = new CCValue(MIDIAddress(valCC, CHANNEL_1));
  valLSB = new CCValue(MIDIAddress(valCC + 32, CHANNEL_1));
}

void initLEDs() {
  leds = new CRGB[NUM_LEDS];
  FastLED.addLeds<LED_TYPE, LED_PIN_DATA, LED_PIN_CLOCK, LED_ORDER>(leds, NUM_LEDS)
    .setCorrection(LED_CC)      // hand-tweaked color correction
    .setTemperature(LED_TEMP);  // color temperature
  Serial.println("FastLED init OK");
}

void updateLEDs() {
  CRGB ledRGB = CRGB(CHSV(g_hue, g_sat, g_val)); // convert HSV ro RGB
    // scale RGB values with nonlinear curve for brightness-based color correction
  ledRGB = CRGB(
    round(fscale(0, 255, 0, 255, ledRGB.r, LED_CURVE_R)),
    round(fscale(0, 255, 0, 255, ledRGB.g, LED_CURVE_G)),
    round(fscale(0, 255, 0, 255, ledRGB.b, LED_CURVE_B))
    );  
  fill_solid(leds, NUM_LEDS, ledRGB);
  FastLED.show();
}

void testLEDs() {
  fill_solid(leds, NUM_LEDS, CRGB(127, 0, 0));
  FastLED.show();
  delay(500);
  fill_solid(leds, NUM_LEDS, CRGB(0, 127, 0));
  FastLED.show();
  delay(500);
  fill_solid(leds, NUM_LEDS, CRGB(0, 0, 127));
  FastLED.show();
  delay(500);
  fill_solid(leds, NUM_LEDS, CRGB(0, 0, 0));
  FastLED.show();
  Serial.println("LED Test complete");
}

// scale values with a curve
float fscale( float originalMin, float originalMax, float newBegin, float
newEnd, float inputValue, float curve){

  float OriginalRange = 0;
  float NewRange = 0;
  float zeroRefCurVal = 0;
  float normalizedCurVal = 0;
  float rangedValue = 0;
  boolean invFlag = 0;


  // condition curve parameter
  // limit range

  if (curve > 10) curve = 10;
  if (curve < -10) curve = -10;

  curve = (curve * -.1) ; // - invert and scale - this seems more intuitive - postive numbers give more weight to high end on output
  curve = pow(10, curve); // convert linear scale into lograthimic exponent for other pow function

  /*
   Serial.println(curve * 100, DEC);   // multply by 100 to preserve resolution  
   Serial.println();
   */

  // Check for out of range inputValues
  if (inputValue < originalMin) {
    inputValue = originalMin;
  }
  if (inputValue > originalMax) {
    inputValue = originalMax;
  }

  // Zero Refference the values
  OriginalRange = originalMax - originalMin;

  if (newEnd > newBegin){
    NewRange = newEnd - newBegin;
  }
  else
  {
    NewRange = newBegin - newEnd;
    invFlag = 1;
  }

  zeroRefCurVal = inputValue - originalMin;
  normalizedCurVal  =  zeroRefCurVal / OriginalRange;   // normalize to 0 - 1 float

  /*
  Serial.print(OriginalRange, DEC);  
   Serial.print("   ");  
   Serial.print(NewRange, DEC);  
   Serial.print("   ");  
   Serial.println(zeroRefCurVal, DEC);  
   Serial.println();  
   */

  // Check for originalMin > originalMax  - the math for all other cases i.e. negative numbers seems to work out fine
  if (originalMin > originalMax ) {
    return 0;
  }

  if (invFlag == 0){
    rangedValue =  (pow(normalizedCurVal, curve) * NewRange) + newBegin;

  }
  else     // invert the ranges
  {  
    rangedValue =  newBegin - (pow(normalizedCurVal, curve) * NewRange);
  }

  return rangedValue;
}