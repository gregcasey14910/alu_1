/*
 * ESP32-S3 Super Mini - ALU Emulator with I2C Scanner & Display
 * Now with proper 74HCT181 ALU emulation
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define LED_PIN    48
#define NUM_PIXELS 1
#define SDA_PIN 8
#define SCL_PIN 9
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

Adafruit_NeoPixel pixel(NUM_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

typedef struct struct_message {
  int cmd;
  int bus_value;
} struct_message;

enum RemoteCmd {
  FCT_ALU     = 0x20,
  B_2_ALU     = 0x21,
  C_2_ALU     = 0x22,
  BC_2_ALU    = 0x23,
  ALU_rsv1    = 0x24,
  ALU_COMBO   = 0x30,
  ALU_2_DBUS  = 0x31,
  CLK_SR      = 0x32,
  HCT_MS      = 0x33,
  ALU_SCZ     = 0x34
};

uint8_t B_REG = 0;
uint8_t C_REG = 0;
uint8_t FCT = 0;
uint8_t ALU_RESULT = 0;
bool sign = false;
bool carry = false;
bool zero = true;
String macAddress = "";

uint8_t F2MS(uint8_t fcode) {
  fcode = fcode & 0x07;
  uint8_t m_bit;
  uint8_t s_code;
  
  switch(fcode) {
    case 0:  // CLR
      m_bit = 1;
      s_code = 0b0011;  // Logic 0
      break;   
    case 1:  // ADD
      m_bit = 0;
      s_code = 0b1001;  // A + B
      break;
    case 2:  // INC
      m_bit = 0;
      s_code = 0b0000;  // A (then we'll add 1 via carry)
      break;
    case 3:  // AND
      m_bit = 1;
      s_code = 0b1011;  // A AND B
      break;
    case 4:  // ORR
      m_bit = 1;
      s_code = 0b1110;  // A OR B
      break;
    case 5:  // XOR
      m_bit = 1;
      s_code = 0b0110;  // A XOR B
      break;
    case 6:  // NOT
      m_bit = 1;
      s_code = 0b0000;  // NOT A
      break;
    case 7:  // SHL (shift left = A + A)
      m_bit = 0;
      s_code = 0b1100;  // A + A
      break;
    default:
      m_bit = 0;
      s_code = 0;
      break;
  }
  
  return (m_bit << 4) | s_code;
}

/*
 * Emulate single 74HCT181 4-bit ALU
 * Inputs: A, B (4-bit values), S (4-bit select), M (mode bit), Cn (carry in, active low)
 * Outputs: F (4-bit result), Cn4 (carry out, active low)
 */
void emulate181(uint8_t a, uint8_t b, uint8_t s, uint8_t m, bool cn, uint8_t &f, bool &cn4) {
  a = a & 0x0F;  // Ensure 4-bit
  b = b & 0x0F;
  s = s & 0x0F;
  
  if (m == 1) {
    // LOGIC MODE - no carry
    switch(s) {
      case 0x0: f = ~a; break;                    // NOT A
      case 0x1: f = ~(a | b); break;              // NOR
      case 0x2: f = (~a) & b; break;              // (NOT A) AND B
      case 0x3: f = 0; break;                     // 0
      case 0x4: f = ~(a & b); break;              // NAND
      case 0x5: f = ~b; break;                    // NOT B
      case 0x6: f = a ^ b; break;                 // XOR
      case 0x7: f = (~a) | b; break;              // (NOT A) OR B
      case 0x8: f = a & (~b); break;              // A AND (NOT B)
      case 0x9: f = ~(a ^ b); break;              // XNOR
      case 0xA: f = b; break;                     // B
      case 0xB: f = a & b; break;                 // AND
      case 0xC: f = 0x0F; break;                  // 1
      case 0xD: f = a | (~b); break;              // A OR (NOT B)
      case 0xE: f = a | b; break;                 // OR
      case 0xF: f = a; break;                     // A
      default: f = 0; break;
    }
    f = f & 0x0F;
    cn4 = true;  // No carry in logic mode
    
  } else {
    // ARITHMETIC MODE - with carry
    uint8_t result;
    uint8_t cin = cn ? 0 : 1;  // Convert active-low to normal
    
    switch(s) {
      case 0x0: result = a + cin; break;                          // A
      case 0x1: result = (a | b) + cin; break;                    // A OR B
      case 0x2: result = (a | (~b & 0x0F)) + cin; break;          // A OR NOT B
      case 0x3: result = 0x0F + cin; break;                       // -1 (all ones)
      case 0x4: result = a + (a & (~b & 0x0F)) + cin; break;      // A + (A AND NOT B)
      case 0x5: result = (a | b) + (a & (~b & 0x0F)) + cin; break;// (A OR B) + (A AND NOT B)
      case 0x6: result = a - b - 1 + cin; break;                  // A - B - 1
      case 0x7: result = (a & (~b & 0x0F)) - 1 + cin; break;      // (A AND NOT B) - 1
      case 0x8: result = a + (a & b) + cin; break;                // A + (A AND B)
      case 0x9: result = a + b + cin; break;                      // A + B
      case 0xA: result = (a | (~b & 0x0F)) + (a & b) + cin; break;// (A OR NOT B) + (A AND B)
      case 0xB: result = (a & b) - 1 + cin; break;                // (A AND B) - 1
      case 0xC: result = a + a + cin; break;                      // A + A (shift left)
      case 0xD: result = (a | b) + a + cin; break;                // (A OR B) + A
      case 0xE: result = (a | (~b & 0x0F)) + a + cin; break;      // (A OR NOT B) + A
      case 0xF: result = a - 1 + cin; break;                      // A - 1
      default: result = 0; break;
    }
    
    f = result & 0x0F;
    cn4 = ((result & 0x10) == 0);  // Carry out (active low) if bit 4 is 0
  }
}

/*
 * Emulate two cascaded 74HCT181 chips for 8-bit operations
 * A is B_REG, B is C_REG in our system
 */
uint8_t emulate8BitALU(uint8_t a_input, uint8_t b_input, uint8_t s_code, uint8_t m_bit, 
                        bool &carry_out, bool &sign_out, bool &zero_out) {
  // Split into nibbles
  uint8_t a_low = a_input & 0x0F;
  uint8_t a_high = (a_input >> 4) & 0x0F;
  uint8_t b_low = b_input & 0x0F;
  uint8_t b_high = (b_input >> 4) & 0x0F;
  
  // For INC operation, we need to force carry in to achieve +1
  bool cn_low = (FCT == 2) ? false : true;  // Active low, false = carry in
  
  // Process lower 4 bits
  uint8_t f_low;
  bool cn4_low;
  emulate181(a_low, b_low, s_code, m_bit, cn_low, f_low, cn4_low);
  
  // Process upper 4 bits (carry from lower chip)
  uint8_t f_high;
  bool cn4_high;
  emulate181(a_high, b_high, s_code, m_bit, cn4_low, f_high, cn4_high);
  
  // Combine results
  uint8_t result = (f_high << 4) | f_low;
  
  // Set flags
  carry_out = !cn4_high;  // Convert active-low to active-high
  sign_out = (result & 0x80) != 0;  // MSB = sign bit
  zero_out = (result == 0);
  
  return result;
}

String getCommandName(int cmd) {
  switch(cmd) {
    case FCT_ALU:     return "FCT_ALU";
    case B_2_ALU:     return "B_2_ALU";
    case C_2_ALU:     return "C_2_ALU";
    case BC_2_ALU:    return "BC_2_ALU";
    case ALU_rsv1:    return "ALU_rsv1";
    case ALU_COMBO:   return "ALU_COMBO";
    case ALU_2_DBUS:  return "ALU_2_DBUS";
    case CLK_SR:      return "CLK_SR";
    case HCT_MS:      return "HCT_MS";
    case ALU_SCZ:     return "ALU_SCZ";
    default:
      char hexbuf[8];
      sprintf(hexbuf, "?0x%02X", cmd);
      return String(hexbuf);
  }
}

void scanI2C() {
  byte error, address;
  int deviceCount = 0;
  
  Serial.println("\n=============================");
  Serial.println("Scanning I2C Bus...");
  Serial.println("=============================");
  
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");
      deviceCount++;
    }
    else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }    
  }
  
  if (deviceCount == 0) {
    Serial.println("No I2C devices found\n");
  } else {
    Serial.print("\nFound ");
    Serial.print(deviceCount);
    Serial.println(" device(s)\n");
  }
}

void displayI2CMap() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  
  display.println("I2C Device Scan");
  display.println("---------------");
  
  byte error, address;
  int deviceCount = 0;
  int line = 2;
  
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      display.setCursor(0, line * 8);
      display.print("0x");
      if (address < 16) display.print("0");
      display.print(address, HEX);
      display.print(" Found");
      deviceCount++;
      line++;
      
      if (line > 7) break;
    }
  }
  
  if (deviceCount == 0) {
    display.setCursor(0, 16);
    display.println("No devices found");
  } else {
    display.setCursor(0, 56);
    display.print("Total: ");
    display.print(deviceCount);
  }
  
  display.display();
}

void displayALUStatus() {
  display.clearDisplay();
  
  // Draw trapezoid - wider at top
  display.drawLine(21, 10, 107, 10, SSD1306_WHITE);   // Top
  display.drawLine(42, 42, 86, 42, SSD1306_WHITE);    // Bottom
  display.drawLine(21, 10, 42, 42, SSD1306_WHITE);    // Left side
  display.drawLine(107, 10, 86, 42, SSD1306_WHITE);   // Right side
  
  // "BLU" centered in trapezoid
  display.setTextSize(2);
  display.setCursor(52, 20);
  display.print("BLU");
  
  // B_REG above left side
  display.setTextSize(1);
  display.setCursor(17, 2);
  display.print("0x");
  if (B_REG < 16) display.print("0");
  display.print(B_REG, HEX);
  
  // C_REG above right side
  display.setCursor(83, 2);
  display.print("0x");
  if (C_REG < 16) display.print("0");
  display.print(C_REG, HEX);
  
  // ALU Result centered below
  display.setTextSize(1);
  display.setCursor(52, 46);
  display.print("0x");
  if (ALU_RESULT < 16) display.print("0");
  display.print(ALU_RESULT, HEX);
  
  // LEFT SIDE - 3 lines
  display.setTextSize(1);
  
  display.setCursor(0, 20);
  display.print("RC");
  
  display.setCursor(0, 28);
  display.print("F=");
  display.print((FCT & 0x04) ? '1' : '0');
  display.print((FCT & 0x02) ? '1' : '0');
  display.print((FCT & 0x01) ? '1' : '0');
  
  // RIGHT SIDE - 3 lines
  uint8_t ms_value = F2MS(FCT);
  uint8_t m_bit = (ms_value >> 4) & 0x01;
  uint8_t s_code = ms_value & 0x0F;
  
  display.setCursor(110, 20);
  display.print("181");
  
  display.setCursor(98, 28);
  display.print("S");
  display.print((s_code & 0x08) ? '1' : '0');
  display.print((s_code & 0x04) ? '1' : '0');
  display.print((s_code & 0x02) ? '1' : '0');
  display.print((s_code & 0x01) ? '1' : '0');
  
  display.setCursor(110, 36);
  display.print("M=");
  display.print(m_bit);
  
  // Status flags at bottom
  display.setTextSize(1);
  
  display.setCursor(0, 56);
  display.print("Sign=");
  display.print(sign ? "1" : "0");
  
  display.setCursor(44, 56);
  display.print("Carry=");
  display.print(carry ? "1" : "0");
  
  display.setCursor(90, 56);
  display.print("Zero=");
  display.print(zero ? "1" : "0");
  
  display.display();
}

void computeALU() {
  // Get M and S bits from FCT
  uint8_t ms_value = F2MS(FCT);
  uint8_t m_bit = (ms_value >> 4) & 0x01;
  uint8_t s_code = ms_value & 0x0F;
  
  // Compute ALU result
  ALU_RESULT = emulate8BitALU(B_REG, C_REG, s_code, m_bit, carry, sign, zero);
  
  Serial.print("ALU Computed: B=0x");
  Serial.print(B_REG, HEX);
  Serial.print(" C=0x");
  Serial.print(C_REG, HEX);
  Serial.print(" FCT=");
  Serial.print(FCT);
  Serial.print(" M=");
  Serial.print(m_bit);
  Serial.print(" S=0x");
  Serial.print(s_code, HEX);
  Serial.print(" => Result=0x");
  Serial.print(ALU_RESULT, HEX);
  Serial.print(" S=");
  Serial.print(sign);
  Serial.print(" C=");
  Serial.print(carry);
  Serial.print(" Z=");
  Serial.println(zero);
}

void flashRandomColor() {
  uint8_t r = random(0, 2) * 255;
  uint8_t g = random(0, 2) * 255;
  uint8_t b = random(0, 2) * 255;
  
  if (r == 0 && g == 0 && b == 0) {
    r = 255;
  }
  
  pixel.setPixelColor(0, pixel.Color(r, g, b));
  pixel.show();
}

void sendALUResult() {
  struct_message outgoingData;
  uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  esp_err_t result;
  
  // Send actual computed result
  outgoingData.cmd = ALU_COMBO; 
  outgoingData.bus_value = ALU_RESULT;
  result = esp_now_send(broadcastAddress, (uint8_t *)&outgoingData, sizeof(outgoingData));
  if (result == ESP_OK) {
    Serial.print("TX ");
    Serial.print(getCommandName(ALU_COMBO));
    Serial.print(": 0x");
    Serial.print(outgoingData.bus_value, HEX);
    Serial.println(" (broadcast sent)");
  } else {
    Serial.println("TX FAILED!");
  }

  // Send HCT_MS
  outgoingData.cmd = HCT_MS; 
  outgoingData.bus_value = F2MS(FCT); 
  result = esp_now_send(broadcastAddress, (uint8_t *)&outgoingData, sizeof(outgoingData));
  if (result == ESP_OK) {
    Serial.print("TX ");
    Serial.print(getCommandName(HCT_MS));
    Serial.print(": 0x");
    Serial.print(outgoingData.bus_value, HEX);
    Serial.println(" (181 codes broadcast sent)");
  } else {
    Serial.println("TX FAILED!");
  }
  
  // Send ALU_SCZ with computed status
  outgoingData.cmd = ALU_SCZ;
  outgoingData.bus_value = (sign ? 0x04 : 0x00) | 
                            (carry ? 0x02 : 0x00) | 
                            (zero ? 0x01 : 0x00);
  result = esp_now_send(broadcastAddress, (uint8_t *)&outgoingData, sizeof(outgoingData));
  if (result == ESP_OK) {
    Serial.print("TX ");
    Serial.print(getCommandName(ALU_SCZ));
    Serial.print(": S=");
    Serial.print(sign ? "1" : "0");
    Serial.print(" C=");
    Serial.print(carry ? "1" : "0");
    Serial.print(" Z=");
    Serial.print(zero ? "1" : "0");
    Serial.print(" (0x");
    Serial.print(outgoingData.bus_value, HEX);
    Serial.println(" broadcast sent)");
  } else {
    Serial.println("TX FAILED!");
  }
  
  Serial.println();
}

void onDataReceive(const esp_now_recv_info *recv_info, const uint8_t *incomingData, int len) {
  struct_message receivedData;
  memcpy(&receivedData, incomingData, sizeof(receivedData));
  
  char macStr[18];
  sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
          recv_info->src_addr[0], recv_info->src_addr[1],
          recv_info->src_addr[2], recv_info->src_addr[3],
          recv_info->src_addr[4], recv_info->src_addr[5]);
  
  bool shouldRespond = false;
  
  switch(receivedData.cmd) {
    case B_2_ALU:
      B_REG = receivedData.bus_value;
      Serial.print("RX ");
      Serial.print(getCommandName(B_2_ALU));
      Serial.print(": B_REG = 0x");
      Serial.print(B_REG, HEX);
      Serial.print(" from ");
      Serial.println(macStr);
      flashRandomColor();
      computeALU();
      displayALUStatus();
      shouldRespond = true;
      break;
      
    case C_2_ALU:
      C_REG = receivedData.bus_value;
      Serial.print("RX ");
      Serial.print(getCommandName(C_2_ALU));
      Serial.print(": C_REG = 0x");
      Serial.print(C_REG, HEX);
      Serial.print(" from ");
      Serial.println(macStr);
      flashRandomColor();
      computeALU();
      displayALUStatus();
      shouldRespond = true;
      break;
      
    case FCT_ALU:
      FCT = receivedData.bus_value & 0x07;
      Serial.print("RX ");
      Serial.print(getCommandName(FCT_ALU));
      Serial.print(": FCT = 0x");
      Serial.print(FCT, HEX);
      Serial.print(" from ");
      Serial.println(macStr);
      flashRandomColor();
      computeALU();
      displayALUStatus();
      shouldRespond = true;
      break;
      
    default:
      break;
  }
  
  if (shouldRespond) {
    sendALUResult();
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n=============================");
  Serial.println("ESP32-S3 ALU Emulator");
  Serial.println("with 74HCT181 Emulation");
  Serial.println("=============================");
  
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(100);
  Serial.print("I2C initialized on SDA=GPIO");
  Serial.print(SDA_PIN);
  Serial.print(", SCL=GPIO");
  Serial.println(SCL_PIN);
  
  scanI2C();
  
  Serial.println("Initializing SSD1306 display...");
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("SSD1306 allocation failed!");
    Serial.println("Check I2C connections and address");
  } else {
    Serial.print("SSD1306 found at address 0x");
    Serial.println(SCREEN_ADDRESS, HEX);
    
    displayI2CMap();
    Serial.println("I2C map displayed on OLED");
    delay(5000);
    
    // Initial ALU computation and display
    computeALU();
    displayALUStatus();
    Serial.println("ALU status display active");
  }
  
  pixel.begin();
  pixel.setBrightness(50);
  pixel.setPixelColor(0, pixel.Color(0, 0, 0));
  pixel.show();
  Serial.println("NeoPixel initialized on GPIO 48");
  
  WiFi.mode(WIFI_STA);
  delay(500);
  
  uint8_t mac[6];
  esp_wifi_get_mac(WIFI_IF_STA, mac);
  char macStr[18];
  sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  macAddress = String(macStr);
  
  Serial.print("MAC Address: ");
  Serial.println(macAddress);
  
  WiFi.disconnect();
  
  Serial.println("Initializing ESP-NOW...");
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW FAILED!");
    pixel.setPixelColor(0, pixel.Color(255, 0, 0));
    pixel.show();
    while(1) delay(1000);
  }
  
  esp_now_register_recv_cb(onDataReceive);
  
  esp_now_peer_info_t peerInfo = {};
  uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
  
  Serial.println("ESP-NOW Ready!");
  Serial.println("Listening for broadcasts...\n");
  
  pixel.setPixelColor(0, pixel.Color(0, 255, 0));
  pixel.show();
  delay(500);
  pixel.setPixelColor(0, pixel.Color(0, 0, 0));
  pixel.show();
}

void loop() {
  delay(10);
}