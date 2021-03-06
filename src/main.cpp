/*****************************************************************
   Author: Steven van der Schoot
   Date Created:  10/02/2020
   Date Modified: 25/05/2020


   WotageiNL kira lights. this script is meant for the M5stickC
   This is a prototype to test if its possible to build a Cheer
   Light for music concerts

Current components during prototyping are;
-ESP32-PICO
-ST7735S
-BM8563
-SH200Q
-MPU6886
-AXP192
-SPM1423
 *****************************************************************/

#include <Arduino.h>
#include <M5StickC.h>
#include <FastLED.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

//Addressable LED Hardware parameters
#define LED_CHIPSET ws2812
#define LED_PIN 26
#define NUM_LEDS 16

//BLE
BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

//Menu
enum KiraLightMenu
{
  MENU_COLOR_PICKER,
  MENU_GYRO,
  MENU_DEBUG
};
byte appMode = MENU_DEBUG;

TaskHandle_t PatternCylonHandle = NULL;
TaskHandle_t MotionFlareTaskHandle = NULL;
//TaskHandle_t FlareCoolDownHandle = NULL;

//FastLED variables
CRGB leds[NUM_LEDS];
CHSV newHSVColor = CHSV(10, 255, 10);
CRGB newRGBColor = CHSV(10, 255, 10);
uint8_t baseHue = -20;
uint8_t Basebrightness = 10;
uint8_t brightness = 0;
uint8_t flareThreshold = 150;
uint8_t dropoff = 10;
uint8_t flareUp = 20;

// TFT ST7735S 80X160@0.96"
TFT_eSprite displayBuffer = TFT_eSprite(&M5.Lcd);

//IMU MPU6886 gyro variables
float gyroX = 0.0F;
float gyroY = 0.0F;
float gyroZ = 0.0F;
float sigmaGyro = 0.0f;

//Image Loaded into PROGMEM just for style
#define Img_width 20
#define Img_height 20
long Image_bits[] PROGMEM = {
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0xff80, 0xfff3, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xfff3, 0xff80, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0xff80, 0xffff, 0xfff3, 0xfff3, 0x0000, 0xfff3, 0xfff3, 0xfff3, 0xfff3, 0x0000, 0xfff3, 0xfff3, 0xffff, 0xff80, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0xfe41, 0xfff3, 0xfff3, 0x0000, 0xfff3, 0xfff3, 0xffff, 0xfff3, 0xfff3, 0xfff3, 0x0000, 0xfff3, 0xfff3, 0xfe41, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0xfff3, 0xfff3, 0xfff3, 0x0000, 0xfff3, 0xfff3, 0xfff3, 0xfff3, 0xfff3, 0xfff3, 0xfff3, 0xfff3, 0x0000, 0xfff3, 0xfff3, 0xfff3, 0x0000, 0x0000,
    0x0000, 0xfff3, 0xfe41, 0xfff3, 0x0000, 0xffff, 0xfff3, 0xfff3, 0xfe41, 0xfff3, 0xfff3, 0xfe41, 0xfff3, 0xfff3, 0xfff3, 0x0000, 0xfff3, 0xfe41, 0xfff3, 0x0000,
    0x0000, 0x0000, 0xfff3, 0xfff3, 0x0000, 0xfff3, 0xfff3, 0xff80, 0xfe41, 0xfff3, 0xfff3, 0xfe41, 0xff80, 0xfff3, 0xfff3, 0x0000, 0xfff3, 0xfff3, 0x0000, 0x0000,
    0x0000, 0xff80, 0xfff3, 0xff80, 0xfe41, 0xff80, 0xfff3, 0xfe41, 0xcd4f, 0xff80, 0xfff3, 0xcd4f, 0xfe41, 0xfff3, 0xff80, 0xfe41, 0xff80, 0xfff3, 0xff80, 0x0000,
    0x0000, 0xff80, 0xff80, 0xff80, 0xfe41, 0xff80, 0xfe41, 0xcd4f, 0xcd4f, 0xcd4f, 0xfff3, 0xff58, 0xcd4f, 0xfe41, 0xff80, 0xfe41, 0xff80, 0xff80, 0xff80, 0x0000,
    0x0000, 0xff80, 0xff80, 0xff80, 0x0000, 0xff80, 0xff80, 0x0000, 0xff58, 0xcd4f, 0xfff3, 0xff58, 0x0000, 0xff80, 0xff80, 0x0000, 0xff80, 0xff80, 0xff80, 0x0000,
    0x0000, 0x0000, 0xff80, 0xff80, 0x0000, 0xff80, 0xff80, 0x0000, 0xff58, 0xff58, 0xcd4f, 0xff58, 0x0000, 0xff80, 0xff80, 0x0000, 0xff80, 0xff80, 0xff80, 0x0000,
    0x0000, 0xfe41, 0xff80, 0xff80, 0xff80, 0x0000, 0xff80, 0x0000, 0xff58, 0xff58, 0xff58, 0xff58, 0x0000, 0xff80, 0x0000, 0xff80, 0xff80, 0xff80, 0x0000, 0x0000,
    0x0000, 0xff80, 0xff80, 0xff80, 0xff80, 0x0000, 0xff80, 0xfdeb, 0xef36, 0xff58, 0xff58, 0xef36, 0xfdeb, 0xff80, 0x0000, 0xff80, 0xff80, 0xff80, 0xfe41, 0x0000,
    0x0000, 0xff80, 0xff80, 0xff80, 0x0000, 0x0000, 0x0000, 0x0000, 0xff58, 0xff58, 0xff58, 0xff58, 0x0000, 0x0000, 0x0000, 0x0000, 0xff80, 0xff80, 0xff80, 0x0000,
    0x0000, 0x0000, 0xff80, 0xff80, 0xfe41, 0x0000, 0x0000, 0x0000, 0x0000, 0xc618, 0xffff, 0x0000, 0x0000, 0x0000, 0x0000, 0xfe41, 0xff80, 0xff80, 0x0000, 0x0000,
    0x0000, 0x0000, 0xff80, 0xff80, 0xff80, 0x0000, 0x0000, 0x0000, 0xbbca, 0xffff, 0xc618, 0xbbca, 0x0000, 0x0000, 0x0000, 0xff80, 0xff80, 0xff80, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0xfff3, 0x0000, 0x0000, 0x0000, 0xff58, 0xfbe4, 0xffff, 0xc618, 0xfbe4, 0xff58, 0x0000, 0x0000, 0x0000, 0xfff3, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xfbe4, 0xfbe4, 0x7497, 0x7497, 0xfbe4, 0xfbe4, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xff58, 0xff58, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xfbe4, 0xfbe4, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};

void PatternCylon();

class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
  }
};

class MyCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    String value = pCharacteristic->getValue().c_str();

    if (value.length() > 0)
    {
      if (value == "Clear")
      {
        newHSVColor.hue = 10;
        newHSVColor.value = 20;
        vTaskSuspendAll();
      }
      else if (value == "Cylon")
      {
        vTaskSuspendAll();
        vTaskResume(PatternCylonHandle);
      }
      else if (value == "Flare")
      {
        vTaskSuspendAll();
        vTaskResume(MotionFlareTaskHandle);
      }
      else
      {
        newHSVColor.hue = value.toInt();
        FastLED.show();
      }
    }
  }
  void onRead(BLECharacteristic *pCharacteristic)
  {
    std::string rxvalue = String(newHSVColor.hue).c_str();
    pCharacteristic->setValue(rxvalue);
  }
};

void StartServer()
{
  Serial.begin(115200);

  Serial.println("Starting BLE server");

  BLEDevice::init("KiraLights");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE);

  pCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
}

void DrawBitImage(long imageBits[], uint8_t imageScale, uint8_t xOffset, uint8_t yOffset)
{
  uint8_t _scalingPixel = 0;
  int _currentPixel = 0;
  uint8_t x = 0;
  uint8_t imageWidth = Img_width;
  uint8_t imageHeight = Img_height;

  for (uint8_t i = 0; i < (imageHeight); i++)
  {
    for (uint8_t k = 0; k < imageScale; k++, x++)
    {
      for (uint8_t j = 0; j < imageWidth * imageScale; j++)
      {
        displayBuffer.drawPixel(j + xOffset, x + yOffset, pgm_read_dword(&(imageBits[_currentPixel])));
        if (_scalingPixel >= (imageScale - 1))
        {
          _currentPixel++;
          _scalingPixel = 0;
        }
        else
        {
          _scalingPixel++;
        }
      }
      if (k < imageScale - 1)
      {
        _currentPixel -= imageWidth;
      }
    }
  }
  _currentPixel = 0;
}

//***********************************
//Colortype converters
//***********************************
CRGB HextoCrgb(uint8_t _hexValue)
{

  uint8_t r = ((((_hexValue >> 11) & 0x1F) * 527) + 23) >> 6;
  uint8_t g = ((((_hexValue >> 5) & 0x3F) * 259) + 33) >> 6;
  uint8_t b = (((_hexValue & 0x1F) * 527) + 23) >> 6;

  //uint32_t RGB888 = r << 16 | g << 8 | b;

  return CRGB(r, g, b);
}

uint8_t CrgbtoHex(CRGB _RGB)
{
  uint8_t red = _RGB.r;
  uint8_t green = _RGB.g;
  uint8_t blue = _RGB.b;

  uint16_t b = (blue >> 3) & 0x1f;
  uint16_t g = ((green >> 2) & 0x3f) << 5;
  uint16_t r = ((red >> 3) & 0x1f) << 11;

  return (uint16_t)(r | g | b);
}

//***********************************
//Continue to draw this
//***********************************
void DrawGyroDisplay()
{
  displayBuffer.fillScreen(BLACK);
  displayBuffer.setTextColor(PINK);
  displayBuffer.setCursor(10, 1);
  displayBuffer.println("Kira Lights");
  displayBuffer.setTextColor(WHITE);
  displayBuffer.setCursor(0, 15);
  displayBuffer.println("Gyro Mode");
  displayBuffer.setCursor(0, 30);
  displayBuffer.printf("Bat:\n  V: %.3fv", M5.Axp.GetBatVoltage());
  if (m5.axp.GetBatChargeCurrent())
  {
    displayBuffer.setTextColor(YELLOW);
    displayBuffer.printf("\n CV: %.3fv", M5.Axp.GetBatChargeCurrent() / 10);
  }
  displayBuffer.setTextColor(WHITE);
  displayBuffer.setCursor(0, 60);
  displayBuffer.printf("  G: %.3f", sigmaGyro);
  //displayBuffer.printf("\n  B: %d " , brightness);
  displayBuffer.printf("\n  B: %d", newHSVColor.value);
  displayBuffer.setCursor(90, 60);
  DrawBitImage(Image_bits, 4, 0, 90);
  displayBuffer.pushSprite(0, 0);
}

void DrawColorDisplay()
{
  displayBuffer.fillScreen(BLACK);
  displayBuffer.setTextColor(BLUE);
  displayBuffer.setCursor(10, 1);
  displayBuffer.println("Kira Lights");
  displayBuffer.setTextColor(WHITE);
  displayBuffer.setCursor(0, 15);
  displayBuffer.println("Color Mode");
  displayBuffer.setCursor(0, 30);
  displayBuffer.printf("Bat:\n  V: %.3fv", M5.Axp.GetBatVoltage());
  if (m5.axp.GetBatChargeCurrent())
  {
    displayBuffer.setTextColor(YELLOW);
    displayBuffer.printf("\n CV: %.3fv", M5.Axp.GetBatChargeCurrent() / 10);
  }
  displayBuffer.setTextColor(WHITE);
  displayBuffer.setCursor(0, 60);
  displayBuffer.printf("  H: %d", newHSVColor.hue);
  //displayBuffer.printf("\n  B: %d " , brightness);
  displayBuffer.printf("\n  B: %d", newHSVColor.value);
  displayBuffer.setCursor(90, 60);
  DrawBitImage(Image_bits, 1, 2, 90);
  DrawBitImage(Image_bits, 2, 30, 90);

  displayBuffer.pushSprite(0, 0);
}

void DrawDebugDisplay()
{
  displayBuffer.fillScreen(BLACK);
  displayBuffer.setTextColor(GREEN);
  displayBuffer.setCursor(10, 1);
  displayBuffer.println("Kira Lights");
  displayBuffer.setTextColor(WHITE);
  displayBuffer.setCursor(0, 15);
  displayBuffer.println("Debug Mode");
  DrawBitImage(Image_bits, 1, 1, 25);
  DrawBitImage(Image_bits, 1, 21, 25);
  DrawBitImage(Image_bits, 1, 41, 25);

  if (deviceConnected)
  {
    displayBuffer.setCursor(1, 60);
    displayBuffer.setTextColor(BLUE);
    displayBuffer.println("BLE Active!");
  }

  //displayBuffer.drawLine(30,0,30,80,CrgbtoHex(newHSVColor));

  displayBuffer.pushSprite(0, 0);
}

//***********************************
//Add all gyroaxis to a collective sigma
//***********************************
void ReadGyro()
{
  M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
  sigmaGyro = abs(gyroX) + abs(gyroY) + abs(gyroZ);
}

//***********************************
//Increase LED brightness on motion
//***********************************
void MotionFlareTask(void *pvParameters)
{
  while (1)
  {
    if (sigmaGyro > flareThreshold && brightness < 200)
    {
      brightness += flareUp;
    }
    else if (brightness > dropoff)
    {
      brightness -= dropoff;
    }
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

void FlareCoolDown(void *pvParameters)
{
  while (1)
  {
      if (brightness > dropoff)
      {
        //brightness = Basebrightness;
        //hue = baseHue;
        brightness -= dropoff;
      }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void SetBrightnessToHSV()
{
  if (brightness < 255)
  {
    newHSVColor.value = brightness;
  }
  else
  {
    newHSVColor.value = 255;
    brightness = 255;
  }

  fill_solid(leds, NUM_LEDS, newHSVColor);
  FastLED.show();

  // if (hue < 255)
  // {
  //   newHSVColor.hue = hue;
  // }else
  // {
  //   newHSVColor.hue = 210;
  // }
}

void ColorPickerButton()
{

  if (m5.BtnB.read())
  {
    newHSVColor.hue += 8;
  }
  while (m5.BtnB.read())
  {
  }
}

void fadeall()
{
  for (int i = 0; i < NUM_LEDS; i++)
  {
    leds[i].nscale8(250);
  }
}



void PatternCylon(void *pvParameters)
{
  while (true)
  {
    static uint8_t hue = 0;
    static int cyloni = 0;
    static bool cylonforward = true;

      fadeall();
      leds[cyloni] = CHSV(hue++, 255, 160);
      FastLED.show();
      leds[cyloni] = CRGB::Black;

    if (cylonforward == true)
    {
      cyloni++;
    }else
    {
      cyloni --;
    }
      Serial.println(cyloni);
    
    if (cyloni >= (2*8) && cylonforward == true)
    {
      Serial.println("reversing Direction");
      cylonforward = false;
    }else if (cyloni == 0 && cylonforward == false)
    {
      Serial.println("Forward Direction"); 
      cylonforward = true;
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

//***********************************
//Setup is called first
//***********************************
void setup()
{
  Serial.begin(115200);
  M5.begin();
  M5.Axp.EnableCoulombcounter();
  StartServer();
  M5.Imu.Init();
  displayBuffer.createSprite(80, 160);

  appMode = MENU_GYRO;

  // Neopixel initialization
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);

  xTaskCreate(MotionFlareTask, "MotionFlare", 1024, NULL, 2, &MotionFlareTaskHandle);
  //xTaskCreate(FlareCoolDown, "FlareCoolDown", 1024, NULL, 2, &FlareCoolDownHandle);
  xTaskCreate(PatternCylon, "PatternCylonTask", 1024, NULL, 2, &PatternCylonHandle);

  //vTaskSuspend(PatternCylonHandle);
  vTaskSuspend(MotionFlareTaskHandle);
  //vTaskSuspend(FlareCoolDownHandle);
}

//***********************************
//Loop
//***********************************
void loop()
{
  m5.BtnA.read();
  m5.BtnB.read();

  switch (appMode)
  {
  case MENU_COLOR_PICKER:
    DrawColorDisplay();
    ColorPickerButton();
    //ChangeLEDColor();

    if (m5.BtnA.read())
    {
      appMode = MENU_GYRO;
      while (m5.BtnA.read() == HIGH)
        ;
    }

    break;

  case MENU_GYRO:
    DrawGyroDisplay();
    SetBrightnessToHSV();
    ReadGyro();
    ColorPickerButton();

    if (m5.BtnA.read())
    {
      appMode = MENU_DEBUG;
      while (m5.BtnA.read() == HIGH)
        ;
    }
    break;

  case MENU_DEBUG:
    DrawDebugDisplay();

    if (m5.BtnA.read())
    {
      appMode = MENU_COLOR_PICKER;
      while (m5.BtnA.read() == HIGH)
        ;
    }
    break;

  default:
    break;
  }
}
