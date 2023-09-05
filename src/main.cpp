#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif
// #define PIN1 3
#define PIN1 D4
#include <NTPClient.h>
#include <TimeLib.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

// const char *ssid     = "OW";
// const char *password = "xxxxxxxx";
const char ssid[] = "OW";
const char pass[] = "xxxxxxxx";

// NTP Servers:
static const char ntpServerName[] = "us.pool.ntp.org";
// static const char ntpServerName[] = "time.nist.gov";
// static const char ntpServerName[] = "time-a.timefreq.bldrdoc.gov";
// static const char ntpServerName[] = "time-b.timefreq.bldrdoc.gov";
// static const char ntpServerName[] = "time-c.timefreq.bldrdoc.gov";

const int timeZone = 2; // Central European Time
// const int timeZone = -5;  // Eastern Standard Time (USA)
// const int timeZone = -4;  // Eastern Daylight Time (USA)
// const int timeZone = -8;  // Pacific Standard Time (USA)
// const int timeZone = -7;  // Pacific Daylight Time (USA)

WiFiUDP Udp;

unsigned int localPort = 8888; // local port to listen for UDP packets

NTPClient timeClient(Udp, "pool.ntp.org", 10800, 3600);

/*-------- NTP code ----------*/

// const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; // buffer to hold incoming & outgoing packets

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
    // set all bytes in the buffer to 0
    memset(packetBuffer, 0, NTP_PACKET_SIZE);
    // Initialize values needed to form NTP request
    // (see URL above for details on the packets)
    packetBuffer[0] = 0b11100011; // LI, Version, Mode
    packetBuffer[1] = 0;          // Stratum, or type of clock
    packetBuffer[2] = 6;          // Polling Interval
    packetBuffer[3] = 0xEC;       // Peer Clock Precision
    // 8 bytes of zero for Root Delay & Root Dispersion
    packetBuffer[12] = 49;
    packetBuffer[13] = 0x4E;
    packetBuffer[14] = 49;
    packetBuffer[15] = 52;
    // all NTP fields have been given values, now
    // you can send a packet requesting a timestamp:
    Udp.beginPacket(address, 123); // NTP requests are to port 123
    Udp.write(packetBuffer, NTP_PACKET_SIZE);
    Udp.endPacket();
}

void printDigits(int digits)
{
    // utility for digital clock display: prints preceding colon and leading 0
    Serial.print(":");
    if (digits < 10)
        Serial.print('0');
    Serial.print(digits);
}

void digitalClockDisplay()
{
    // digital clock display of the time
    Serial.print(hour());
    printDigits(minute());
    printDigits(second());
    Serial.print(" ");
    Serial.print(day());
    Serial.print(".");
    Serial.print(month());
    Serial.print(".");
    Serial.print(year());
    Serial.println();
}

time_t getNtpTime()
{
    IPAddress ntpServerIP; // NTP server's ip address

    while (Udp.parsePacket() > 0)
        ; // discard any previously received packets
    Serial.println("Transmit NTP Request");
    // get a random server from the pool
    WiFi.hostByName(ntpServerName, ntpServerIP);
    Serial.print(ntpServerName);
    Serial.print(": ");
    Serial.println(ntpServerIP);
    sendNTPpacket(ntpServerIP);
    uint32_t beginWait = millis();
    while (millis() - beginWait < 1500)
    {
        int size = Udp.parsePacket();
        if (size >= NTP_PACKET_SIZE)
        {
            Serial.println("Receive NTP Response");
            Udp.read(packetBuffer, NTP_PACKET_SIZE); // read packet into the buffer
            unsigned long secsSince1900;
            // convert four bytes starting at location 40 to a long integer
            secsSince1900 = (unsigned long)packetBuffer[40] << 24;
            secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
            secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
            secsSince1900 |= (unsigned long)packetBuffer[43];
            return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
        }
    }
    Serial.println("No NTP Response :-(");
    return 0; // return 0 if unable to get the time
}

time_t getNtpTime();
void digitalClockDisplay();
void printDigits(int digits);
void sendNTPpacket(IPAddress &address);

// #include <iarduino_RTC.h>                                               // Подключаем библиотеку iarduino_RTC для работы с модулями реального времени.
// iarduino_RTC rtc(RTC_DS1302, 8, 9, 10);
unsigned long currentTime;
unsigned long blinkTime;
boolean blinker = 1;
uint8_t D, M, Y, h, m, s, W; // Объявляем переменные для получения следующих значений: D-день, M-месяц, Y-год, h-часы, m-минуты, s-секунды, W-день недели.
uint8_t DD, MM, YY, hh, mm, ss, WW;
#define TIME_HEADER 'T' // Header tag for serial time sync message

#define HOUR 'H'
#define MIN 'I'
#define SEC 'S'
#define DAY 'D'
// #define MONTH             'M'
// #define YEAR              'Y'

#define BRIGHTNESS_HEADER 'B'

int8_t brightness_Value = 4;

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel strip1 = Adafruit_NeoPixel(60, PIN1, NEO_GRB + NEO_KHZ800);

// byte s3[8] = {0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00};
// long xxx = 0b111111000111111l;
long theDigits[11] =
    {
        0b111111000111111, // 0
        0b000001111100010, // 1
        0b101111010111101, // 2
        0b111111010110101, // 3
        0b111110010000111, // 4
        0b111011010110111, // 5
        0b111011010111111, // 6
        0b111110000100001, // 7
        0b111111010111111, // 8
        0b111111010110111, // 9
        0b000000000000000, // empty
                           //  0b01110,
                           //  0b00000,
                           //  0b00000,
};
// IMPORTANT: To reduce NeoPixel burnout risk, add 1000 uF capacitor across
// pixel power leads, add 300 - 500 Ohm resistor on first pixel's data input
// and minimize distance between Arduino and first pixel.  Avoid connecting
// on a live circuit...if you must, connect GND first.

const char *strM = "JanFebMarAprMayJunJulAugSepOctNovDec"; // Определяем массив всех вариантов текстового представления текущего месяца находящегося в предопределенном макросе __DATE__.
const char *sysT = __TIME__;                               // Получаем время компиляции скетча в формате "SS:MM:HH".
const char *sysD = __DATE__;                               // Получаем дату  компиляции скетча в формате "MMM:DD:YYYY", где МММ - текстовое представление текущего месяца, например: Jul.
//  Парсим полученные значения в массив:                    // Определяем массив «i» из 6 элементов типа int, содержащий следующие значения: секунды, минуты, часы, день, месяц и год компиляции скетча.
const int i[6]{(sysT[6] - '0') * 10 + (sysT[7] - '0'), (sysT[3] - '0') * 10 + (sysT[4] - '0'), (sysT[0] - '0') * 10 + (sysT[1] - '0'), (sysD[4] - '0') * 10 + (sysD[5] - '0'), ((int)memmem(strM, 36, sysD, 3) + 3 - (int)&strM[0]) / 3, (sysD[9] - '0') * 10 + (sysD[10] - '0')};

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos)
{
    WheelPos = 255 - WheelPos;
    if (WheelPos < 85)
    {
        return strip1.Color(255 - WheelPos * 3, 0, WheelPos * 3);
    }
    if (WheelPos < 170)
    {
        WheelPos -= 85;
        return strip1.Color(0, WheelPos * 3, 255 - WheelPos * 3);
    }
    WheelPos -= 170;
    return strip1.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

// Fill the dots one after the other with a color
void colorWipe(uint8_t wait)
{

    for (uint16_t i = 0; i < strip1.numPixels(); i++)
    {

        strip1.setPixelColor(i, Wheel(i));
        strip1.show();
        delay(wait);
    }
}

void showSymbol(int sym, int placeNum)
{
    int binArr = theDigits[sym];
    for (uint16_t i = 0; i < 60; i++)
    {
        byte x = bitRead(binArr, i);
        if (x > 0)
        {
            strip1.setPixelColor(i + placeNum * 15, strip1.Color(0, 200, 255));
        }
        else
        {
            strip1.setPixelColor(i + placeNum * 15, strip1.Color(0, 0, 30));
        }
        //    strip1.setPixelColor(s, strip1.Color( 255, 0, 0));
        //    strip1.show();
    }
}

void processSyncMessage()
{
    int8_t value;
    char c = Serial.read();
    Serial.println(c);
    switch (c)
    {
    case HOUR:
        value = Serial.parseInt();
        //   rtc.settime(-1, -1, value);
        Serial.println("HOUR");
        Serial.println(value);
        break;
    case MIN:
        value = Serial.parseInt();
        //   rtc.settime(-1, value);
        Serial.println("MIN");
        Serial.println(value);
        break;
    case SEC:
        value = Serial.parseInt();
        //   rtc.settime( value);
        Serial.println("SEC");
        Serial.println(value);
        break;
    case DAY:
        value = Serial.parseInt();
        //   rtc.settime(-1, -1, -1, value);
        Serial.println("DAY");
        Serial.println(value);
        break;
    // case MONTH:
    //   value = Serial.parseInt();
    //   rtc.settime(-1, -1, -1, -1, value);
    //   Serial.println("MONTH");
    //   Serial.println(value);
    //   break;
    // case YEAR:
    //   value = Serial.parseInt();
    //   rtc.settime(-1, -1, -1, -1, -1, value);
    //   Serial.println("YEAR");
    //   Serial.println(value);
    //   break;
    default:
        // выполнить, если val ни 1 ни 2
        // default опционален
        break;
    }
    //  unixTime = rtc.gettimeUnix();
    //  Serial.println(unixTime);
    //  setTime(unixTime);
}

void processBrightnessMessage()
{
    //  int8_t value;
    brightness_Value = Serial.parseInt();
    Serial.println(brightness_Value);
    strip1.setBrightness(brightness_Value);
    //  analogWrite(LCD_BACKLIGHT_PIN, brightness_Value); //set backlight brightness
}

void setup()
{
    Serial.begin(115200);
    Serial.println("sysT");
    Serial.println(sysT);
    Serial.println(sysD);
    Serial.println((String)i[0] + ":" + i[1] + ":" + i[2] + ":" + i[3] + ":" + i[4] + ":" + i[5]);
    // This is for Trinket 5V 16MHz, you can remove these three lines if you are not using a Trinket
#if defined(__AVR_ATtiny85__)
    if (F_CPU == 16000000)
        clock_prescale_set(clock_div_1);
#endif
    // End of trinket special code
    //   rtc.begin();
    //   rtc.period(600);
    strip1.begin();
    strip1.setBrightness(brightness_Value);
    strip1.show(); // Initialize all pixels to 'off'

    colorWipe(40); // Red

    WiFi.begin(ssid, pass);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    timeClient.begin();
}
int place = 0;
int iii = 0;
uint32_t timer2 = millis();
void loop()
{
    timeClient.update();

    // Serial.println(timeClient.getFormattedTime());
    if (timeClient.isTimeSet())
    {

        h = timeClient.getHours();   // Получаем из буфера библиотеки текущие часы        0-23.
        m = timeClient.getMinutes(); // Получаем из буфера библиотеки текущие минуты      0-59.
        s = timeClient.getSeconds(); // Получаем из буфера библиотеки текущие секунды     0-59.
        W = timeClient.getDay();
    }

    //  if (millis() - currentTime > 990) {
    //    currentTime = millis();
    //   rtc.gettime();                                                  // Считываем текущее время из модуля в буфер библиотеки.
    //   D = rtc.day;                                                    // Получаем из буфера библиотеки текущий день месяца 1-31.
    //   M = rtc.month;                                                  // Получаем из буфера библиотеки текущий месяц       1-12.
    //   Y = rtc.year;                                                   // Получаем из буфера библиотеки текущий год         0-99.

    //    Serial.println((String) D + "-" + M + "-" + Y + ", " + h + ":" + m + ":" + s + ", " + W); // Выводим время в монитор, одной строкой.

    //  if (millis() - currentTime > 990) {
    //  currentTime = millis();
    //
    if (ss != s && millis() - timer2 > 1000)
    {
        timer2 = millis();
        ss = s;
        //    Serial.println((String) D + "-" + M + "-" + Y + ", " + h + ":" + m + ":" + s + ", " + W);

        showSymbol(round(h / 10), 0);
        showSymbol(h % 10, 1);
        showSymbol(round(m / 10), 2);
        showSymbol(m % 10, 3);
        // Serial.println((String)round(h / 10) + "" + h % 10 + ":" + round(m / 10) + "" + m % 10 + ":" + s);
        strip1.setPixelColor(s, strip1.Color(255, 0, 0));
        strip1.show();
    }

    if (Serial.available() > 2)
    { // wait for at least two characters
        char c = Serial.read();
        Serial.print(" - c - ");
        Serial.println(c);
        if (c == TIME_HEADER)
        {
            processSyncMessage();
        }
        else if (c == BRIGHTNESS_HEADER)
        {
            processBrightnessMessage();
        }
    }
}
