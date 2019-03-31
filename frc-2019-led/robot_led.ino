#include <Wire.h>
#include <FastLED.h>

#define LED_PIN 3
#define LED_PIN_L 5
#define LED_COUNT 28

CRGB leds[LED_COUNT];

enum LedMode : uint8_t
{
    k_Idle = 0,
    k_NoTarget = 1,
    k_HasTarget = 2,
    k_BallIntake = 3,
    k_Climb = 4
};

LedMode ledMode = k_Idle;

void setup()
{
    FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, LED_COUNT);
    FastLED.addLeds<WS2812B, LED_PIN_L, GRB>(leds, LED_COUNT);
    FastLED.setBrightness(50);
    Wire.begin(1);
    Wire.onReceive(onReceive);
}

void setAll(const CRGB &color)
{
    for (int i = 0; i < LED_COUNT; i++)
    {
        leds[i] = color;
    }
}

void loop()
{
    switch (ledMode)
    {
    default:
    case k_Idle:
    {
        uint8_t hue = beatsin8(8, 0, 255);
        for (int i = 0; i < LED_COUNT; i++) {
            CHSV meme(random8(255), 255, 255);
            leds[i] = meme;
        }
        // setAll(CHSV(hue, 255, 255));
        delay(2000);
        FastLED.show();
        break;
    }
    case k_NoTarget:
    {
        setAll(CRGB::Red);
        FastLED.show();
        delay(600);
        setAll(CRGB::Black);
        FastLED.show();
        delay(400);
        break;
    }
    case k_HasTarget:
    {
        CRGB green(CRGB::Green);
        uint8_t fadeAmount = beatsin8(20, 0, 64);
        green.fadeToBlackBy(fadeAmount);
        setAll(green);
        FastLED.show();
        break;
    }
    case k_BallIntake:
    {
        uint8_t leadLed = beatsin8(10, 0, LED_COUNT);
        leds[leadLed] = CRGB::Magenta;
        FastLED.show();
        fadeToBlackBy(leds, LED_COUNT, 2);
        break;
    }
    case k_Climb:
    {
        CRGB magenta(CRGB::Magenta);
        uint8_t leadLed = beatsin8(20, 0, LED_COUNT),
                fadeAmount = beatsin8(25, 0, 64);
        magenta.fadeToBlackBy(fadeAmount);
        leds[leadLed] = magenta;
        FastLED.show();
        EVERY_N_MILLIS(20)
        {
            fadeToBlackBy(leds, LED_COUNT, 48);
        }
        break;
    }
    }
}

void onReceive(int count)
{
    ledMode = (LedMode)Wire.read();
}