# Arduino ompact faceting head

Arduino sketch for the compact faceting head's rotation sensor, display, depth of cut indicator and RPM sensor. 

[Watch GemWorks on Youtube for more context on what this is used for](https://www.youtube.com/c/GemWorks)

# Libraries used

1. `LiquidCrystal_I2C` for the LCD display: https://github.com/johnrickman/LiquidCrystal_I2C, do not confuse with the more supported [LiquidCrystal library](https://www.arduino.cc/reference/en/libraries/liquidcrystal/).
2. ISR Timer Interrupts for the RPM hall sensor: [https://github.com/khoih-prog/TimerInterrupt](https://github.com/khoih-prog/TimerInterrupt).

# Common modifications

Here's how you can adapt this sketch to your own (most common) differences w.r.t encoders and I2C LCD display drivers:

```diff
 // Rotary Encoder
-unsigned int encCal = 2048;            // Encoder counts at calibration point
+unsigned int encCal = 4096;            // Encoder counts at calibration point
-float degPerStep = 0.090;              // The number of degrees per encoder step (1000 counts)
+float degPerStep = 0.04678;              // The number of degrees per encoder step (1000 counts)

@@ -64,7 +89,7 @@ int lastCalibState = false;

 // Set the LCD address  for a 20 chars and 4 line display
-LiquidCrystal_I2C lcd(0x3F, 20, 4);
+LiquidCrystal_I2C lcd(0x27, 20, 4);
```

So tweak those settings and match them to your hardware. The ones above are tuned for [the HS25 optical encoder from Sensata](https://www.digikey.com.au/en/products/detail/sensata-bei-sensors/HS25F-62-R10-BS-1024-ABZC-15V-V-SM12-S/7071760?s=N4IgjCBcoLQBxVAYygMwIYBsDOBTANCAPZQDa4A7AEwAsIAugL6OFVkgASAylQKwBiMAGxUYAJTAAGGACEuMKbRgBBGQC0Awgt4A1APQ6YXALJhRXBoyA) 

Also the LCD display can have I2C addresses `0x3F` or `0x27`, YMMV.
