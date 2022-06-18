# Arduino compact faceting head

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
+unsigned int encCal = 4096;            // Encoder counts at calibration point (Note: that one is specific for each faceting arm!)
-float degPerStep = 0.090;              // The number of degrees per encoder step (1000 counts)
+float degPerStep = 0.04678;              // The number of degrees per encoder step (1000 counts)

@@ -64,7 +89,7 @@ int lastCalibState = false;

 // Set the LCD address  for a 20 chars and 4 line display
-LiquidCrystal_I2C lcd(0x3F, 20, 4);
+LiquidCrystal_I2C lcd(0x27, 20, 4);
```

So tweak those settings and match them to your hardware. The ones above are tuned for [the HS25 optical encoder from Sensata](https://www.digikey.com.au/en/products/detail/sensata-bei-sensors/HS25F-62-R10-BS-1024-ABZC-15V-V-SM12-S/7071760?s=N4IgjCBcoLQBxVAYygMwIYBsDOBTANCAPZQDa4A7AEwAsIAugL6OFVkgASAylQKwBiMAGxUYAJTAAGGACEuMKbRgBBGQC0Awgt4A1APQ6YXALJhRXBoyA) 

The initial script used a rather cheap and modular variant like this one: [1000 line rotary incremental encoder kit](https://www.aliexpress.com/item/3256803587186474.html?spm=a2g0o.productlist.0.0.61bd7217SrJ8T5&algo_pvid=3db81b70-310e-43eb-8980-5ba0656321ac&algo_exp_id=3db81b70-310e-43eb-8980-5ba0656321ac-0&pdp_ext_f=%7B%22sku_id%22%3A%2212000027117606147%22%7D&pdp_npi=2%40dis%21USD%21%2119.9%21%21%21%21%21%402100bb4916555490493305222eec40%2112000027117606147%21sea)

Also the LCD display can have I2C addresses `0x3F` or `0x27`, YMMV.
