# Simple 4 Wire aRGB Case Fan Controller

A PlatformIO project for controlling a 120mm 4-wire PWM fan with integrated addressable RGB LEDs using an Arduino-compatible microcontroller.

## Hardware

### Components
- **Microcontroller**: SparkFun Pro Micro 5V (ATmega32U4)
- **Fan**: 120mm 4-wire PWM fan with 8 addressable WS2812B LEDs
- **Potentiometer**: Variable resistor for speed control
- **Optional**: Logic-level N-channel MOSFET for complete fan shutoff (recommended)

### Fan Connector Pinouts

#### 4-Wire PWM Fan Connector
Standard 4-pin fan connector pinout:

| Pin No. | Wire Color | Signal | Description |
|---------|------------|--------|-------------|
| 1 | Black | Ground | Common ground |
| 2 | Red | +12V | Fan motor power (constant) |
| 3 | Yellow | Tachometer | RPM feedback signal (2 pulses/rev) |
| 4 | Blue | PWM | PWM speed control input (25kHz) |

**Note**: The PWM signal controls the fan's internal motor driver but does NOT switch power. The fan always has 12V connected, which is why some fans won't completely stop with PWM alone.

#### 3-Pin Addressable RGB Connector (5V)
The aRGB connector powers and controls the addressable LEDs:

| Pin No. | Wire Color | Signal | Description |
|---------|------------|--------|-------------|
| 1 | Black | Ground | Common ground |
| 2 | Red | +5V | LED power supply |
| 3 | White/Green | Data | WS2812B data signal |

**Important**:
- This is a **5V addressable RGB** header, NOT the 12V standard RGB header
- The data pin uses the WS2812B protocol (compatible with FastLED NEOPIXEL)
- Connect +5V to Arduino 5V output or external 5V supply
- Connect Data to Arduino Pin 3

### Current Wiring

| Component | Arduino Pin | Notes |
|-----------|-------------|-------|
| Fan PWM Control | Pin 9 | 25kHz hardware PWM (Timer1) |
| Fan Tachometer | Pin 2 | Interrupt-driven RPM sensing |
| LED Data Line | Pin 3 | 8x WS2812B addressable LEDs |
| Potentiometer | A0 | Analog input for speed control |
| Fan 12V Power | 12V Supply | Direct connection |
| Fan Ground | GND | Direct connection |

### Known Issue: Fan Won't Stop Completely

**Problem**: 4-wire PWM fans maintain constant 12V power, and the PWM signal only controls the internal motor driver. Many fans have minimum operating speeds or internal pull-ups that prevent complete shutoff via PWM alone.

**Solution**: Add low-side MOSFET switching to cut power completely when fan should be off.

## Proposed MOSFET Circuit (Recommended)

### Low-Side Switching Configuration

```
                     +12V Power Supply
                           |
                           |
                    Fan 12V (Red Wire)
                           |
                         [FAN]
                           |
                    Fan GND (Black Wire)
                           |
                           +--- MOSFET Drain
                                    |
                                [MOSFET]  N-Channel Logic-Level
                                    |     (IRLZ44N, IRL540N, etc.)
                                    +--- MOSFET Source
                                           |
                                          GND (Common Ground)

Arduino Pin 10 ---[220Ω]--- MOSFET Gate
                               |
                            [10kΩ] (pull-down)
                               |
                              GND
```

### Component Recommendations
- **MOSFET**: IRLZ44N or IRL540N (logic-level, fully on at 5V)
- **Gate Resistor**: 220Ω - 1kΩ (protects Arduino pin)
- **Pull-down Resistor**: 10kΩ (keeps MOSFET off during startup)

### Why Low-Side Switching?
For an N-channel MOSFET to turn on, the gate voltage must be higher than the source voltage (Vgs). With low-side switching:
- Source is at GND (0V)
- Gate at 5V gives Vgs = 5V (MOSFET fully on)
- Gate at 0V gives Vgs = 0V (MOSFET off)

High-side switching would put the source at 12V, making Vgs = 5V - 12V = -7V, which won't work.

### Enabling MOSFET Control in Code
1. Wire the MOSFET circuit as shown above
2. In `src/main.cpp`, uncomment line 9:
   ```cpp
   #define FAN_POWER_PIN 10
   ```
3. Upload the modified code
4. The fan will now completely shut off when potentiometer is in the lowest 10%

## Features

### Fan Control
- **25kHz PWM frequency** for silent operation (hardware Timer1)
- **Potentiometer-based speed control** with smooth response
- **10% deadzone** at bottom of potentiometer range (fan off)
- **RPM monitoring** via tachometer signal (assumes 2 pulses/revolution)
- **Serial debug output** showing fan speed percentage and RPM

### LED Effects

#### Normal Operation: Rainbow Spiral Breathing
- Rainbow colors spiral around the 8 LED ring
- Breathing effect pulses from 30% to potentiometer-set brightness
- Example: At 70% fan speed, LEDs breathe from 30% to 70% brightness
- Brightness range: 30% (minimum) to 100% (maximum)

#### Alert Mode: Red Flashing
- Triggers when fan speed < 100 RPM while it should be running
- Full brightness red flashing at 1Hz (500ms on, 500ms off)
- Indicates potential fan stall or malfunction

#### Fan Off State
- All LEDs turn off when fan is off (potentiometer < 10%)

## Software Dependencies

- **PlatformIO**: Build system and library management
- **FastLED**: Addressable LED control library (v3.7.0+)

## Installation

1. Clone this repository
2. Open in VSCode with PlatformIO extension installed
3. Connect SparkFun Pro Micro via USB
4. Build and upload: `PlatformIO: Upload`

### Linux Permissions
If you get a permission error on `/dev/ttyACM0`:
```bash
sudo usermod -a -G dialout $USER
```
Then log out and log back in.

## Configuration

### Adjustable Parameters (in `src/main.cpp`)

```cpp
// Fan configuration
#define PWM_FREQ 25000          // 25kHz PWM frequency
#define MIN_RPM_THRESHOLD 100   // RPM below which alert triggers
#define POT_DEADZONE 10         // Bottom 10% is off

// Timing
#define TACH_UPDATE_INTERVAL 1000  // Update RPM every 1 second
#define LED_UPDATE_INTERVAL 20     // Update LEDs at ~50Hz
#define ALERT_BLINK_INTERVAL 500   // 1Hz blink rate

// LED configuration
#define NUM_LEDS 8              // Number of LEDs in fan
```

### LED Brightness Adjustments
The `calculateBrightness()` function ([main.cpp:186-190](src/main.cpp#L186-L190)) maps fan speed to LED brightness (30%-100%). Adjust the range by changing the map values:
```cpp
return map(fanSpeed, 0, 255, 77, 255);  // 77 = 30%, 255 = 100%
```

### Breathing Speed
Adjust the breathing rate in `showRainbowBreathing()` ([main.cpp:206](src/main.cpp#L206)):
```cpp
breathPhase += 0.05;  // Increase for faster breathing
```

### Rainbow Spiral Speed
Adjust the spiral rotation speed ([main.cpp:203](src/main.cpp#L203)):
```cpp
hueOffset += 2;  // Increase for faster color rotation
```

## Technical Details

### PWM Generation
Uses ATmega32U4 Timer1 in Fast PWM mode with ICR1 as TOP:
- Frequency: 25kHz (beyond human hearing)
- Resolution: 640 steps (ICR1 = 639)
- No prescaler (maximum resolution)

### RPM Calculation
```
RPM = (pulses / 2) × 60
```
Assumes standard 4-wire fan with 2 pulses per revolution. Update if your fan differs.

### Pin Usage Notes
- **Pin 9 (OC1A)**: Hardware PWM output, cannot be changed without code modifications
- **Pin 2**: Hardware interrupt capable (required for tachometer)
- **Pin 3**: Any digital pin works for WS2812B data
- **Pin A0**: Any analog pin works for potentiometer

## Serial Monitor Output

Connect at 115200 baud to see real-time fan status:
```
Fan Controller Initialized
Fan Speed: 128 (50%)  RPM: 1250
Fan Speed: 0 (0%)  RPM: 0
Fan Speed: 255 (100%)  RPM: 2500
```

## Future Improvements

- [ ] Add MOSFET circuit for complete fan shutoff
- [ ] Add temperature sensor input for automatic fan control
- [ ] Implement multiple LED pattern modes
- [ ] Add EEPROM storage for user preferences
- [ ] PWM frequency adjustment for different fan models
- [ ] Web interface or Bluetooth control

## Troubleshooting

### Fan won't stop spinning
- This is a known issue with 4-wire PWM fans
- **Solution**: Implement the MOSFET circuit (see above)
- **Alternative**: Some fans have minimum operating speeds by design

### LEDs flicker at low brightness
- Increase minimum brightness in `calculateBrightness()`
- Current minimum is 30% (77/255) which should prevent flickering

### RPM reading is incorrect
- Check tachometer wire connection (yellow wire to Pin 2)
- Verify pulses per revolution for your specific fan (usually 2)
- Some fans output 1 or 4 pulses per revolution

### Permission denied when uploading
- Add user to dialout group: `sudo usermod -a -G dialout $USER`
- Log out and back in for changes to take effect

## License

This project is provided as-is for educational and personal use.

## Author

Developed for controlling aRGB case fans with Arduino-compatible microcontrollers.

## Acknowledgments

- FastLED library by Daniel Garcia
- SparkFun for Pro Micro hardware design
