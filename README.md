# bloom-chandelier
Interactive chandelier that lights and sounds respond to people approaching, using time-of-flight sensors, an ESP32, and a Raspberry Pi.

## Demo

▶️ **[Watch it in action on Instagram](https://www.instagram.com/p/DaQSBc3xLLd/)**

## How it works

- An **ESP32** reads the 8 sensors, drives the LED strip, and streams distance
  data over USB serial.
- A **Raspberry Pi 4** listens to that serial stream and synthesizes the audio,
  routed out through a USB DAC to a powered speaker.

### The interaction

Each of the 8 sensors owns a zone of the LED ring and a musical note, a C
major pentatonic scale spread across two octaves. When something enters a
sensor's range around 1500 mm:

1. **Light:** the zone lights up, brightness mapped to distance, dim far and
   bright near, following a `closeness^2` curve so the swell is pronounced.
   The zone fills in LED by LED at a fixed rate from the middle outward if no
   neighboring zone is currently lit, or from the side of an already lit
   neighbor so the light appears to flow around the ring.
2. **Sound:** that zone's note swells in volume, using the same `closeness^2`
   curve so it stays perfectly in sync with the light.
3. **Fade:** when you move away, both light and sound fade out slowly, leaving
   a trail as you move to the next zone.

### Avoiding sensor interference

All 8 VL53L1X sensors share the same I2C address, 0x29, so they're connected
through a PCA9548 8 channel I2C multiplexer. The firmware selects one channel
at a time and reads the sensors round robin, so only one is on the bus at any
moment. That plus aiming the sensors outward in a circle so their IR cones
don't overlap avoids both electrical and optical interference.

## Hardware

- **ESP32**. Adafruit HUZZAH32 Feather
- **8× VL53L1X**  time of flight distance sensors (STEMMA QT)
- **PCA9548 / TCA9548**  8-channel I2C multiplexer (address 0x70)
- **DotStar LED strip**  60 LED per m, warm white, trimmed to 59 LEDs, mounted on
  a ring hung inside the chandelier among the crystals
- **Raspberry Pi 4B**. runs the audio synthesis
- **Khadas Tone Board**  USB DAC
- **Powered speaker** (line-in / aux)
- Separate 5V supply for the LED strip

### Wiring notes

- ESP32 pin 18 -> strip (data in), pin 5 → strip (clock in).
  Connect to the strip's input end (DI/CI)
- Sensors connect to the mux via STEMMA QT; the mux connects to the ESP32's
  I2C pins. STEMMA colors: red=3V, black=GND, blue=SDA, yellow=SCL.
- The LED strip needs its own 5V supply. The strip's ground, the supply
  ground, and the ESP32 ground must all be common.
- ESP32 -> Pi over USB (serial, 115200 baud).
- Pi -> Tone Board over USB (data cable). Tone Board → speaker over RCA to 3.5mm.


### Dependencies

**ESP32 (Arduino IDE):**
- Adafruit VL53L1X
- Adafruit DotStar
- TCA9548 (by Rob Tillaart)
- ESP32 board support:
  `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`

**Raspberry Pi (Python 3):**


### Sound samples

The audio uses four pad samples (C2, C3, C4, C5) as pitch anchors. I used
pads from [Pianobook](https://www.pianobook.co.uk/). Source your own samples
and update the file paths and `ZONE_NOTES` in the Pi script. The script
pitch-shifts these anchors by small intervals to build an 8 note C major
pentatonic and processes each into a seamless loop.

### Configuration

- `SERIAL_PORT` in the Pi script check with `ls /dev/ttyUSB*`, it can change.
- `OUTPUT_DEVICE_INDEX` run the script once, note the printed device list,
  set it to your DAC's index.
- `MASTER_GAIN` overall volume, tune by ear.
- Zone mapping (`ZONE_STARTS` / `ZONE_SIZES` in the ESP32) is specific to how
  the strip physically lands on the ring, you'll need to remap for your build.

## Notes

This was built iteratively with a lot of real-world debugging, sensor
placement, ambient IR effects, seamless audio looping, light/sound sync. The
tuning constants reflect one particular physical build; expect to adjust them
for yours.
