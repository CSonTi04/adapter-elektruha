# Safety & Disclaimer

> **This project is for educational and experimental use only.**
> It is provided **"AS IS"**, with **no warranty** and **no liability** of any kind.
> See [LICENSE](LICENSE) for full terms.

---

## Common hardware risks — read before wiring

### 1. ESP32 GPIOs are 3.3 V — not 5 V tolerant

The ESP32's I/O pins are rated for **3.3 V logic**.
Connecting a 5 V I2C bus (SDA/SCL) directly to ESP32 GPIO pins **can permanently
damage the chip**.

- If your PCF8574 module runs at **5 V**, use a **logic-level shifter** on SDA and SCL,
  or power the module from **3.3 V** instead.
- Always verify the VCC/logic voltage of every module before wiring it up.

### 2. Check VCC levels and logic compatibility

Before powering anything on:

- Confirm that **every module's VCC pin** receives the correct voltage.
- Check that all logic signals operate at the **same voltage level**, or use a shifter.
- The PCF8574 supports 2.5 V – 6 V supply; operating at 3.3 V keeps it directly
  compatible with the ESP32.

### 3. I2C pull-up resistors and correct wiring

I2C requires pull-up resistors on **both SDA and SCL**:

- Typical values: **4.7 kΩ** to the supply rail (3.3 V in this project).
- Without pull-ups the bus may appear to work intermittently, or not at all.
- Double-check pin assignments: in this project `SDA = GPIO21`, `SCL = GPIO22`.

### 4. Avoid shorts, reversed polarity, and multiple power sources

- **Shorts** (wires touching unintended pins) can destroy components instantly.
- **Reversed polarity** on VCC/GND destroys most ICs immediately.
- **Never power the board from USB and an external supply at the same time** unless
  you have verified that the power rails are properly isolated.
- Always unplug everything before re-wiring.

### 5. Do not drive loads directly from GPIO or PCF8574 pins

ESP32 GPIOs and PCF8574 outputs have very limited current drive capability
(a few milliamps each):

- **Do not** connect motors, solenoids, relays, or other inductive/high-current
  loads directly to these pins.
- Use a proper driver circuit: **transistor**, **MOSFET**, or a **relay module**.
- Always add a **flyback diode** (e.g. 1N4007) across inductive loads (motors,
  relay coils, solenoids) to suppress voltage spikes.

### 6. Current limits and heat

- The ESP32's total GPIO current budget is limited (~40 mA per pin, ~1.2 A total
  across all pins — do not approach these limits in practice).
- LEDs must have **current-limiting resistors** unless a constant-current driver
  is used.
- If any component gets **unexpectedly hot**, power off immediately and investigate.

---

## No warranty — no liability

This software and any associated hardware designs are provided for **educational
and experimental purposes only**.

> THE SOFTWARE AND HARDWARE DESCRIPTIONS ARE PROVIDED "AS IS", WITHOUT WARRANTY
> OF ANY KIND, EXPRESS OR IMPLIED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
> ANY CLAIM, DAMAGES, OR OTHER LIABILITY — INCLUDING BUT NOT LIMITED TO DAMAGE TO
> COMPONENTS, PERSONAL INJURY, OR PROPERTY DAMAGE — ARISING FROM THE USE OF THIS
> MATERIAL.

Use it at your own risk.  If you are unsure about any step, consult someone with
hands-on electronics experience before powering up your build.
