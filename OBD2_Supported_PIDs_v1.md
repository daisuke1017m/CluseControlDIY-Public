# OBD-II PIDs Scan Results (Service 01)

This file contains the list of supported OBD-II PIDs under Service 01 (Show current data) based on your provided diagnostic scan log. The table layout follows the Wikipedia OBD-II PIDs standard format.

| PID (Hex) | Data Bytes | Description | Min Value | Max Value | Unit | Formula |
| :---: | :---: | :--- | :---: | :---: | :---: | :--- |
| 01 | 4 | Monitor status since DTCs cleared. (Includes malfunction indicator lamp (MIL) status and number of DTCs.) |  |  |  | Bit encoded |
| 03 | 2 | Fuel system status |  |  |  | Bit encoded |
| 04 | 1 | Calculated engine load | 0 | 100 | % | A \times \frac{100}{255} |
| 05 | 1 | Engine coolant temperature | -40 | 215 | °C | A - 40 |
| 06 | 1 | Short term fuel trim—Bank 1 | -100 | 99.2 | % | (A - 128) \times \frac{100}{128} |
| 07 | 1 | Long term fuel trim—Bank 1 | -100 | 99.2 | % | (A - 128) \times \frac{100}{128} |
| 0B | 1 | Intake manifold absolute pressure | 0 | 255 | kPa | A |
| 0C | 2 | Engine speed (RPM) | 0 | 16383.75 | rpm | \frac{256A + B}{4} |
| 0D | 1 | Vehicle speed | 0 | 255 | km/h | A |
| 0E | 1 | Timing advance | -64 | 63.5 | ° before TDC | \frac{A}{2} - 64 |
| 0F | 1 | Intake air temperature | -40 | 215 | °C | A - 40 |
| 10 | 2 | Mass air flow (MAF) air flow rate | 0 | 655.35 | g/s | \frac{256A + B}{100} |
| 11 | 1 | Throttle position | 0 | 100 | % | A \times \frac{100}{255} |
| 12 | 1 | Commanded secondary air status |  |  |  | Bit encoded |
| 13 | 1 | Oxygen sensors present (in 2 banks) |  |  |  | Bit encoded |
| 15 | 2 | Oxygen sensor 2 (Voltage & Short term fuel trim) | 0 / -100 | 1.275 / 99.2 | V / % | A \times 0.005 \text{ and } (B-128) \times \frac{100}{128} \text{ (if } B=0xFF\text{, sensor not used)} |
| 1C | 1 | OBD standards this vehicle conforms to |  |  |  | Bit encoded |
| 1F | 2 | Run time since engine start | 0 | 65535 | seconds | 256A + B |
| 20 | 4 | PIDs supported [21 — 40] |  |  |  | Bit encoded |
| 21 | 2 | Distance traveled with malfunction indicator lamp (MIL) on | 0 | 65535 | km | 256A + B |
| 24 | 4 | Oxygen Sensor 1 (Air-Fuel Equivalence Ratio & Voltage) | 0 / 0 | 1.999 / 7.999 | ratio / V | \frac{2}{65536}(256A + B) \text{ and } \frac{8}{65536}(256C + D) |
| 2E | 1 | Commanded evaporative purge | 0 | 100 | % | A \times \frac{100}{255} |
| 2F | 1 | Fuel Tank Level Input | 0 | 100 | % | A \times \frac{100}{255} |
| 30 | 1 | Warm-ups since codes cleared | 0 | 255 | count | A |
| 31 | 2 | Distance traveled since codes cleared | 0 | 65535 | km | 256A + B |
| 33 | 1 | Absolute Barometric Pressure | 0 | 255 | kPa | A |
| 34 | 4 | Oxygen Sensor 1 (Air-Fuel Equivalence Ratio & Current) | 0 / -128 | 1.999 / 127.99 | ratio / mA | \frac{2}{65536}(256A + B) \text{ and } (256C + D)/256 - 128 |
| 3C | 2 | Catalyst Temperature: Bank 1, Sensor 1 | -40 | 6513.5 | °C | \frac{256A + B}{10} - 40 |
| 40 | 4 | PIDs supported [41 — 60] |  |  |  | Bit encoded |
| 41 | 4 | Monitor status this drive cycle |  |  |  | Bit encoded |
| 42 | 2 | Control module voltage | 0 | 65.535 | V | \frac{256A + B}{1000} |
| 43 | 2 | Absolute load value | 0 | 25700 | % | \frac{256A + B \times 100}{255} |
| 44 | 2 | Fuel–Air commanded equivalence ratio | 0 | 1.999 | ratio | \frac{2(256A + B)}{65536} |
| 45 | 1 | Relative throttle position | 0 | 100 | % | A \times \frac{100}{255} |
| 46 | 1 | Ambient air temperature | -40 | 215 | °C | A - 40 |
| 47 | 1 | Absolute throttle position B | 0 | 100 | % | A \times \frac{100}{255} |
| 49 | 1 | Accelerator pedal position D | 0 | 100 | % | A \times \frac{100}{255} |
| 4A | 1 | Accelerator pedal position E | 0 | 100 | % | A \times \frac{100}{255} |
| 4C | 1 | Commanded throttle actuator | 0 | 100 | % | A \times \frac{100}{255} |
| 4D | 2 | Time run with MIL on | 0 | 65535 | minutes | 256A + B |
| 4E | 2 | Time since trouble codes cleared | 0 | 65535 | minutes | 256A + B |
| 51 | 1 | Fuel Type |  |  |  | Bit encoded |
| 5A | 1 | Relative accelerator pedal position | 0 | 100 | % | A \times \frac{100}{255} |
