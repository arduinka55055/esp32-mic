# 🎙️ ESP32-C6 Wi-Fi IPv6 Multicast Microphone

This project implements a real-time microphone streaming system using the ESP32-C6 microcontroller. It captures audio via an I2S MEMS microphone and streams the data over IPv6 multicast using RTP packets via UDP.

Perfect for multi-listener audio broadcasting across local networks or for a wireless wifi microphone for Discord and gaming.

Using pure ESP-IDF, no Arduino libraries needed!


# 📦 Features
- 📡 Connects to Wi-Fi and obtains an IPv6 address
- 🎧 Captures mono 44.1 kHz audio using I2S
- 📤 Encodes audio into 16-bit linear PCM
- 📶 Streams audio via UDP multicast (ff0e::abcd) with RTP headers
- 💡 Uses IPv6 because it's the ~~future~~present and IPv4 just sucks

# 🔧 Hardware
- 🧠 ESP32-C6 dev board
- 🎙️ I2S MEMS microphone (e.g., ICS43434)

### 🧵 Connections:
- DIN → GPIO4
- BCLK → GPIO5
- WS (LRCLK) → GPIO6
- GND → GPIO20

# 🌐 Network Configuration
- Wi-Fi SSID: MyWifi (change to yours)
- Password: 12345678
- IPv6 Multicast Group: `ff0e::abcd`
- Port: 12345
- RTP packets have a fixed SSRC and incrementing sequence and timestamp values.
- by using `ff0e::abcd` I got rid of annoying interface ID, but you can easily use link-local multicast or even unicast if needed. Just ask ChatGPT to do it.

# 🚀 How It Works
- Initializes NVS and Wi-Fi in STA mode.
- Connects to the configured SSID and waits for an IPv6 address.
- Configures I2S input and starts a FreeRTOS task.
- Captures audio chunks (~5ms), converts to 16-bit samples.
- Prepends RTP headers and sends over IPv6 multicast.
- Receivers can decode and play the stream using standard RTP tools.

# 🎧 Receiving the Stream

You can use ffplay (from FFmpeg, tested on Windows and macOS) to receive and play the stream:

```sh
ffplay -protocol_whitelist file,udp,rtp -fflags nobuffer -i rtp://[ff0e::abcd]:12345

# You may need to quote an IP address:
ffplay -protocol_whitelist file,udp,rtp -fflags nobuffer -i "rtp://[ff0e::abcd]:12345"
```

### You can also route it using VB Cable (this way you make get a wireless wifi microphone, how cool is that)

# 🛠 Build and Flash

Requires ESP-IDF v5+. Ask AI for help if you get weird errors. The code works on my computer (and esp32c6) tho.

```
idf.py set-target esp32c6
idf.py menuconfig  # Optional: configure Wi-Fi etc.
idf.py build
idf.py flash monitor
```

# 🧾 License

### MIT License


Copyright 2025 arduinka55055 (SudoHub Team)

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
