import sounddevice as sd
import numpy as np
import socket

UDP_PORT = 12345
SAMPLE_RATE = 44100
CHUNK = 240  # 480 bytes per packet

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", UDP_PORT))

print("Receiving UDP audio...")

print(sd.query_devices())
print(sd.query_devices(43)['default_samplerate'])

"""
with sd.OutputStream(
    device=16,
    samplerate=44100,
    channels=1,
    dtype='int16',
    blocksize=CHUNK,  # match your CHUNK size
    latency='low'
) as stream:
"""
with sd.OutputStream(samplerate=SAMPLE_RATE, channels=1, dtype='int16') as stream:
    try:
        while True:
            data, addr = sock.recvfrom(512)
            if len(data) == CHUNK * 2:
                samples = np.frombuffer(data, dtype='<i2')  # force little-endian
                stream.write(samples)
    except KeyboardInterrupt:
        print("Stopped.")
