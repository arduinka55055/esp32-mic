import sounddevice as sd
import numpy as np
import socket
import struct


MCAST_GRP = 'ff12::abcd'
MCAST_PORT = 12345
SAMPLE_RATE = 44100
CHUNK = 240

sock = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(('', MCAST_PORT))

# Join multicast group on all interfaces (interface index 0)
group_bin = socket.inet_pton(socket.AF_INET6, MCAST_GRP)
mreq = group_bin + struct.pack('@I', 18)  # 0 means all interfaces
sock.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_JOIN_GROUP, mreq)

print(f"Listening on IPv6 multicast {MCAST_GRP}:{MCAST_PORT}")

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
