import serial
import numpy as np
import pyaudio
import threading
from scipy.signal import resample
import soundfile as sf

SERIAL_PORT = "/dev/ttyUSB0"     # check with: ls /dev/ttyUSB*
BAUD_RATE = 115200
SAMPLE_RATE = 44100
BUFFER_SIZE = 2048               
NUM_ZONES = 8
OUTPUT_DEVICE_INDEX = 1          # set to your DAC's index

SAMPLE_DIR = "/home/samfunston/bowl_samples"

PAD_C2 = "niue_padc2.wav"
PAD_C3 = "niue_padc3.wav"
PAD_C4 = "niue_padc4.wav"
PAD_C5 = "niue_padc5.wav"

# 8-note C major pentatonic C3, D3, E3, G3, A3, C4, E4, G4, built by
# pitch-shifting the nearest real octave sample by a small interval.
ZONE_NOTES = [
    (PAD_C3,  0),   # note 0  C3
    (PAD_C3,  2),   # note 1  D3
    (PAD_C3,  4),   # note 2  E3
    (PAD_C3,  7),   # note 3  G3
    (PAD_C4, -3),   # note 4  A3
    (PAD_C4,  0),   # note 5  C4
    (PAD_C4,  4),   # note 6  E4
    (PAD_C4,  7),   # note 7  G4
]

# Which note each SENSOR plays
NOTE_INDEX = [6, 5, 4, 3, 2, 1, 0, 7]

DIST_MIN  = 50         # mm, full volume
DIST_MAX  = 1500       # mm, silence threshold ~5ft matches BRIGHT_MAX
CURVE_POW = 2.0        # matches the light's pow(closeness, 2.0)

VOL_FULL    = 1.0
MASTER_GAIN = 0.5      # overall level tune by ear

ATTACK  = 0.06         # fade-in matched to light's bloom
RELEASE = 0.04         # fade-out slow trail, matched to light

XFADE = int(0.25 * SAMPLE_RATE)  # crossfade length ~250ms
LOOP_START_SEC = 1.5             # start of loop window after the attack
LOOP_LEN_SEC   = 4.0             # length of the stable loop window


# Load & build samples
def load_mono(path):
    data, sr = sf.read(path, dtype="float32")
    if data.ndim == 2:
        data = data.mean(axis=1)
    return data.astype(np.float32)

def shift_semitones(sample, semitones):
    if semitones == 0:
        return sample.copy()
    ratio = 2 ** (semitones / 12.0)
    new_len = int(len(sample) / ratio)
    return resample(sample, new_len).astype(np.float32)

def make_seamless(sample):
    n = len(sample)
    start = int(LOOP_START_SEC * SAMPLE_RATE)
    length = int(LOOP_LEN_SEC * SAMPLE_RATE)
    if start + length + XFADE > n:
        length = n - start - XFADE
    if length <= XFADE * 2:
        return sample.astype(np.float32)

    body = sample[start:start + length + XFADE].astype(np.float32).copy()

    # flatten the decay across the window so start and end match in loudness
    win = int(0.05 * SAMPLE_RATE)
    env = []
    for s in range(0, len(body), win):
        seg = body[s:s + win]
        env.append(np.sqrt(np.mean(seg ** 2)) + 1e-6)
    env = np.array(env, dtype=np.float32)
    target = np.median(env)
    gain_frames = np.clip(target / env, 0.5, 2.0)
    gain = np.interp(np.arange(len(body)),
                     np.arange(len(gain_frames)) * win,
                     gain_frames).astype(np.float32)
    body = body * gain

    # crossfade the seam
    L = len(body) - XFADE
    loop = body[:L].copy()
    head = body[:XFADE]
    tail = body[L:L + XFADE]
    fade_in = np.linspace(0.0, 1.0, XFADE, dtype=np.float32)
    fade_out = 1.0 - fade_in
    loop[:XFADE] = tail * fade_out + head * fade_in
    return loop.astype(np.float32)

print("Loading pad samples.")
cache = {}
def get_anchor(fname):
    if fname not in cache:
        cache[fname] = load_mono(f"{SAMPLE_DIR}/{fname}")
    return cache[fname]

print("Building 8 seamless pentatonic notes.")
SAMPLES = []
for fname, semis in ZONE_NOTES:
    SAMPLES.append(make_seamless(shift_semitones(get_anchor(fname), semis)))
print(f"Built {len(SAMPLES)} notes.")

# State
brightness = [0.0] * NUM_ZONES
smooth_vol = [0.0] * NUM_ZONES
positions = [0] * NUM_ZONES

# Serial
def serial_listener():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Listening on {SERIAL_PORT}.")
    while True:
        try:
            line = ser.readline().decode().strip()
            if not line:
                continue
            values = [int(x) for x in line.split(",")]
            if len(values) >= NUM_ZONES:
                for i in range(NUM_ZONES):
                    d = values[i]
                    if d <= 0 or d >= DIST_MAX:
                        brightness[i] = 0.0
                    else:
                        t = (d - DIST_MIN) / (DIST_MAX - DIST_MIN)
                        t = max(0.0, min(1.0, t))
                        closeness = 1.0 - t
                        brightness[i] = closeness ** CURVE_POW
        except:
            pass

# Helpers
def read_loop(sample, pos, n):
    L = len(sample)
    out = np.empty(n, dtype=np.float32)
    filled = 0
    p = pos
    while filled < n:
        navail = L - p
        ncopy = min(n - filled, navail)
        out[filled:filled + ncopy] = sample[p:p + ncopy]
        filled += ncopy
        p = (p + ncopy) % L
    return out, p

# Audio
def audio_callback(in_data, frame_count, time_info, status):
    output = np.zeros(frame_count, dtype=np.float32)

    for i in range(NUM_ZONES):
        note_idx = NOTE_INDEX[i]
        sample = SAMPLES[note_idx]

        target = brightness[i] * VOL_FULL
        if target > smooth_vol[i]:
            smooth_vol[i] += (target - smooth_vol[i]) * ATTACK
        else:
            smooth_vol[i] += (target - smooth_vol[i]) * RELEASE
        vol = smooth_vol[i]

        if vol < 0.001:
            _, positions[i] = read_loop(sample, positions[i], frame_count)
            continue

        chunk, positions[i] = read_loop(sample, positions[i], frame_count)
        output += chunk * vol

    output *= MASTER_GAIN
    output = np.tanh(output)

    return (output.astype(np.float32).tobytes(), pyaudio.paContinue)

# Run
listener_thread = threading.Thread(target=serial_listener, daemon=True)
listener_thread.start()

p = pyaudio.PyAudio()
print("Available output devices:")
for i in range(p.get_device_count()):
    dev = p.get_device_info_by_index(i)
    if dev['maxOutputChannels'] > 0:
        print(f"  [{i}] {dev['name']}")

stream = p.open(
    format=pyaudio.paFloat32,
    channels=1,
    rate=SAMPLE_RATE,
    output=True,
    output_device_index=OUTPUT_DEVICE_INDEX,
    frames_per_buffer=BUFFER_SIZE,
    stream_callback=audio_callback,
)

print("Audio stream started")
print("Press Ctrl+C to stop")
stream.start_stream()

try:
    while stream.is_active():
        pass
except KeyboardInterrupt:
    pass

stream.stop_stream()
stream.close()
p.terminate()
print("Stopped.")
