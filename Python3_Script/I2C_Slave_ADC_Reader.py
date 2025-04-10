from smbus2 import SMBus
import time

DEFAULT_ADDRESS = 0x10
BUFFER_LEN = 2001  # 1000 Samples × 2 Kanäle × 8 Bit + 1 CRC

def calculate_crc8(data):
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x07
            else:
                crc <<= 1
            crc &= 0xFF
    return crc

def format_samples(label, samples):
    print(f"\n📊 {label} – {len(samples)} Werte:")
    for i in range(0, len(samples), 50):
        print(' '.join(f"{b:02X}" for b in samples[i:i+50]))

def read_buffer(i2c_addr):
    with SMBus(1) as bus:
        print(f"▶️ Sende Sampling-Kommando (0x03)...")
        bus.write_byte(i2c_addr, 0x03)
        print(f"⏳ Warte 1.2s auf Daten...")
        time.sleep(1.2)

        print(f"📡 Sende Anfrage-Kommando (0x02)...")
        bus.write_byte(i2c_addr, 0x02)
        time.sleep(0.01)

        print(f"📥 Lese {BUFFER_LEN} Bytes...")
        data = []
        for i in range(BUFFER_LEN):
            try:
                byte = bus.read_byte(i2c_addr)
                data.append(byte)
            except Exception as e:
                print(f"❌ Fehler bei Byte {i}: {e}")
                break

        print(f"\n📦 Empfangen: {len(data)} Bytes.")

        if len(data) == BUFFER_LEN:
            ch1 = data[:-1:2]
            ch2 = data[1:-1:2]
            crc = data[-1]
            calc_crc = calculate_crc8(data[:-1])

            format_samples("CH1 (Kanal 1)", ch1)
            format_samples("CH2 (Kanal 2)", ch2)

            if crc == calc_crc:
                print(f"\n✅ CRC stimmt! 🧮 CRC = 0x{crc:02X}")
            else:
                print(f"\n❌ CRC-Fehler! Empfangen: 0x{crc:02X}, Erwartet: 0x{calc_crc:02X}")
        else:
            print("🚫 Nicht alle Daten empfangen!")

if __name__ == "__main__":
    print("🌟 STM32 I2C ADC Reader")
    try:
        addr_input = input(f"🔧 I2C-Adresse eingeben (hex, Default = 0x{DEFAULT_ADDRESS:02X}): ").strip()
        address = int(addr_input, 16) if addr_input else DEFAULT_ADDRESS
    except ValueError:
        print("❌ Ungültige Eingabe – nutze Default-Adresse.")
        address = DEFAULT_ADDRESS

    read_buffer(address)
