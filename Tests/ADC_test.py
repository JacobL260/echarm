import time
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

# from config
# ADS1115 I2C addresses
ADS_ADDRESSES = [0x48, 0x49]

# Channel mapping (ADS index, channel enum name)
ADC_CHANNEL_MAP = [
    (0, 0),
    (0, 1),
    (0, 2),
    (1, 0),
    (1, 1),
    (1, 2),
]

# Create I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Initialize ADS1115 boards dynamically from config
ads_list = [ADS.ADS1115(i2c, address=addr) for addr in ADS_ADDRESSES]

# Set gain for all boards
for ads in ads_list:
    ads.gain = 1  # ±4.096 V

while True:
    voltages = []

    # Read channels according to mapping in config
    for ads_index, channel_index in ADC_CHANNEL_MAP:
        ch = AnalogIn(ads_list[ads_index], channel_index)
        voltages.append(f"{ch.voltage:.3f} V")

    # Print results
    print(", ".join(voltages))
    print("-" * 40)
    time.sleep(1)