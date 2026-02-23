git clone https://github.com/JacobL260/echarm.git
cd echarm

sudo apt-get update
sudo apt-get install python3-rpi.gpio python3-pigpio

sudo apt-get update
sudo apt-get install swig python3-dev build-essential

sudo apt-get install liblgpio-dev



python3 -m venv echarmvenv

source echarmvenv/bin/activate

python -m pip install --upgrade pip



pip install -r requirements.txt

ADC_MODE=simulation python main.py
valid values: "hardware", "simulation"

deactivate