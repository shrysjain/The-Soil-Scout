"""
Event Handler
August 2023
Shreyas Jain, Minati Divakar
"""

# Install dependencies
import code
from xml.etree.ElementTree import tostring
import serial
import time

# Open serial monitor
ser = serial.Serial('COM5', 115200)
time.sleep(2)

# Create array
values = []

# Test
while True:
  values = ser.readline().decode().split("|")

  try:
    values[4] = values[4][0:5]
  except:
    values = values;

  phVoltage = values[0]
  moisturePercentage = values[1]
  ambientHumidity = values[2]
  ambientTemperature = values[3]
  headIndex = values[4]

  # print(values)