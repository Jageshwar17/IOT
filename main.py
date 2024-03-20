import os
import sys
import time
import board
import adafruit_dht
import paho.mqtt.client as mqtt
import json

import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(6, GPIO.OUT)
GPIO.output(6, False)

def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    client.subscribe("Input")

def on_message(client, userdata, mssg):
    print(f"{mssg.topic} {mssg.payload}")
    if mssg.payload == b'LEDON':
        print("Turn on led")
        GPIO.output(6, True)
    if mssg.payload == b'LEDOFF':
        print("Turn off led")
        GPIO.output(6, False)

dhtDevice = adafruit_dht.DHT11(board.D19, use_pulseio=False)
sensor_data = {'temperature': 0, 'humidity': 0}

MQTTServer = 'broker.hivemq.com'
TCPPort =  1883
WebSocketPort = 8000

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
client.connect(MQTTServer, TCPPort, WebSocketPort)
client.on_connect = on_connect
client.on_message = on_message
client.loop_start()

if __name__ == '__main__':
    try:
        while True:
            temperature = dhtDevice.temperature
            humidity = dhtDevice.humidity
            print("Temp: {:.1f} C          Humidity: {}%".format(temperature, humidity))
            time.sleep(2.0)
            sensor_data['temperature'] = temperature
            sensor_data['humidity'] = humidity
            
            client.publish('DHT11', json.dumps(sensor_data), 0)
            
            DHT11_DATA = str(temperature) + ',' + str(humidity)
            client.publish('DHT11_DATA', DHT11_DATA, 0)
            time.sleep(10)
            
    except RuntimeError as error:
        print(error.args[0])
        time.sleep(2.0)
        
    except KeyboardInterrupt:
        client.loop_stop()
        client.disconnect()
        print('Exiting Program')
        sys.exit(0)