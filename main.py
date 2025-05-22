import network
import time
import json
from machine import Pin, PWM, ADC
from umqtt_simple import MQTTClient

WIFI_SSID = "wifiNameHere"
WIFI_PASSWORD = "PasswordHere"

# MQTT Settings
MQTT_BROKER = "192.168.0.3"  # Change to your HA MQTT broker IP
MQTT_CLIENT_ID = "pico_moon_lamp"
MQTT_TOPIC = "home/moonlamp"

# Initialize PWM pins for RGB LED
red = machine.PWM(machine.Pin(4))
green = machine.PWM(machine.Pin(3))
blue = machine.PWM(machine.Pin(5))
led = machine.Pin("LED", machine.Pin.OUT)

# Set PWM frequency
red.freq(1000)
green.freq(1000)
blue.freq(1000)

red.duty_u16(0)
green.duty_u16(0)
blue.duty_u16(0)

# Potentiometer input (ADC0 on GP26)
pot = ADC(Pin(26))

# Track LED state
led_on = False  # Default ON
brightness = 255  # Default max brightness (0-255 scale)
current_color = {"r": 255, "g": 255, "b": 255}  # Default white

# WiFi Connection
def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(WIFI_SSID, WIFI_PASSWORD)
    
    print("Connecting to WiFi...", end="")
    while not wlan.isconnected():
        # Blink LED when not connected
        led.value(1)  # Turn LED on
        time.sleep(0.5)
        led.value(0)  # Turn LED off
        time.sleep(0.5)

    led.value(1)

def set_led(r, g, b, brightness):
    """ Adjust LED brightness and color based on received values. """
    scale = brightness / 255  # Scale 255-based input to 16-bit duty cycle
    red.duty_u16(int(r * scale * 257))  # Convert 8-bit (0-255) to 16-bit (0-65535)
    green.duty_u16(int(g * scale * 257))
    blue.duty_u16(int(b * scale * 257))
    
def mqtt_callback(topic, msg):
    global led_on, brightness, current_color
    print(f"Received: {msg}")

    try:
        msg_str = msg.decode('utf-8')
        
        # Load JSON data
        data = json.loads(msg_str)
        if "state" in data:
            if data["state"] == "OFF":
                led_on = False
                set_led(0, 0, 0, 0)  # Turn off LED
            elif data["state"] == "ON":
                led_on = True
                set_led(current_color["r"], current_color["g"], current_color["b"], brightness)

        if "brightness" in data:
            brightness = data["brightness"]  # Expecting 0-255
            if led_on:
                set_led(current_color["r"], current_color["g"], current_color["b"], brightness)

        if "color" in data:
            current_color = data["color"]  # Expecting {"r": 255, "g": 255, "b": 255}
            if led_on:
                set_led(current_color["r"], current_color["g"], current_color["b"], brightness)
    except Exception as e:
        print("Error processing MQTT message:", e)

# Connect WiFi & MQTT
connect_wifi()
mqtt_client = MQTTClient(MQTT_CLIENT_ID, MQTT_BROKER)
mqtt_client.set_callback(mqtt_callback)
mqtt_client.connect()
mqtt_client.subscribe(MQTT_TOPIC)

print("Connected to MQTT. Listening for messages...")


# Main Loop
while True:
    try:
        # Check for MQTT messages
        mqtt_client.check_msg()
    except Exception as e:
        print("Error:", e)
        time.sleep(5)  # Retry if error

