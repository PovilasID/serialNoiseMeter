#!/bin/ash

# --- SENSOR CONFIGURATION ---
SERIAL="/dev/ttyACM1" # <--- IMPORTANT: CHANGE THIS to your serial device (run: ls /dev/tty* to find)
BAUD_RATE="9600" # <--- IMPORTANT: VERIFY THIS FROM SENSOR DATASHEET

# Modbus request: slave 1, func 3, addr 0x0000, 1 register
REQ='\x01\x03\x00\x00\x00\x01\x84\x0A'

# --- MQTT CONFIGURATION ---
MQTT_BROKER="127.0.0.1" # <--- IMPORTANT: CHANGE THIS
MQTT_PORT="1883"                             # <--- IMPORTANT: CHANGE THIS if different (e.g., 8883 for TLS)
MQTT_USERNAME=""                             # <--- Optional: Your MQTT username
MQTT_PASSWORD=""                             # <--- Optional: Your MQTT password

# Home Assistant Discovery Details
HA_DISCOVERY_PREFIX="homeassistant"
DEVICE_NAME="DIY Noise Sensor"
DEVICE_ID="noise_sensor_DYI_01" # Unique ID for the device
SENSOR_NAME="Noise Level"
SENSOR_UNIQUE_ID="${DEVICE_ID}_noise_level"
HW_VERSION="KE-ZS-BZ-TTL-05 + USB-TTL"
STATE_TOPIC="${HA_DISCOVERY_PREFIX}/sensor/${DEVICE_ID}/noise_level/state"
DISCOVERY_TOPIC="${HA_DISCOVERY_PREFIX}/sensor/${DEVICE_ID}/noise_level/config"


# --- MQTT AUTHENTICATION FLAGS (for mosquitto_pub) ---
MQTT_AUTH_FLAGS=""
if [ -n "$MQTT_USERNAME" ]; then
    MQTT_AUTH_FLAGS="-u \"$MQTT_USERNAME\""
    if [ -n "$MQTT_PASSWORD" ]; then
        MQTT_AUTH_FLAGS="$MQTT_AUTH_FLAGS -P \"$MQTT_PASSWORD\""
    fi
fi


# --- FUNCTIONS ---

publish_mqtt() {
    local topic="$1"
    local payload="$2"
    # echo "Publishing to $topic: $payload" # This is informational, not error. Keep as STDOUT or remove if you want no info logging.
    # Using full path for mosquitto_pub for robustness in init.d environment
    /usr/bin/mosquitto_pub -h "$MQTT_BROKER" -p "$MQTT_PORT" $MQTT_AUTH_FLAGS -t "$topic" -m "$payload" -q 1 -r || {
        echo "Error: Could not publish to MQTT (topic: $topic, payload: $payload). Check broker and network." >&2 # <--- REDIRECTED TO STDERR
    }
}

send_ha_discovery_payload() {
    # Home Assistant MQTT Sensor Discovery Payload
    # See: https://www.home-assistant.io/integrations/sensor.mqtt/#mqtt-discovery
    # and https://www.home-assistant.io/docs/mqtt/discovery/

    local DISCOVERY_PAYLOAD=$(cat <<EOF
{
  "name": "${SENSOR_NAME}",
  "unique_id": "${SENSOR_UNIQUE_ID}",
  "state_topic": "${STATE_TOPIC}",
  "unit_of_measurement": "dB",
  "device_class": "sound_pressure",
  "state_class": "measurement",
  "value_template": "{{ value_json.value | float }}",
  "json_attributes_topic": "${STATE_TOPIC}",
  "device": {
    "identifiers": ["${DEVICE_ID}"],
    "name": "${DEVICE_NAME}",
    "model": "Modbus RTU Noise Sensor",
    "manufacturer": "Generic",
    "hw_version": "${HW_VERSION}"
  }
}
EOF
)
    # The publish_mqtt function handles its own errors
    publish_mqtt "$DISCOVERY_TOPIC" "$DISCOVERY_PAYLOAD"
}

# --- SCRIPT START ---

# Configure serial port
stty -F "$SERIAL" "$BAUD_RATE" raw -echo cs8 -cstopb -parenb || {
    echo "Error: Could not configure serial port $SERIAL. Exiting." >&2 # <--- REDIRECTED TO STDERR
    exit 1
}

echo "Polling noise sensor on $SERIAL at $BAUD_RATE baud..." # This is informational, keep as STDOUT or remove
echo "Press Ctrl+C to stop." # This is informational, keep as STDOUT or remove

# Send discovery payload once at startup
echo "Sending Home Assistant MQTT Discovery payload..." # This is informational, keep as STDOUT or remove
send_ha_discovery_payload

while true; do
    # Send read command
    echo -ne "$REQ" > "$SERIAL"

    # Read up to 7 bytes. On OpenWrt, 'dd' might block if no data arrives.
    RESPONSE=$(dd if="$SERIAL" bs=7 count=1 2>/dev/null | xxd -p -c 14)

    # Check if we got anything back
    if [ -z "$RESPONSE" ]; then
        echo "Warning: No response from sensor. Check connection/baud rate or sensor connection." >&2 # <--- REDIRECTED TO STDERR
    # Check if response is at least 7 bytes (14 hex characters)
    elif [ $(echo -n "$RESPONSE" | wc -c) -ge 14 ]; then
        # Extract the two data bytes (chars 7-10) using 'cut'
        HEX_VAL=$(echo "$RESPONSE" | cut -c7-10)

        # Convert hex value to decimal using portable 'printf'
        # The '0x' prefix is required for printf
        DEC=$(printf "%d" "0x$HEX_VAL")

        # Assuming the sensor value is scaled by 10 (e.g., 356 means 35.6 dB)
        INTEGER_PART=$((DEC / 10))
        DECIMAL_PART=$((DEC % 10))
        SOUND_LEVEL="${INTEGER_PART}.${DECIMAL_PART}"

        # This is a normal successful reading, typically not an error.
        # If you don't want this in any log, you can remove this line.
        echo "Sound Level: ${SOUND_LEVEL} dB" # <--- Keep as STDOUT (will be discarded by init.d script)

        # --- MQTT PUBLISHING ---
        # Publish the value as JSON for Home Assistant to parse easily
        JSON_PAYLOAD="{\"value\": ${SOUND_LEVEL}}"
        publish_mqtt "$STATE_TOPIC" "$JSON_PAYLOAD"
    else
        echo "Warning: Incomplete response received. Raw: $RESPONSE" >&2 # <--- REDIRECTED TO STDERR
    fi

    sleep 2 # Delay before the next poll
done