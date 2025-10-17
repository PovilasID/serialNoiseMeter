# Serial Noise Meter

This an example of how to use a cheap ZTS-ZS-BZ-TTL-05 (probably same as KE-ZS-BZ-TTL-05). It is a cheap 'precalibrated' sensnor for messuring noise levels and sending to MQTT for HomeAssitant.


![sensnor](images/sensnor.avif)

### ⚠️ Very sensiive to power supply! 
Then using ESP32 to power it would increase the meassurment by ~20dB
Really not sure if it even worth the money.

###  ❗ AI slop
Code is mostly AI slop with a bit of refining from me. I am only sharing because I tested it and works and there dose not seam to be a lot of resources online.

I found two ways to run it:

## USB-to-TTL + compute + sensnor
Good for hardware testing

Use a USB-to-TTL that would be able to drive 5V devices. They are cheap 1-2 EUR and good chance you already own one. Only other thing you need is some compute device to plug it in.

I used an OpenWRT router so I ran a shell script to run it. I added a service (path /etc/init.d/noise-monitor) in OpenWRT to run start and stop running the script. 

## ESP32 + sensor
Much more flexible because you can run power to and put it anywhere like above a house window where the noise level is suppose to be measured acording to ISO standards for health.

I had a POE to USB-C and RJ45 and Lilygo T3 S3 for another project waiting it's turn so I used it but any 2 or 3 EUR ESP32 with 5V should work for this just need to change board definition in platformio file settings. 