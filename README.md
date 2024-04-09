# DHT

## Read temperature and humidity from DHT11 and DHT22 sensors on a Raspberry Pi in Swift.

Adapted from [C sample code](https://github.com/adafruit/Adafruit_Python_DHT) provided by the nice people at [AdaFruit](https://www.adafruit.com).

#### Example:

This reads from pin 4 every five seconds, supplying a moving average after every twelve successful reads (provided the input changes):
````
import DHT
import SwiftyGPIO

let allGPIOs = SwiftyGPIO.GPIOs(for: .RaspberryPi3)
let dhtGPIO = allGPIOs[.P4]!
let dht = DHT(pin: dhtGPIO, device: .dht22)

dht.start(readInterval: 5.0, updateSampleCount: 12,
          humidity: { print("humidity: \($0)%") },
          temperature: { print("temperature: \($0)°C") })
````

#### Use:

To add DHT to your project, declare a dependency in your Package.swift file,
````
.package(url: "https://github.com/nallick/DHT.git", from: "2.0.0"),
````
and add the dependency to your target:
````
.target(name: "MyProjectTarget", dependencies: ["DHT"]),
````
