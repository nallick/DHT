//
//  DHT.swift
//
//  Read temperature and humidity from DHT11 and DHT22/AM2302 sensors on a Raspberry Pi.
//
//  Adapted from: https://github.com/adafruit/Adafruit_Python_DHT
//
//  Copyright © 2019 Purgatory Design. Licensed under the MIT License.
//

import Foundation
import SwiftyGPIO

public class DHT {

    public typealias HumdityCallback = (Int) -> Void
    public typealias TemperatureCallback = (Double) -> Void
    public typealias SampleCallback = (SampleResult) -> Void
    public typealias Sample = (humidity: Int, temperature: Int)   // tenths of percent relative humidity, tenths of degrees celsius
    public typealias SampleResult = Result<Sample, SampleError>

    public enum Device {
        case dht11, dht22
    }

    public enum SampleError: Error {
        case timeout, checksum
    }

    public let pin: GPIO
    public let device: Device

    public private(set) var isRunning = false
    public private(set) var humidity = Int.min          // percent relative humidity (since we have at best 2% or 5% accuracy there's no reason for more precision)
    public private(set) var temperature = Double.nan    // degrees celsius (accurate to 2.0°C or 0.5°C)

    private let timeoutLoopLimit: Int
    private var samples: [(Date, Sample)] = []
    private var activityQueue: DispatchQueue?
    private var updateTimer: Timer?
    private var readInterval: TimeInterval = 0.0
    private var updateInterval: TimeInterval = 0.0
    private var humidityCallback: HumdityCallback?
    private var temperatureCallback: TemperatureCallback?
    private var sampleCallback: SampleCallback?

    /// Initialize a DHT reader.
    ///
    /// - Parameter pin: The GPIO pin the device is attached to.
    /// - Parameter device: The type of DHT device to read.
    /// - Parameter timeoutLoopLimit: The number of times to loop before giving up. Faster machines may need larger numbers, but the default is over 130% of typical on RPi 4 or 200% on RPi 2.
    ///
    public init(pin: GPIO, device: Device, timeoutLoopLimit: Int = 300) {
        self.pin = pin
        self.device = device
        self.timeoutLoopLimit = timeoutLoopLimit
    }

    /// Read from the device.
    ///
    /// - Returns: The sample read from the device (in tenths of units), or a timeout or invalid checksum failure.
    ///
    public func sample() -> SampleResult {
        let pulseCount = 41
        var lowPulseCount = [Int](repeating: 0, count: pulseCount)
        var highPulseCount = [Int](repeating: 0, count: pulseCount)

        //********* start time sensitive section *********//

        // set pin high for ~500 milliseconds
        self.pin.direction = .OUT
        self.pin.value = 1
        usleep(500_000)

        // set pin low for ~20 milliseconds
        self.pin.value = 0
        usleep(20_000)

        // prepare to read the pin
        self.pin.direction = .IN
        usleep(1)

        // wait for DHT to pull pin low
        var waitForStartCount = 0
        while self.pin.value != 0 {
            waitForStartCount += 1
            guard waitForStartCount < timeoutLoopLimit else { return .failure(.timeout) }
        }
        var maxCount = waitForStartCount

        // record pulse widths for the expected result bits
        for index in 0 ..< pulseCount {

            // count how long pin is low and store in lowPulseCounts[index]
            while self.pin.value == 0 {
                lowPulseCount[index] += 1
                guard lowPulseCount[index] < timeoutLoopLimit else { return .failure(.timeout) }
                maxCount = max(maxCount, lowPulseCount[index])
            }

            // count how long pin is high and store in highPulseCounts[index]
            while self.pin.value != 0 {
                highPulseCount[index] += 1
                guard highPulseCount[index] < timeoutLoopLimit else {
                    if index == pulseCount - 1 { break }  // don't timeout on the last pulse (although the checksum may fail)
                    return .failure(.timeout)
                }
                maxCount = max(maxCount, highPulseCount[index])
            }
        }

        //********* end time sensitive section *********//

        // ignore the first reading because it's a constant 80 microsecond pulse
        let lowPulseWidth = lowPulseCount.dropFirst()
        let highPulseWidth = Array(highPulseCount.dropFirst())

        // compute the average low pulse width to use as a 50 microsecond reference threshold
        let averageLowPulseWidth = lowPulseWidth.reduce(0, +)/lowPulseWidth.count

        // interpret each high pulse as a 0 or 1 by comparing it to the 50us reference
        // if the count is less than 50us it must be a ~28us 0 pulse, and if it's higher it must be a ~70us 1 pulse
        var data = [Int](repeating: 0, count: 5)
        for index in 0 ..< highPulseWidth.count {
            let byteIndex = index/8
            data[byteIndex] <<= 1
            if highPulseWidth[index] >= averageLowPulseWidth {
                data[byteIndex] |= 1  // a long pulse is a one bit, otherwise leave it zero
            }
        }

        guard (data[4] & 0xFF) == ((data[0] + data[1] + data[2] + data[3]) & 0xFF) else { return .failure(.checksum) }

        if self.device == .dht11 { return .success((data[0]*10 + data[1], data[2]*10 + data[3])) }

        let humidity = (data[0] << 8) | data[1]
        var temperature = ((data[2] & 0x7F) << 8) | data[3]
        if (data[2] & 0x80) != 0 { temperature = -temperature }

        return .success((humidity, temperature))
    }

    /// Start sampling the device periodically.
    ///
    /// - Parameter readInterval: The time between reads (this should be at least 1.0 for a DHT11 and 2.0 for a DHT22).
    /// - Parameter updateInterval: The minimum time between updates (if nothing changes updates will be less frequent).
    /// - Parameter queueLabel: The DispatchQueue label.
    /// - Parameter humidity: The humidity change callback (this will be on the receiver's background activity thread).
    /// - Parameter temperature: The temperature change callback (this will be on the receiver's background activity thread).
    /// - Parameter sample: The sample callback for each read (this will be on the receiver's background activity thread).
    ///
    public func start(readInterval: TimeInterval, updateInterval: TimeInterval, queueLabel: String = "com.PurgatoryDesign.DHT", humidity: HumdityCallback? = nil, temperature: TemperatureCallback? = nil, sample: SampleCallback? = nil) {
        self.readInterval = readInterval
        self.updateInterval = updateInterval
        self.humidityCallback = humidity
        self.temperatureCallback = temperature
        self.sampleCallback = sample
        self.isRunning = true

        self.activityQueue = DispatchQueue(label: queueLabel, qos: .userInteractive)  // a high thread priority works best for timing loops
        self.activityQueue?.async { [weak self] in
            self?.performRead()
        }

        self.updateTimer = Timer.scheduledTimer(withTimeInterval: updateInterval, repeats: true) { [weak self] _ in
            self?.activityQueue?.async {
                self?.performUpdate()
            }
        }
    }

    /// Stop the periodic sampling of the device.
    ///
    public func stop() {
        self.activityQueue?.async { [weak self] in
            guard let self = self else { return }
            self.isRunning = false
            self.updateTimer?.invalidate()
            self.updateTimer = nil
            self.humidityCallback = nil
            self.temperatureCallback = nil
            self.activityQueue = nil
            self.samples.removeAll()
        }
    }

    /// Perform an individual device read.
    ///
    private func performRead() {
        if self.isRunning {
            let result = self.sample()
            if case .success(let sample) = result {
                self.samples.append((Date(), sample))
            }

            self.activityQueue?.asyncAfter(deadline: .now() + self.readInterval) { [weak self] in
                self?.performRead()
            }

            self.sampleCallback?(result)
        }
    }

    /// Perform a periodic update, making the device callbacks for changes.
    ///
    private func performUpdate() {
        let now = Date()
        self.samples = self.samples.filter { now.timeIntervalSince($0.0) <= self.updateInterval }
        guard !self.samples.isEmpty else { return }
        let sampleCount = Double(self.samples.count)

        let humiditySum = self.samples.reduce(0) { accumulator, value in accumulator + value.1.humidity }
        let averageHumidity = Int(round(Double(humiditySum)/(sampleCount*10.0)))
        if averageHumidity != self.humidity {
            self.humidity = averageHumidity
            self.humidityCallback?(averageHumidity)
        }

        let temperatureSum = self.samples.reduce(0) { accumulator, value in accumulator + value.1.temperature }
        let averageTemperature = round(Double(temperatureSum)/sampleCount)/10.0
        if averageTemperature != self.temperature {
            self.temperature = averageTemperature
            self.temperatureCallback?(averageTemperature)
        }
    }
}

//
// Adapted from: https://github.com/adafruit/Adafruit_Python_DHT
//
// https://github.com/adafruit/Adafruit_Python_DHT/blob/master/source/Raspberry_Pi/pi_dht_read.c
//
// Copyright (c) 2014 Adafruit Industries
// Author: Tony DiCola

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
