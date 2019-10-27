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

    public private(set) var humidity = Int.min          // percent relative humidity (since we have at best 2% or 5% accuracy there's no reason for more precision)
    public private(set) var temperature = Double.nan    // degrees celsius (accurate to 2.0°C or 0.5°C)

    private let timeoutLoopLimit: Int
    private var samples: [(Date, Sample)] = []

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

        do {
            //********* start time sensitive section *********//
            DHT.setMaximumThreadPriority()
            defer { DHT.setDefaultThreadPriority() }

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

            // record pulse widths for the expected result bits
            for index in 0 ..< pulseCount {

                // count how long pin is low and store in lowPulseCounts[index]
                while self.pin.value == 0 {
                    lowPulseCount[index] += 1
                    guard lowPulseCount[index] < timeoutLoopLimit else { return .failure(.timeout) }
                }

                // count how long pin is high and store in highPulseCounts[index]
                while self.pin.value != 0 {
                    highPulseCount[index] += 1
                    guard highPulseCount[index] < timeoutLoopLimit else { return .failure(.timeout) }
                }
            }
            //********* end time sensitive section *********//
        }

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

    /// Set the priority of the current thread to maximum with FIFO scheduling to get as close to real time computing behavior as possible.
    ///
    private static func setMaximumThreadPriority() {
        #if os(Linux)
        var scheduler = sched_param()
        scheduler.sched_priority = sched_get_priority_max(SCHED_FIFO)
        _ = sched_setscheduler(0, SCHED_FIFO, &scheduler)   // this tends to set errno to EPERM (insufficent privileges) but seems to help regardless
        #endif
    }

    /// Set the priority of the current thread to the default.
    ///
    private static func setDefaultThreadPriority() {
        #if os(Linux)
        var scheduler = sched_param()
        scheduler.sched_priority = 0
        _ = sched_setscheduler(0, SCHED_OTHER, &scheduler)
        #endif
    }
}

extension DHT {

    public typealias HumdityCallback = (DHT, Int) -> Void
    public typealias TemperatureCallback = (DHT, Double) -> Void
    public typealias SampleCallback = (DHT, SampleResult) -> Void

    public private(set) static var isRunning = false

    private static var devices: [DHT] = []
    private static var activityQueue: DispatchQueue?
    private static var updateTimer: Timer?
    private static var readInterval: TimeInterval = 0.0
    private static var updateInterval: TimeInterval = 0.0

    /// Start sampling a list of devices periodically.
    ///
    /// - Parameter devices: The devices to sample.
    /// - Parameter readInterval: The time between reads (this should be at least 1.0 for a DHT11 and 2.0 for a DHT22).
    /// - Parameter updateInterval: The minimum time between updates (if nothing changes updates will be less frequent).
    /// - Parameter queueLabel: The DispatchQueue label.
    /// - Parameter humidity: The humidity change callback (this will be on the receiver's background activity thread).
    /// - Parameter temperature: The temperature change callback (this will be on the receiver's background activity thread).
    /// - Parameter sample: The sample callback for each read (this will be on the receiver's background activity thread).
    ///
    public static func start(devices: [DHT], readInterval: TimeInterval, updateInterval: TimeInterval, queueLabel: String = "com.PurgatoryDesign.DHT", humidity: HumdityCallback? = nil, temperature: TemperatureCallback? = nil, sample: SampleCallback? = nil) {
        self.devices = devices
        self.readInterval = readInterval
        self.updateInterval = updateInterval
        self.isRunning = true

        self.activityQueue = DispatchQueue(label: queueLabel)
        self.activityQueue?.async {
            self.devices.forEach { $0.performRead(sample: sample) }
        }

        self.updateTimer = Timer.scheduledTimer(withTimeInterval: updateInterval, repeats: true) { _ in
            DHT.activityQueue?.async {
                DHT.devices.forEach { $0.performUpdate(humidity: humidity, temperature: temperature) }
            }
        }
    }

    /// Stop the periodic sampling of the devices.
    ///
    public static func stop() {
        self.activityQueue?.async {
            self.isRunning = false
            self.updateTimer?.invalidate()
            self.updateTimer = nil
            self.activityQueue = nil

            self.devices.forEach { $0.samples.removeAll() }
            self.devices.removeAll()
        }
    }

    /// Perform an individual device read.
    ///
    /// - Parameter sample: The sample callback for each read (this will be on the receiver's background activity thread).
    ///
    private func performRead(sample: SampleCallback?) {
        if DHT.isRunning {
            let result = self.sample()
            if case .success(let sample) = result {
                self.samples.append((Date(), sample))
            }

            DHT.activityQueue?.asyncAfter(deadline: .now() + DHT.readInterval) { [weak self] in
                self?.performRead(sample: sample)
            }

            sample?(self, result)
        }
    }

    /// Perform a periodic update, making the device callbacks for changes.
    ///
    /// - Parameter humidity: The humidity change callback (this will be on the receiver's background activity thread).
    /// - Parameter temperature: The temperature change callback (this will be on the receiver's background activity thread).
    ///
    private func performUpdate(humidity: HumdityCallback?, temperature: TemperatureCallback?) {
        let now = Date()
        self.samples = self.samples.filter { now.timeIntervalSince($0.0) <= DHT.updateInterval }
        guard !self.samples.isEmpty else { return }
        let sampleCount = Double(self.samples.count)

        let humiditySum = self.samples.reduce(0) { accumulator, value in accumulator + value.1.humidity }
        let averageHumidity = Int(round(Double(humiditySum)/(sampleCount*10.0)))
        if averageHumidity != self.humidity {
            self.humidity = averageHumidity
            humidity?(self, averageHumidity)
        }

        let temperatureSum = self.samples.reduce(0) { accumulator, value in accumulator + value.1.temperature }
        let averageTemperature = round(Double(temperatureSum)/sampleCount)/10.0
        if averageTemperature != self.temperature {
            self.temperature = averageTemperature
            temperature?(self, averageTemperature)
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
