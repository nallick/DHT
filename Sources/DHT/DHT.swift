//
//  DHT.swift
//
//  Read temperature and humidity from DHT11 and DHT22/AM2302 sensors on a Raspberry Pi.
//
//  Adapted from: https://github.com/adafruit/Adafruit_Python_DHT
//
//  Copyright © 2019, 2023-2024 Purgatory Design. Licensed under the MIT License.
//

import Foundation
import SwiftyGPIO

open class DHT {

    public typealias Sample = (humidity: Int, temperature: Int)   // tenths of percent relative humidity, tenths of degrees celsius
    public typealias SampleResult = Result<Sample, SampleError>

    public enum Device: String, Codable {
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
    private var samples: [Sample] = []

    private enum PinState { case low, high }

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

    /// Ensure we can read from the device's GPIO pin without error.
    ///
    /// - Returns: Any error encountered when reading from the GPIO pin, or nil if the read is successful.
    ///
    public func preflight() -> Error? {
        do {
            try self.pin.setDirection(.IN)
            _ = try self.pin.getValue()
            return nil
        } catch {
            return error
        }
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
            guard let _ = self.waitForPinState(.low) else { return .failure(.timeout) }

            // record pulse widths for the expected result bits
            for index in 0 ..< pulseCount {

                // count how long pin is low and store in lowPulseCounts[index]
                guard let lowCount = self.waitForPinState(.high) else { return .failure(.timeout) }
                lowPulseCount[index] = lowCount

                // count how long pin is high and store in highPulseCounts[index]
                guard let highCount = self.waitForPinState(.low) else { return .failure(.timeout) }
                highPulseCount[index] = highCount
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

    /// Wait for our GPIO pin to acheive a specific state or timeout.
    ///
    /// - Parameter expectedState: The pin state to wait for.
    /// 
    /// - Returns: The number of times we performed a tight loop waiting for the state, or nil if we timed out with too many loop iterations.
    ///
    private func waitForPinState(_ expectedState: PinState) -> Int? {
        let continueWaiting: (Int) -> Bool = (expectedState == .low) ? { $0 != 0 } : { $0 == 0 }

        var loopCount = 0
        while continueWaiting(self.pin.value) {
            loopCount += 1
            guard loopCount < self.timeoutLoopLimit else { return nil }
        }

        return loopCount
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
    public typealias UpdatedCallback = (DHT, Date) -> Void
    public typealias SampleCallback = (DHT, SampleResult) -> Void

    public private(set) static var isRunning = false

    private static var devices: [DHT] = []
    private static var activityQueue: DispatchQueue?
    private static var readInterval: TimeInterval = 0.0
    private static var updateSampleCount: Int = 0

    /// Start sampling a list of devices periodically.
    ///
    /// - Parameter devices: The devices to sample.
    /// - Parameter readInterval: The time between reads (this should be at least 1.0 for a DHT11 and 2.0 for a DHT22).
    /// - Parameter updateSampleCount: The minimum number of good samples required for an update.
    /// - Parameter queueLabel: The DispatchQueue label.
    /// - Parameter humidity: The humidity change callback (this will be on the receiver's background activity thread).
    /// - Parameter temperature: The temperature change callback (this will be on the receiver's background activity thread).
    /// - Parameter updated: The humidity or temperature change callback (this will be on the receiver's background activity thread).
    /// - Parameter sample: The sample callback for each read (this will be on the receiver's background activity thread).
    ///
    public static func start(devices: [DHT], readInterval: TimeInterval, updateSampleCount: Int, queueLabel: String = "com.PurgatoryDesign.DHT", humidity: HumdityCallback? = nil, temperature: TemperatureCallback? = nil, updated: UpdatedCallback? = nil, sample: SampleCallback? = nil) {
        self.devices = devices
        self.readInterval = readInterval
        self.updateSampleCount = updateSampleCount
        self.isRunning = true

        self.activityQueue = DispatchQueue(label: queueLabel)
        self.activityQueue?.async {
            self.devices.forEach { $0.performRead(sample: sample, humidity: humidity, temperature: temperature, updated: updated) }
        }
    }

    /// Stop the periodic sampling of the devices.
    ///
    public static func stop() {
        self.activityQueue?.async {
            self.isRunning = false
            self.activityQueue = nil

            self.devices.forEach { $0.samples.removeAll() }
            self.devices.removeAll()
        }
    }

    /// Perform an individual device read.
    ///
    /// - Parameter sample: The sample callback for each read (this will be on the receiver's background activity thread).
    /// - Parameter humidity: The humidity change callback (this will be on the receiver's background activity thread).
    /// - Parameter temperature: The temperature change callback (this will be on the receiver's background activity thread).
    /// - Parameter updated: The humidity or temperature change callback (this will be on the receiver's background activity thread).
    ///
    private func performRead(sample: SampleCallback?, humidity: HumdityCallback?, temperature: TemperatureCallback?, updated: UpdatedCallback?) {
        if DHT.isRunning {
            let result = self.sample()
            if case .success(let sample) = result {
                self.samples.append(sample)
            }

            DHT.activityQueue?.asyncAfter(deadline: .now() + DHT.readInterval) { [weak self] in
                self?.performRead(sample: sample, humidity: humidity, temperature: temperature, updated: updated)
            }

            sample?(self, result)

            if self.samples.count >= Self.updateSampleCount {
                self.performUpdate(humidity: humidity, temperature: temperature, updated: updated)
            }
        }
    }

    /// Perform a periodic update, making the device callbacks for changes.
    ///
    /// - Parameter humidity: The humidity change callback (this will be on the receiver's background activity thread).
    /// - Parameter temperature: The temperature change callback (this will be on the receiver's background activity thread).
    /// - Parameter updated: The humidity or temperature change callback (this will be on the receiver's background activity thread).
    ///
    private func performUpdate(humidity: HumdityCallback?, temperature: TemperatureCallback?, updated: UpdatedCallback?) {
        guard !self.samples.isEmpty else { return }

        let now = Date()
        let sampleCount = Double(self.samples.count)
        let humiditySum = self.samples.reduce(0) { accumulator, value in accumulator + value.humidity }
        let temperatureSum = self.samples.reduce(0) { accumulator, value in accumulator + value.temperature }
        self.samples.removeAll()

        var wasUpdated = false
        let averageHumidity = Int(round(Double(humiditySum)/(sampleCount*10.0)))
        if averageHumidity != self.humidity {
            self.humidity = averageHumidity
            humidity?(self, averageHumidity)
            wasUpdated = true
        }

        let averageTemperature = round(Double(temperatureSum)/sampleCount)/10.0
        if averageTemperature != self.temperature {
            self.temperature = averageTemperature
            temperature?(self, averageTemperature)
            wasUpdated = true
        }

        if wasUpdated {
            updated?(self, now)
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
