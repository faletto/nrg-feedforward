/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.util;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.RobotConstants;

/** A class to manage a segment of the total LED string. */
public class LEDSegment {
  public static final int GAMMA_TABLE[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02,
    0x02, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x04, 0x04, 0x04, 0x04, 0x04, 0x05, 0x05, 0x05,
    0x05, 0x06, 0x06, 0x06, 0x06, 0x07, 0x07, 0x07, 0x07, 0x08, 0x08, 0x08, 0x09, 0x09, 0x09, 0x0a,
    0x0a, 0x0a, 0x0b, 0x0b, 0x0b, 0x0c, 0x0c, 0x0d, 0x0d, 0x0d, 0x0e, 0x0e, 0x0f, 0x0f, 0x10, 0x10,
    0x11, 0x11, 0x12, 0x12, 0x13, 0x13, 0x14, 0x14, 0x15, 0x15, 0x16, 0x16, 0x17, 0x18, 0x18, 0x19,
    0x19, 0x1a, 0x1b, 0x1b, 0x1c, 0x1d, 0x1d, 0x1e, 0x1f, 0x20, 0x20, 0x21, 0x22, 0x23, 0x23, 0x24,
    0x25, 0x26, 0x27, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31, 0x32, 0x32,
    0x33, 0x34, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x42, 0x43, 0x44,
    0x45, 0x46, 0x48, 0x49, 0x4a, 0x4b, 0x4d, 0x4e, 0x4f, 0x51, 0x52, 0x53, 0x55, 0x56, 0x57, 0x59,
    0x5a, 0x5c, 0x5d, 0x5f, 0x60, 0x62, 0x63, 0x65, 0x66, 0x68, 0x69, 0x6b, 0x6d, 0x6e, 0x70, 0x72,
    0x73, 0x75, 0x77, 0x78, 0x7a, 0x7c, 0x7e, 0x7f, 0x81, 0x83, 0x85, 0x87, 0x89, 0x8a, 0x8c, 0x8e,
    0x90, 0x92, 0x94, 0x96, 0x98, 0x9a, 0x9c, 0x9e, 0xa0, 0xa2, 0xa4, 0xa7, 0xa9, 0xab, 0xad, 0xaf,
    0xb1, 0xb4, 0xb6, 0xb8, 0xba, 0xbd, 0xbf, 0xc1, 0xc4, 0xc6, 0xc8, 0xcb, 0xcd, 0xd0, 0xd2, 0xd5,
    0xd7, 0xda, 0xdc, 0xdf, 0xe1, 0xe4, 0xe7, 0xe9, 0xec, 0xef, 0xf1, 0xf4, 0xf7, 0xf9, 0xfc, 0xff
  };

  private static final AddressableLEDBuffer ledBuffer =
      new AddressableLEDBuffer(RobotConstants.LED_COUNT);
  private static final AddressableLED leds = createAddressableLED(ledBuffer);

  private final int firstLED;
  private final int numberOfLEDs;

  /**
   * Creates and initializes an addressable LED.
   *
   * @param buffer The addressable LED buffer.
   */
  private static AddressableLED createAddressableLED(AddressableLEDBuffer buffer) {
    AddressableLED led = new AddressableLED(RobotConstants.PWMPort.LED);
    led.setLength(RobotConstants.LED_COUNT);
    led.setData(buffer);
    led.start();
    return led;
  }

  /**
   * Constructs an instance of this class.
   *
   * @param firstLED The index of the first LED in the segment.
   * @param numberOfLEDs The number of LEDs in the segment.
   */
  public LEDSegment(int firstLED, int numberOfLEDs) {
    this.firstLED = firstLED;
    this.numberOfLEDs = numberOfLEDs;
  }

  /**
   * Fills the LED segment with a specific color.
   *
   * @param color The color it fills the LEDs.
   */
  public void fill(Color8Bit color) {
    for (var i = 0; i < numberOfLEDs; i++) {
      setColor(color, i + firstLED);
    }
  }

  /**
   * Sets the color of the LED at the specified index.
   *
   * @param color The color to set.
   * @param index The index of the LED to set.
   */
  public void setColor(Color8Bit color, int index) {
    ledBuffer.setRGB(
        index, GAMMA_TABLE[color.red], GAMMA_TABLE[color.green], GAMMA_TABLE[color.blue]);
  }

  /** Sends the LED data to the LED string. */
  public void commitColor() {
    leds.setData(ledBuffer);
  }
}
