/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Color;
import frc.robot.ColorSystem;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDSubsystem extends SubsystemBase {
  public static final int NUMBER_OF_LEDS = 40;
  //public static final int NUMBER_OF_LEDS = 12;
  public static final int LED_PORT = 9;
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;

  private int m_chaseLightIndex = 0;
  private boolean m_isDecreasing = false;
  private int m_lastValue = 0;

  /**
   * Creates a new LEDSubsystem.
   */
  public LEDSubsystem() {
    m_led = new AddressableLED(LED_PORT);
    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(NUMBER_OF_LEDS);
    m_led.setLength(m_ledBuffer.getLength());
    // Set the data
     m_led.setData(m_ledBuffer);
     m_led.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void switchOnLEDs(String colorName, String pattern) {
    if (pattern.equalsIgnoreCase(ColorSystem.COLOR_PATTERN_SOLID)) {
      Color color = ColorSystem.getColor(colorName);
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setHSV(i, color.getHue(), color.getSaturation(), color.getValue());
      }
    }

    if (pattern.equalsIgnoreCase(ColorSystem.COLOR_PATTERN_CHASE)) {
      Color color = ColorSystem.getColor(colorName);
      lightChase(color.getHue(), color.getSaturation(), color.getValue());
    }

    if (pattern.equalsIgnoreCase(ColorSystem.COLOR_PATTERN_PULSE)) {
      colorPulse(ColorSystem.getColor(colorName));
    }
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void solidColorPurple() {
    Color color = ColorSystem.getColor(ColorSystem.COLOR_PURPLE);
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setHSV(i, color.getHue(), color.getSaturation(), color.getValue());
    }
    m_led.setData(m_ledBuffer);
    m_led.start();
  }
  public void solidColorRed() {
    Color color = ColorSystem.getColor(ColorSystem.COLOR_RED);
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setHSV(i, color.getHue(), color.getSaturation(), color.getValue());
    }
    m_led.setData(m_ledBuffer);
    m_led.start();
  }
  public void solidColorYellow() {
    Color color = ColorSystem.getColor(ColorSystem.COLOR_YELLOW);
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setHSV(i, color.getHue(), color.getSaturation(), color.getValue());
    }
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void solidColorBlue() {
    Color color = ColorSystem.getColor(ColorSystem.COLOR_BLUE);
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setHSV(i, color.getHue(), color.getSaturation(), color.getValue());
    }
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void solidColorGreen() {
    Color color = ColorSystem.getColor(ColorSystem.COLOR_GREEN);
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setHSV(i, color.getHue(), color.getSaturation(), color.getValue());
    }
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void lightChaseRed() {
    Color color = ColorSystem.getColor(ColorSystem.COLOR_RED);
    lightChase(color.getHue(), color.getSaturation(), color.getValue());
  }

  public void lightChaseYellow() {
    Color color = ColorSystem.getColor(ColorSystem.COLOR_YELLOW);
    lightChase(color.getHue(), color.getSaturation(), color.getValue());
  }

  public void lightChaseGreen() {
    Color color = ColorSystem.getColor(ColorSystem.COLOR_GREEN);
    lightChase(color.getHue(), color.getSaturation(), color.getValue());
  }

  public void lightChaseBlue() {
    Color color = ColorSystem.getColor(ColorSystem.COLOR_BLUE);
    lightChase(color.getHue(), color.getSaturation(), color.getValue());
  }

  public void lightChaseOrange() {
    Color color = ColorSystem.getColor(ColorSystem.COLOR_ORANGE);
    lightChase(color.getHue(), color.getSaturation(), color.getValue());
  }

  public void lightChasePurple() {
    Color color = ColorSystem.getColor(ColorSystem.COLOR_PURPLE);
    lightChase(color.getHue(), color.getSaturation(), color.getValue());
  }

  private void lightChase(int h, int s, int v) {

    m_chaseLightIndex = 0;
    m_isDecreasing = false;
    // Turn th current light off
    // m_ledBuffer.setHSV(m_chaseLightIndex, h, 255, 0);
    // Now increase of decrase the index for the next light to turn on
    if (!m_isDecreasing && (m_chaseLightIndex < m_ledBuffer.getLength() - 1)) {
      m_chaseLightIndex++;
    } else if (m_isDecreasing && m_chaseLightIndex > 0) {
      m_chaseLightIndex--;
    }
    m_ledBuffer.setHSV(m_chaseLightIndex, h, s, v);
    if (m_chaseLightIndex == (m_ledBuffer.getLength() - 1)) {
      m_isDecreasing = true;
    }
    if (m_chaseLightIndex == 0) {
      m_isDecreasing = false;
    }

    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void colorPulsePurple() {
    colorPulse(ColorSystem.getColor(ColorSystem.COLOR_PURPLE));
  }
  
  public void colorPulseRed() {
    colorPulse(ColorSystem.getColor(ColorSystem.COLOR_RED));
  }

  public void colorPulseGreen() {
    colorPulse(ColorSystem.getColor(ColorSystem.COLOR_GREEN));
  }

  public void colorPulseBlue() {
    colorPulse(ColorSystem.getColor(ColorSystem.COLOR_BLUE));
  }

  public void colorPulsePink() {
    colorPulse(ColorSystem.getColor(ColorSystem.COLOR_PINK));
  }

  public void colorPulseYellow() {
    colorPulse(ColorSystem.getColor(ColorSystem.COLOR_YELLOW));
  }

  public void colorPulseOrange() {
    colorPulse(ColorSystem.getColor(ColorSystem.COLOR_ORANGE));
  }
  

  public void drivingTargetNotInView() {
    solidColorYellow();
  }

  public void drvingTargetInViewNotInRange() {
    solidColorRed();
  }

  public void drivingTargetInViewAndInRange() {
    solidColorGreen();
  }

  public void drivingConveyorNotFull() {
    lightChaseRed();
  }

  public void drivingConveryorFull() {
    lightChaseGreen();
  }

  public void climbing() {
    lightChaseBlue();
  }

  private void colorPulse(Color color) {
    if (m_isDecreasing) {
      if (m_lastValue == 0) {
        m_isDecreasing = false;
      } else {
        m_lastValue -= 5;
      }
    } else {
      if (m_lastValue == 255) {
        m_isDecreasing = true;
      } else {
        m_lastValue += 5;
      }
    }

    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setHSV(i, color.getHue(), 255, m_lastValue);
    }
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void party() {
     /* colorPulsePurple();
      lightChaseRed();
      colorPulseBlue();
      lightChaseRed();
      colorPulseGreen();
      lightChaseBlue();
      colorPulseOrange();
      lightChaseGreen();
      colorPulsePink();
      lightChasePurple();
      colorPulseRed();
      lightChaseOrange();
      colorPulseYellow(); */
      
      switchOnLEDs(ColorSystem.COLOR_PURPLE, ColorSystem.COLOR_PATTERN_CHASE);
      colorPulse(ColorSystem.getColor(ColorSystem.COLOR_PURPLE));
      
    }
    
  
}
