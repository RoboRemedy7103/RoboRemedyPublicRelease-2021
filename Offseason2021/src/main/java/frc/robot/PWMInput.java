package frc.robot;

import edu.wpi.first.wpilibj.*;

//class to calculte the PMW signal value for a device such as an encoder

public class PWMInput {

  private double minPulseWidthOn, minPulseWidthOff;
  private int resolution;
  private DigitalInput digitalIn;
  private Counter counterHi;
  private Counter counterFull;

  public PWMInput(int inputNumber, double minPulseWidthOn, double minPulseWidthOff, int resolution) {
    this.minPulseWidthOn = minPulseWidthOn;
    this.minPulseWidthOff = minPulseWidthOff;
    this.resolution = resolution;
    digitalIn = new DigitalInput(inputNumber);
    counterHi = new Counter();
    counterHi.setUpSource(digitalIn);
    counterHi.setSemiPeriodMode(true);
    counterFull = new Counter();
    counterFull.setUpSource(digitalIn);
    counterFull.setUpSourceEdge(true, false);
  }

  public int getLastPulse() {
    double hiPulse = counterHi.getPeriod();
    double fullPulse = counterFull.getPeriod();
    hiPulse -= minPulseWidthOn;
    fullPulse -= minPulseWidthOff;
    return (int) Math.round((hiPulse / fullPulse) * resolution);
  }

}