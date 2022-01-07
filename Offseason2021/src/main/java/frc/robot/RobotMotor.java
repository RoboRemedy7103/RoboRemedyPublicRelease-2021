package frc.robot;

import edu.wpi.first.wpilibj.*;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.revrobotics.*;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.*;

/**
 * Class to be used for any type of motor
 */
public class RobotMotor {
    public enum RobotMotorType {
        SparkMax, Falcon, TalonSRX, Victor
    }

    public enum RobotEncoderType {
        None, Internal, Cancoder, MagEncoder, ThroughBore
    }

    private RoboLog rLog;
    private RobotMotorType motorType;
    private CANSparkMax neoMotor;
    private RelativeEncoder neoEncoder;
    private SparkMaxPIDController neoPID;
    private TalonFX falconMotor;
    private TalonSRX talonMotor;
    private VictorSPX victorMotor;
    private BaseMotorController baseCtreMotor;
    Timer timer = new Timer();

    private double[] percentValue;
    private double[] velocityValue;
    private double unitsPerEncoderTick = 1.0;
    private double encoderTicksPerUnit = 1.0;
    private double lastAssignedPercentage = 0;
    private double lastAssignedPosition = 0;
    private double lastAssignedVelocity = 0;
    private double lastAssignedFeedForward = 0;
    private double lastAssignedTime = 0;
    private boolean isCoast;

    RobotMotor(RobotMotorType motorType, int motorID, boolean isInverted, boolean isCoast, RoboLog rLog,
            double unitsPerRevolution, double maxCurrent, RobotEncoderType encoderType, int absoluteEncoderID) {
        this.rLog = rLog;
        this.motorType = motorType;
        this.isCoast = isCoast;
        timer.start();
        if (motorType == RobotMotorType.SparkMax) {
            neoMotor = new CANSparkMax(motorID, MotorType.kBrushless);
            neoMotor.restoreFactoryDefaults();
            neoEncoder = neoMotor.getEncoder();
            neoPID = neoMotor.getPIDController();
            neoMotor.setInverted(isInverted);
            neoMotor.setIdleMode(isCoast ? IdleMode.kCoast : IdleMode.kBrake);
            neoEncoder.setVelocityConversionFactor(unitsPerRevolution / 60.0);
            neoEncoder.setPositionConversionFactor(unitsPerRevolution);
            setOutputCurrentLimit(maxCurrent);
            setPID(0, 0, 0, 0, false);
        } else if (motorType == RobotMotorType.Falcon || motorType == RobotMotorType.TalonSRX
                || motorType == RobotMotorType.Victor) {
            if (motorType == RobotMotorType.Falcon) {
                falconMotor = new TalonFX(motorID);
                baseCtreMotor = falconMotor;
            } else if (motorType == RobotMotorType.TalonSRX) {
                talonMotor = new TalonSRX(motorID);
                baseCtreMotor = talonMotor;
            } else if (motorType == RobotMotorType.Victor) {
                victorMotor = new VictorSPX(motorID);
                baseCtreMotor = victorMotor;
            }
            baseCtreMotor.configFactoryDefault();
            baseCtreMotor.setInverted(isInverted);
            baseCtreMotor.setNeutralMode(isCoast ? NeutralMode.Coast : NeutralMode.Brake);
            baseCtreMotor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_100Ms);
            baseCtreMotor.configVelocityMeasurementWindow(1);
            baseCtreMotor.configNeutralDeadband(0.01);
            baseCtreMotor.configVoltageCompSaturation(12.0);
            baseCtreMotor.enableVoltageCompensation(true);
            setInputCurrentLimit(maxCurrent);
            if (encoderType == RobotEncoderType.None) {

            } else if (encoderType == RobotEncoderType.Internal) {
                baseCtreMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
                unitsPerEncoderTick = unitsPerRevolution / 2048.0;
            } else if (encoderType == RobotEncoderType.Cancoder) {
                baseCtreMotor.configRemoteFeedbackFilter(absoluteEncoderID, RemoteSensorSource.CANCoder, 0);
                baseCtreMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
                unitsPerEncoderTick = unitsPerRevolution / 4096;
            } else if (encoderType == RobotEncoderType.MagEncoder) {
                baseCtreMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
                baseCtreMotor.setSensorPhase(true);
                unitsPerEncoderTick = unitsPerRevolution / 4096;
            } else if (encoderType == RobotEncoderType.ThroughBore) {
                baseCtreMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
                unitsPerEncoderTick = unitsPerRevolution / 8192;
            }
            encoderTicksPerUnit = 1.0 / unitsPerEncoderTick;
        } else {
            this.rLog.print("RobotMotor created with unknown RobotMotorType");
        }
    }

    public void setInputCurrentLimit(double amps) {
        if (motorType == RobotMotorType.Falcon) {
            SupplyCurrentLimitConfiguration falconCurrentLimitConfig = new SupplyCurrentLimitConfiguration(true, amps,
                    amps, 0.4);
            falconMotor.configSupplyCurrentLimit(falconCurrentLimitConfig);
        } else if (motorType == RobotMotorType.TalonSRX) {
            SupplyCurrentLimitConfiguration talonCurrentLimitConfig = new SupplyCurrentLimitConfiguration(true, amps,
                    amps, 0.4);
            talonMotor.configSupplyCurrentLimit(talonCurrentLimitConfig);
        }
    }

    public void setOutputCurrentLimit(double amps) {
        if (motorType == RobotMotorType.SparkMax) {
            neoMotor.setSmartCurrentLimit((int) amps);
        } else if (motorType == RobotMotorType.Falcon) {

        }
    }

    public double getEncoderPosition() {
        if (motorType == RobotMotorType.SparkMax) {
            return neoEncoder.getPosition();
        } else if (motorType == RobotMotorType.Falcon || motorType == RobotMotorType.TalonSRX
                || motorType == RobotMotorType.Victor) {
            return baseCtreMotor.getSelectedSensorPosition() * unitsPerEncoderTick;
        } else {
            return 0;
        }
    }

    public double getEncoderVelocity() {
        if (motorType == RobotMotorType.SparkMax) {
            return neoEncoder.getVelocity();
        } else if (motorType == RobotMotorType.Falcon || motorType == RobotMotorType.TalonSRX
                || motorType == RobotMotorType.Victor) {
            return baseCtreMotor.getSelectedSensorVelocity() * unitsPerEncoderTick * 10;
        } else {
            return 0;
        }
    }

    // Assign the value to be used as the current encoder position
    // This does not move the motor. Use setPosition to move the motor.
    public void setEncoderValue(double position) {
        if (motorType == RobotMotorType.SparkMax) {
            neoEncoder.setPosition(position);
        } else if (motorType == RobotMotorType.Falcon || motorType == RobotMotorType.TalonSRX
                || motorType == RobotMotorType.Victor) {
            baseCtreMotor.setSelectedSensorPosition((int) (position * encoderTicksPerUnit));
        }
    }

    // Assign the values to be used for feed forward calculations
    public void setPercentVelocityMapping(double[] percentValue, double[] velocityValue) {
        this.percentValue = percentValue;
        this.velocityValue = velocityValue;
    }

    // Assign the PID values to be used for setPosition or setVelicty calls
    public void setPID(double p, double i, double d, double iZone, boolean isVelocity) {
        if (motorType == RobotMotorType.SparkMax) {
            neoPID.setFF(0);
            neoPID.setP(p);
            neoPID.setI(i);
            neoPID.setD(d);
            neoPID.setIZone(iZone);
        } else if (motorType == RobotMotorType.Falcon || motorType == RobotMotorType.TalonSRX
                || motorType == RobotMotorType.Victor) {
            double scalingFactor = (isVelocity ? encoderTicksPerUnit * 0.1 : unitsPerEncoderTick);
            baseCtreMotor.config_kF(0, 0);
            baseCtreMotor.config_kP(0, p * scalingFactor);
            baseCtreMotor.config_kI(0, i * scalingFactor);
            baseCtreMotor.config_kD(0, d * scalingFactor);
            baseCtreMotor.config_IntegralZone(0, (int) (iZone * scalingFactor));
        }
    }

    public void setDriveOpenLoopRamp(double rate) {
        if (motorType == RobotMotorType.SparkMax) {
            neoMotor.setOpenLoopRampRate(rate);
        } else if (motorType == RobotMotorType.Falcon || motorType == RobotMotorType.TalonSRX
                || motorType == RobotMotorType.Victor) {
            baseCtreMotor.configOpenloopRamp(rate);
        }
    }

    public void setPercent(double percent) {
        if (motorType == RobotMotorType.SparkMax) {
            neoMotor.set(percent);
        } else if (motorType == RobotMotorType.Falcon || motorType == RobotMotorType.TalonSRX
                || motorType == RobotMotorType.Victor) {
            baseCtreMotor.set(ControlMode.PercentOutput, percent);
        }
        lastAssignedPercentage = percent;
        lastAssignedPosition = 0;
        lastAssignedVelocity = 0;
        lastAssignedFeedForward = 0;
        lastAssignedTime = timer.get();
    }

    // Attempt to move the motor to a specific position using a PID loop
    // Use setEncoderValue to change the current encoder value without moving the motor
    public void setPosition(double position) {
        if (motorType == RobotMotorType.SparkMax) {
            neoPID.setReference(position, CANSparkMax.ControlType.kPosition);
            // /rLog.print("Position to Set: " + position);
        } else if (motorType == RobotMotorType.Falcon || motorType == RobotMotorType.TalonSRX
                || motorType == RobotMotorType.Victor) {
            baseCtreMotor.set(ControlMode.Position, (position * encoderTicksPerUnit));
        }
        lastAssignedPercentage = 0;
        lastAssignedPosition = position;
        lastAssignedVelocity = 0;
        lastAssignedFeedForward = 0;
        lastAssignedTime = timer.get();
    }

    // Attempt to move the motor at a specific velocity using a PID loop
    public void setVelocity(double velocity) {
        double feedForward = getPercentageOutputFromVelocity(velocity);
        if (motorType == RobotMotorType.SparkMax) {
            // rLog.print("setVelocity:" + velocity + " AFF:" + feedForward);
            neoPID.setReference(velocity, CANSparkMax.ControlType.kVelocity, 0, feedForward, ArbFFUnits.kVoltage);
        } else if (motorType == RobotMotorType.Falcon || motorType == RobotMotorType.TalonSRX
                || motorType == RobotMotorType.Victor) {
            baseCtreMotor.set(ControlMode.Velocity, (velocity * encoderTicksPerUnit * 0.1),
                    DemandType.ArbitraryFeedForward, feedForward);
        }
        lastAssignedPercentage = 0;
        lastAssignedPosition = 0;
        lastAssignedVelocity = velocity;
        lastAssignedFeedForward = feedForward;
        lastAssignedTime = timer.get();
    }

    // Calculate the best guess of the voltage perctage output (from -1.0 to 1.0) that
    // will be needed for a given velocity.  This value can be used as a feed forward
    // value for a PID loop.
    private double getPercentageOutputFromVelocity(double velocity_In) {
        double percent_Out = 0;
        if (velocity_In >= 0) {
            if (velocity_In < 0.01) {
                percent_Out = 0;
            } else if (velocity_In <= velocityValue[0]) {
                percent_Out = percentValue[0];
            } else if (velocity_In >= velocityValue[velocityValue.length - 1]) {
                percent_Out = percentValue[percentValue.length - 1];
            } else {
                boolean found = false;
                for (int v = 1; !found; v++) {
                    if (velocity_In <= velocityValue[v]) {
                        double rise = velocityValue[v] - velocityValue[v - 1];
                        double run = percentValue[v] - percentValue[v - 1];
                        double slope = rise / run;
                        percent_Out = ((velocity_In - velocityValue[v - 1]) / slope) + percentValue[v - 1];
                        found = true;
                    }
                }
            }
        } else {
            percent_Out = (-1 * getPercentageOutputFromVelocity(-1 * velocity_In));
        }
        return percent_Out;
    }

    public void setMaxOutput(double percentOutput) {
        if (motorType == RobotMotorType.SparkMax) {
            neoPID.setOutputRange(-percentOutput, percentOutput);
        } else if (motorType == RobotMotorType.Falcon || motorType == RobotMotorType.TalonSRX
                || motorType == RobotMotorType.Victor) {
            baseCtreMotor.configClosedLoopPeakOutput(0, percentOutput);
        }
    }

    // The Spark Max requires this method to be called after changing parameters
    public void burnFlash() {
        if (motorType == RobotMotorType.SparkMax) {
            neoMotor.burnFlash();
        } else if (motorType == RobotMotorType.Falcon || motorType == RobotMotorType.TalonSRX
                || motorType == RobotMotorType.Victor) {

        }
    }

    public void stopMotor() {
        setPercent(0);
    }

    public double getOutputPercent() {
        double outputPercent = 0;
        if (motorType == RobotMotorType.SparkMax) {
            outputPercent = neoMotor.getAppliedOutput();
        } else if (motorType == RobotMotorType.Falcon || motorType == RobotMotorType.TalonSRX
                || motorType == RobotMotorType.Victor) {
            outputPercent = baseCtreMotor.getMotorOutputPercent();
        }
        return outputPercent;
    }

    public double getLastAssignedPercentage() {
        return lastAssignedPercentage;
    }

    public double getLastAssignedVelocity() {
        return lastAssignedVelocity;
    }

    public double getLastAssignedPosition() {
        return lastAssignedPosition;
    }

    public double getLastAssignedFeedForward() {
        return lastAssignedFeedForward;
    }

    public void rampToPercent(double percent, double maxPercentPerSecond) {
        double timeBetween = timer.get() - lastAssignedTime;
        double maxChange = maxPercentPerSecond * timeBetween;
        percent = Math.max(Math.min(percent, lastAssignedPercentage + maxChange), lastAssignedPercentage - maxChange);
    }

    public void rampToVelocity(double velocity, double maxVelocityPerSecond) {
        double timeBetween = timer.get() - lastAssignedTime;
        double maxChange = maxVelocityPerSecond * timeBetween;
        velocity = Math.max(Math.min(velocity, lastAssignedVelocity + maxChange), lastAssignedVelocity - maxChange);
    }

    public void setNeutralModeDeadband(double percentDeadband) {
        if (motorType == RobotMotorType.SparkMax) {

        } else if (motorType == RobotMotorType.Falcon || motorType == RobotMotorType.TalonSRX
                || motorType == RobotMotorType.Victor) {
            baseCtreMotor.configNeutralDeadband(percentDeadband);
        }
    }

    public void setNeutralMode(boolean isCoast) {
        if (this.isCoast == isCoast)
            return;
        this.isCoast = isCoast;
        if (motorType == RobotMotorType.SparkMax) {
            neoMotor.setIdleMode(isCoast ? IdleMode.kCoast : IdleMode.kBrake);
        } else if (motorType == RobotMotorType.Falcon || motorType == RobotMotorType.TalonSRX
                || motorType == RobotMotorType.Victor) {
            baseCtreMotor.setNeutralMode(isCoast ? NeutralMode.Coast : NeutralMode.Brake);
        }
    }
}