package frc.robot.subsystems.io.motor.CTRE;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.System_StateValue;

import frc.robot.Constants;

public class CtreTalonFxIO {
    private final Map<String, Consumer<Object>> setters = new HashMap<>();

    private boolean isRioCANBUS = false;
    public final TalonFX motor;
    public int motorId;
    private TalonFXConfiguration config;

    public TalonFX followerMotor = null;
    private MotorAlignmentValue followerAligned;
    public int followerId = -1;
    public boolean hasFollower = false;

    public LoggedNetworkNumber measuredLogged, setpointLogged, voltageLogged;

    public String NTPath;
    private ArrayList<LoggedNetworkNumber> tunables;
    private double[] tunables_old;

    public CtreTalonFxIO(Map<String, Object> cfg) {
        config = new TalonFXConfiguration();

        // Maps input config to TalonFx config but doesn't set it

        setters.put("motorId", value -> this.motorId = (int) value);
        setters.put("followerId", value -> this.followerId = (int) value);

        setters.put(
            "followerAligned",
            value -> this.followerAligned = toPhoenixFollowerAlignment((Boolean) value));
        setters.put(
            "inverted",
            value -> this.config.MotorOutput.withInverted(toPhoenixInverted((Boolean) value)));
        setters.put(
            "neutralMode",
            value -> this.config.MotorOutput.withNeutralMode(toPhoenixNeutralMode((Constants.NeutralMode) value)));

        setters.put("kP", value -> this.config.Slot0.kP = (double) value);
        setters.put("kI", value -> this.config.Slot0.kI = (double) value);
        setters.put("kD", value -> this.config.Slot0.kD = (double) value);
        setters.put("kA", value -> this.config.Slot0.kA = (double) value);
        setters.put("kV", value -> this.config.Slot0.kV = (double) value);
        setters.put("kS", value -> this.config.Slot0.kS = (double) value);
        setters.put("kG", value -> this.config.Slot0.kG = (double) value);

        setters.put(
            "forwardLimitEnabled",
            value -> this.config.SoftwareLimitSwitch.ForwardSoftLimitEnable = (boolean) value);
        setters.put(
            "reverseLimitEnabled",
            value -> this.config.SoftwareLimitSwitch.ReverseSoftLimitEnable = (boolean) value);
        setters.put(
            "forwardLimitRotations",
            value -> this.config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = (double) value);
        setters.put(
            "reverseLimitRotations",
            value -> this.config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = (double) value);
        
        setters.put(
            "MotionMagicCruiseVelocity", 
            value -> this.config.MotionMagic.MotionMagicCruiseVelocity = (double) value);
        setters.put(
            "MotionMagicAcceleration", 
            value -> this.config.MotionMagic.MotionMagicAcceleration = (double) value);
        setters.put(
            "MotionMagicJerk", 
            value -> this.config.MotionMagic.MotionMagicJerk = (double) value);

        setters.put(
            "supplyCurrentLimit", 
            value -> this.config.CurrentLimits.SupplyCurrentLimit = (double) value);
        setters.put(
            "supplyCurrentLimitEnabled", 
            value -> this.config.CurrentLimits.SupplyCurrentLimitEnable = (boolean) value);
        setters.put(
            "statorCurrentLimit", 
            value -> this.config.CurrentLimits.StatorCurrentLimit = (double) value);
        setters.put(
            "statorCurrentLimitEnabled", 
            value -> config.CurrentLimits.StatorCurrentLimitEnable = (boolean) value);
        
        setters.put(
            "isRioCANBUS", 
            value -> this.isRioCANBUS = (boolean) value);

        setConfiguration(cfg);

        // MotorID and Follower settings are now available

        CANBus CANBUS = isRioCANBUS ? Constants.RioCANBUS : Constants.CANBUS;

        motor = new TalonFX(motorId, CANBUS);

        applyConfiguration();   

        if (followerId > -1) {
            followerMotor = new TalonFX(followerId, CANBUS);
            followerMotor.getConfigurator().apply(config);
            followerMotor.setControl(new Follower(this.motorId, followerAligned));
            hasFollower = true;
        }

        // Set up tuning variables

        NTPath = "/Tuning/"+Constants.motorNames.get(motorId);
        tunables = new ArrayList<LoggedNetworkNumber>();
        
        for (String key : Constants.tunableKeys) {
            if (cfg.get(key) != null)
            {
                tunables.add(new LoggedNetworkNumber(
                    NTPath + "/Tunables/" + key, 
                    (double) cfg.get(key)
                ));
            }
        }

        tunables_old = new double[tunables.size()];
        copyToOldTunables();

        measuredLogged = new LoggedNetworkNumber(NTPath+"/Measured");
        setpointLogged = new LoggedNetworkNumber(NTPath+"/Setpoint");
        voltageLogged = new LoggedNetworkNumber(NTPath+"/PID Output (V)");
    }

    private static com.ctre.phoenix6.signals.NeutralModeValue toPhoenixNeutralMode(
        Constants.NeutralMode mode) {
        return mode == Constants.NeutralMode.BRAKE
            ? com.ctre.phoenix6.signals.NeutralModeValue.Brake
            : com.ctre.phoenix6.signals.NeutralModeValue.Coast;
    }

    private static com.ctre.phoenix6.signals.InvertedValue toPhoenixInverted(boolean inverted) {
        return inverted
            ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
            : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
    }

    private static MotorAlignmentValue toPhoenixFollowerAlignment(Boolean followerAlignment) {
        return followerAlignment ? MotorAlignmentValue.Aligned : MotorAlignmentValue.Opposed;
    }

    public void setConfiguration(Map<String, Object> fields) {
        if (fields == null) return;

        // Always apply incoming fields to the local config/state. 

        fields.forEach((key, value) -> {
            Consumer<Object> setter = setters.get(key);
            if (setter != null) {
                setter.accept(value);
            } else {
                throw new Error("Unknown field: " + key);
            }
        });
    }

    public void applyConfiguration() {
        motor.getConfigurator().apply(config);
        if (hasFollower) {
            followerMotor.getConfigurator().apply(config);
        }
    }

    private void copyToOldTunables()
    {
        for (int i = 0; i < tunables.size(); i++) {
            tunables_old[i] = tunables.get(i).getAsDouble();
        }
    }

    public void updateFromTunables() {
        boolean changed = false;

        for (int i = 0; i < tunables.size(); i++) {
            double value = tunables.get(i).getAsDouble();
            if (value != tunables_old[i])
            {   
                setters.get(Constants.tunableKeys[i]).accept(value);
                if (Constants.tunableKeys[i] == "kS") {
                    System.out.println("kS SET TO: "+value);
                }
                changed = true;
            }
        }

        if (changed)
        {   
            applyConfiguration();
            copyToOldTunables();
        }
    }

    public double getSetpoint() {
        double input = setpointLogged.get();
        // setpointLogged.set(input);
        return input;
    }

    public void setSetpoint(double value) {
        setpointLogged.set(value);
        System.out.println(NTPath+": Set setpoint value to "+value);
    }
}
