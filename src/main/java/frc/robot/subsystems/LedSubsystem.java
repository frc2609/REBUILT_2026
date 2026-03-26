package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {

    @FunctionalInterface
    public interface LedPattern {
        void apply(AddressableLEDBuffer buf, int start, int length, int step);

        static LedPattern solid(int h, int s, int v) {
            return (buf, start, len, step) -> {
                for (int i = start; i < start + len; i++) {
                    buf.setHSV(i, h, s, v);
                }
            };
        }

        static LedPattern blink(int h, int s, int v, int periodSteps) {
            return (buf, start, len, step) -> {
                boolean on = (step / periodSteps) % 2 == 0;
                int value = on ? v : 0;
                int sat   = on ? s : 0;
                for (int i = start; i < start + len; i++) {
                    buf.setHSV(i, h, sat, value);
                }
            };
        }

        static LedPattern bounce(int h, int s, int v) {
            return (buf, start, len, step) -> {
                int totalSteps = len * 2;
                int t = step % totalSteps;
                int pixel = t < len ? t : totalSteps - 1 - t;
                buf.setHSV(start + pixel, h, s, v);
            };
        }

        static LedPattern off() {
            return (buf, start, len, step) -> {
                for (int i = start; i < start + len; i++) {
                    buf.setHSV(i, 0, 0, 0);
                }
            };
        }
    }

    public record PatternEntry(BooleanSupplier trigger, int start, int length, LedPattern pattern) {
        public static PatternEntry entry(BooleanSupplier trigger, int start, int length, LedPattern pattern) {
            return new PatternEntry(trigger, start, length, pattern);
        }
    }

    private final int ledLength;
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;
    private final Timer deployedWait = new Timer();
    private final PatternEntry[] entries;
    private boolean ledFree = false;
    private int animStep = 0;

    public LedSubsystem(int ledLength, int ledPort, PatternEntry... entries) {
        led = new AddressableLED(ledPort);
        ledBuffer = new AddressableLEDBuffer(ledLength);
        this.ledLength = ledLength;
        this.entries = entries;

        for (int i = 0; i < ledLength; i++) {
            ledBuffer.setHSV(i, 0, 255, 10);
        }

        led.setLength(ledLength);
        led.setData(ledBuffer);
        led.start();
        deployedWait.start();
    }

    public boolean isDeployComplete() {
        return deployedWait.get() > 5;
    }

    public void signalEndDeploy() {
        for (int i = 0; i < ledLength; i++) {
            ledBuffer.setHSV(i, 60, 255, 10);
        }
        led.setData(ledBuffer);
        ledFree = true;
    }

    /** Called automatically by the WPILib scheduler during enabled mode. */
    @Override
    public void periodic() {
        if (DriverStation.isDisabled() || !ledFree) {
            return;
        }

        for (int i = 0; i < ledLength; i++) {
            ledBuffer.setHSV(i, 0, 0, 0);
        }

        for (PatternEntry entry : entries) {
            if (entry.trigger().getAsBoolean()) {
                entry.pattern().apply(ledBuffer, entry.start(), entry.length(), animStep);
            }
        }

        led.setData(ledBuffer);
        animStep++;
    }

    /** Called from RobotContainer.disabledPeriodic() to show the idle bounce animation. */
    public void updateDisabled() {
        if (!ledFree) {
            return;
        }

        for (int i = 0; i < ledLength; i++) {
            ledBuffer.setHSV(i, 0, 0, 0);
        }

        int totalSteps = ledLength * 2;
        int t = animStep % totalSteps;
        int pixel = t < ledLength ? t : totalSteps - 1 - t;
        for (int offset = -1; offset <= 1; offset++) {
            int p = pixel + offset;
            if (p >= 0 && p < ledLength) {
                ledBuffer.setHSV(p, 0, 255, 10);
            }
        }

        led.setData(ledBuffer);
        animStep++;
    }
}
