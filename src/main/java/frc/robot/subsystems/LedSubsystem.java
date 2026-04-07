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
    private int ledGroup;
    private final int groupedLength;

    //disabled pattern
    private final double travelTime;

    //tell the programer the robot is working
    public final Timer deployedWait = new Timer();
    private boolean LedFree = false;


    public LedSubsystem(int LedLength, int LedPort, double travelTime,int ledGroup) {
        Led = new AddressableLED(LedPort);
    ledBuffer = new AddressableLEDBuffer(LedLength);
    this.LedLength = LedLength;
    this.travelTime = travelTime;
    this.ledGroup = ledGroup;
    groupedLength = (int) (LedLength/ledGroup);

    // sets a default pettern, to make it easier to detect full deploy
    for (int i = 0; i < groupedLength; i++) {
      generateLeds(i, 0,255,10);
    }

    Led.setLength(LedLength);
    Led.setData(ledBuffer);
    Led.start();
    deployedWait.start();
  }

  public void SignalEndDeploy() {
    for (int i = 0; i < groupedLength; i++) {
      generateLeds(i,60,255,10);
    }
    Led.setData(ledBuffer);
    LedFree = true;
  }

  public void pattern() {
    if (!LedFree) {
      return;
    }
    int timerPixel = (int)(((groupedLength/travelTime)* deployedWait.get()));
    if ((int)(timerPixel/groupedLength)%2 == 0) {
      generateLeds(timerPixel%groupedLength, 30, 255, 10);
      Led.setData(ledBuffer);
      generateLeds(timerPixel%groupedLength, 100, 255, 1);
    }else{
      generateLeds( groupedLength-(timerPixel%groupedLength)-1, 30, 255, 10);
      Led.setData(ledBuffer);
      generateLeds( groupedLength-(timerPixel%groupedLength)-1, 100, 255, 1);
      
    }
  }

  private void generateLeds(int index, int hue, int saturation, int value) {
    for (int idx = 0;idx<ledGroup;idx++) {
      ledBuffer.setHSV((index*ledGroup)+idx, hue, saturation, value);
    }
  }
}