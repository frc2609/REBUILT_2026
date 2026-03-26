package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {
    private final int ledLength;
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;
    private final Timer deployedWait = new Timer();
    private boolean ledFree = false;
    private int animStep = 0;

    public LedSubsystem(int ledLength, int ledPort) {
        led = new AddressableLED(ledPort);
        ledBuffer = new AddressableLEDBuffer(ledLength);
        this.ledLength = ledLength;

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

    public void pattern() {
        if (!ledFree) {
            return;
        }

        int totalSteps = ledLength * 2;
        int step = animStep % totalSteps;

        for (int i = 0; i < ledLength; i++) {
            ledBuffer.setHSV(i, 0, 0, 0);
        }

        int pixelIndex = step < ledLength ? step : totalSteps - 1 - step;
        ledBuffer.setHSV(pixelIndex, 0, 255, 10);

        led.setData(ledBuffer);
        animStep++;
    }
}
