package frc.robot.subsystems;

import java.security.PublicKey;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {

    private final int ledLength;
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;
    private int ledGroup;
    private final int groupedLength;

    // disabled pattern
    private final double travelTime;

    // tell the programer the robot is working
    public final Timer deployedWait = new Timer();
    private boolean ledFree = false;

    public LedSubsystem(int ledLength, int ledPort, double travelTime, int ledGroup) {
        led = new AddressableLED(ledPort);
        ledBuffer = new AddressableLEDBuffer(ledLength);
        this.ledLength = ledLength;
        this.travelTime = travelTime;
        this.ledGroup = ledGroup;
        groupedLength = (int) (ledLength / ledGroup);

        // sets a default pettern, to make it easier to detect full deploy
        for (int i = 0; i < groupedLength; i++) {
            generateLeds(i, 0, 255, 10);
        }

        led.setLength(ledLength);
        led.setData(ledBuffer);
        led.start();
        deployedWait.start();
    }

    public void SignalEndDeploy() {
        for (int i = 0; i < groupedLength; i++) {
            generateLeds(i, 60, 255, 10);
        }
        led.setData(ledBuffer);
        ledFree = true;
    }

    public void pattern() {
        if (!ledFree) {
            return;
        }
        int timerPixel = (int) (((groupedLength / travelTime) * deployedWait.get()));
        if ((int) (timerPixel / groupedLength) % 2 == 0) {
            generateLeds(timerPixel % groupedLength, 30, 255, 10);
            led.setData(ledBuffer);
            generateLeds(timerPixel % groupedLength, 100, 255, 1);
        } else {
            generateLeds(groupedLength - (timerPixel % groupedLength) - 1, 30, 255, 10);
            led.setData(ledBuffer);
            generateLeds(groupedLength - (timerPixel % groupedLength) - 1, 100, 255, 1);

        }
    }

    private void generateLeds(int index, int hue, int saturation, int value) {
        for (int idx = 0; idx < ledGroup; idx++) {
            ledBuffer.setHSV((index * ledGroup) + idx, hue, saturation, value);
        }
    }

    public void ShotValid(boolean isValidShot) {
        for (int i = 0; i<groupedLength;i++) {
            if (i%2==0){
                if (isValidShot) {
                    generateLeds(i, 60, 255, 10);
                } else{
                    generateLeds(i, 100, 255, 10);
                }
            }
        }
        led.setData(ledBuffer);
    }


    public void HubActive() {
        for (int i = 0; i<groupedLength;i++) {
            if (i%2==1){
                if (isHubActive()) {
                    generateLeds(i, 60, 255, 10);
                } else{
                    generateLeds(i, 30, 255, 10);
                }
            }
        }
        led.setData(ledBuffer);
    }

    public static boolean isHubActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        // If we have no alliance, we cannot be enabled, therefore no hub.
        if (alliance.isEmpty()) {
            return false;
        }
        // Hub is always enabled in autonomous.
        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }
        // At this point, if we're not teleop enabled, there is no hub.
        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }

        // We're teleop enabled, compute.
        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
        if (gameData.isEmpty()) {
            return true;
        }
        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
            // If we have invalid game data, assume hub is active.
            return true;
            }
        }

        // Shift was is active for blue if red won auto, or red if blue won auto.
        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        if (matchTime > 130) {
            // Transition shift, hub is active.
            return true;
        } else if (matchTime > 105) {
            // Shift 1
            return shift1Active;
        } else if (matchTime > 80) {
            // Shift 2
            return !shift1Active;
        } else if (matchTime > 55) {
            // Shift 3
            return shift1Active;
        } else if (matchTime > 30) {
            // Shift 4
            return !shift1Active;
        } else {
            // End game, hub always active.
            return true;
        }
    }
}