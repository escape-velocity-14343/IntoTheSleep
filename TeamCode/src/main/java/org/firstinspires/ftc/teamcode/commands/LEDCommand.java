package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;

import java.util.Map;

import kotlin.jvm.functions.Function3;

public class LEDCommand extends CommandBase {
    LEDSubsystem leds;
    private Function3<Integer, String, Boolean, Boolean> ledPredicate;

    private LEDCommand(LEDSubsystem leds, Function3<Integer, String, Boolean, Boolean> ledPredicate) {
        this.leds = leds;
        this.ledPredicate = ledPredicate;
        addRequirements(this.leds);
    }

    @Override
    public void execute() {
        leds.setServosState(ledPredicate);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    /**
     * Turns on all LEDs
     * @param leds The {@link org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem} to use
     * @return A new LEDCommand
     */
    public static LEDCommand on(LEDSubsystem leds) {
        return setState(leds, true);
    }

    /**
     * Turns off all LEDs
     * @param leds The {@link org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem} to use
     * @return A new LEDCommand
     */
    public static LEDCommand off(LEDSubsystem leds) {
        return setState(leds, false);
    }

    /**
     * @param leds The {@link org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem} to use
     * @param state The state to set all the LEDs to
     * @return A new LEDCommand
     */
    public static LEDCommand setState(LEDSubsystem leds, boolean state) {
        return new LEDCommand(leds, (i, key, enabled) -> state);
    }

    /**
     * @param leds The {@link org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem} to use
     * @return A new LEDCommand
     */
    public static LEDCommand invertState(LEDSubsystem leds) {
        return new LEDCommand(leds, (i, key, enabled) -> !enabled);
    }

    /**
     * @param leds The {@link org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem} to use
     * @param function A function that accepts the index, the name, and the current state of each LED (servo)
     * @return A new LEDCommand
     */
    public static LEDCommand fromFunction(LEDSubsystem leds, Function3<Integer, String, Boolean, Boolean> function) {
        return new LEDCommand(leds, function);
    }
}
