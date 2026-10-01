package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Reads user input continuously from a wide variety of human interface
 * devices (including, eventually, keyboard input).  The directional inputs
 * are exposed through the three channel querying methods {@link
 * #getForwardBack()}, {@link #getLeftRight()}, and {@link #getTurn()}, while
 * any remaining buttons or sliders are exposed through their own methods.
 *
 * <p>On joystick support:</p>
 * <ul>
 *   <li>
 *     <p>The system supports up to two joysticks, known as the <em>main</em>
 *     and <em>secondary</em> joytsticks.  If there is only one joystick
 *     plugged into the system, it will be considered the main joystick; if
 *     there are two plugged in, whichever one was plugged in first will be
 *     main and the most recent one will be secondary.</p>
 *     <ul>
 *       <li> Any joysticks plugged in beyond the first two will be ignored.</li>
 *     </ul>
 *   </li>
 *   <li>
 *     <p>If only one joystick is plugged into the system, then the mappings
 *     to the driver channels are straightforward:</p>
 *     <ul>
 *       <li>X axis: {@link #getLeftRight() leftRight}</li>
 *       <li>Y axis: {@link #getForwardBack() forwardBack}</li>
 *       <li>Z axis (twisting the joystick): {@link #getTurn() turn}</li>
 *     </ul>
 *   </li>
 *   <li>
 *     <p>When two joysticks are plugged in, the mappings change as follows:</p>
 *     <ul>
 *       <li>Main joystick, X axis: {@link #getLeftRight() leftRight}</li>
 *       <li>Main joystick, Y axis: {@link #getForwardBack() forwardBack}</li>
 *       <li>Secondary joystick, X axis: {@link #getTurn() turn}</li>
 *     </ul>
 *   </li>
 * </ul>
 * <p>See `README.md` for a complete list of joystick and controller button
 * mappings.</p>
 */
public class InputSubsystem extends SubsystemBase {

    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
    private XboxController xboxController;
    private Joystick mainJoystick;
    private Joystick secondaryJoystick;

    /**
     * How often should we look for new human input devices?
     */
    private static final double JOYSTICK_POLLING_INTERVAL_SECONDS = 3.0;

    /**
     * When we last looked for new human input devices, measured in seconds
     * since the robot powered on.
     */
    private double lastCheckTimeSeconds;

    int currentHeight;

    public InputSubsystem() {
        xboxController = null;
        mainJoystick = null;
        secondaryJoystick = null;
        lastCheckTimeSeconds = Timer.getFPGATimestamp() - JOYSTICK_POLLING_INTERVAL_SECONDS; // Force initial check
        currentHeight = 0;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String getName() {
        return "Input Subsystem";
    }

    @Override
    public void periodic() {
        super.periodic();

        // Check to see if anything new was plugged in (or removed.)
        double currentTimeSeconds = Timer.getFPGATimestamp();
        if (currentTimeSeconds - lastCheckTimeSeconds >= JOYSTICK_POLLING_INTERVAL_SECONDS) {
            controllerCheck();
            lastCheckTimeSeconds = currentTimeSeconds;
        }
    }

    /**
     * How much does the user want to strafe sideways?  -1.0 is full speed
     * leftward, +1.0 is full speed rightward, and 0.0 means that no strafing
     * should take place.
     *
     * <ul>
     *   <li>On the Xbox controller, this is the left joystick's horizontal channel.</li>
     *   <li>For one-joystick setups, this is the horizontal channel.</li>
     *   <li>For two-joystick setups, this is the horizontal channel of the
     *   primary (first) joystick.</li>
     *   <li>For keyboard input, this is controlled by the A and D keys.</li>
     * </ul>
     */
    public double getLeftRight() {
        double xboxLeftRight = (xboxController != null && xboxController.isConnected() ? xboxController.getLeftX() : 0);
        double joystickLeftRight = (mainJoystick != null && mainJoystick.isConnected() ? mainJoystick.getX() : 0);

        // If all devices are plugged in, then they'll all contribute so long as
        // a human is touching them.  This allows for pair driving.
        double leftRight = MathUtil.applyDeadband(xboxLeftRight, Constants.OperatorConstants.JOYSTICK_DEAD_ZONE) +
                           MathUtil.applyDeadband(joystickLeftRight, Constants.OperatorConstants.JOYSTICK_DEAD_ZONE);

        leftRight = MathUtil.clamp(leftRight, -1.0, 1.0);

        return m_xspeedLimiter.calculate(leftRight);
    }

    /**
     * How much does the user want to drive straight ahead?  -1.0 is full speed
     * backward, +1.0 is full speed forward, and 0.0 means that no movement
     * should occur in that axis.
     *
     * <ul>
     *   <li>On the Xbox controller, this is the left joystick's vertical channel.</li>
     *   <li>For one-joystick setups, this is the vertical channel.</li>
     *   <li>For two-joystick setups, this is the vertical channel of the
     *   primary (first) joystick.</li>
     *   <li>For keyboard input, this is controlled by the W and S keys.</li>
     * </ul>
     */
    public double getForwardBack() {
        double xboxForwardBack = (xboxController != null && xboxController.isConnected() ? xboxController.getLeftY() : 0);
        double joystickForwardBack = (mainJoystick != null && mainJoystick.isConnected() ? mainJoystick.getY() : 0);

        double forwardBack = MathUtil.applyDeadband(xboxForwardBack, Constants.OperatorConstants.JOYSTICK_DEAD_ZONE) +
                             MathUtil.applyDeadband(joystickForwardBack, Constants.OperatorConstants.JOYSTICK_DEAD_ZONE);

        forwardBack = MathUtil.clamp(forwardBack, -1.0, 1.0);
        
        return m_yspeedLimiter.calculate(forwardBack);
    }

    /**
     * How much does the user want to turn?  -1.0 is full speed
     * counterclockwise, +1.0 is full speed clockwise, and 0.0 means not to
     * turn.
     *
     * <ul>
     *   <li>On the Xbox controller, this is the right joystick's horizontal channel.</li>
     *   <li>For one-joystick setups, this is the so-called "Z" channel
     *   (produced by twisting the joystick.)</li>
     *   <li>For two-joystick setups, this is the horizontal channel of the
     *   secondary joystick.</li>
     *   <li>For keyboard input, this is controlled by the LEFT and RIGHT keys.</li>
     * </ul>
     */
    public double getTurn() {
        double xboxTurn = (xboxController != null && xboxController.isConnected() ? xboxController.getRightX() : 0);
        double joystickTurn = (secondaryJoystick != null && secondaryJoystick.isConnected() ? secondaryJoystick.getX() :
                               (mainJoystick != null && mainJoystick.isConnected() ? mainJoystick.getZ() : 0));

        double turn = MathUtil.applyDeadband(xboxTurn, Constants.OperatorConstants.JOYSTICK_DEAD_ZONE) +
                      MathUtil.applyDeadband(joystickTurn, Constants.OperatorConstants.JOYSTICK_DEAD_ZONE);

        turn = MathUtil.clamp(turn, -1.0, 1.0);
        
        return m_rotLimiter.calculate(turn);
    }

    /**
     * THIS IS OBSOLETE. WE WILL NOT BE USING THIS AT COMP.
     * A function to return the number of desired rotations of the lift motors based on user input.
     * - Stage 0 represents base height of the lift when idle, which is equal to the height of the trough.
     * - Each additional stage (up to the top) represents the next level of the coral branch.
     *   - Lift stage 1 -> coral level 2
     *   - Lift stage 2 ->  coral level 3
     *   - Lift stage 3 -> coral level 4
     *
     * - Each time the user presses one of the lift triggering buttons,
     *   the lift will automatically transition between the stages.
     */
    public double getDesiredPosition() {
        double[] liftHeights = {
            Constants.ArmConstants.DEFAULT_HEIGHT,
            Constants.ArmConstants.LIFT_HEIGHT_1,
            Constants.ArmConstants.LIFT_HEIGHT_2,
            Constants.ArmConstants.LIFT_HEIGHT_3};


        if(xboxController != null && xboxController.isConnected() == true) {
            if(xboxController.getBButtonPressed() && currentHeight <= liftHeights.length) {
                currentHeight++;
            } else if(xboxController.getAButtonPressed() && currentHeight >= 0) {
                currentHeight--;
            }
        }

        if(mainJoystick != null && mainJoystick.isConnected() && secondaryJoystick != null && secondaryJoystick.isConnected()) {
            if(secondaryJoystick.getRawButtonPressed(6) && currentHeight <= liftHeights.length) {
                currentHeight++;
            } else if(secondaryJoystick.getRawButtonPressed(4) && currentHeight >= 0) {
                currentHeight--;
            }
        } else if(mainJoystick != null && mainJoystick.isConnected()) {
            if(mainJoystick.getRawButtonPressed(6) && currentHeight <= liftHeights.length) {
                currentHeight++;
            } else if(mainJoystick.getRawButtonPressed(4) && currentHeight >= 0) {
                currentHeight--;
            }
        }

        return liftHeights[currentHeight];
    }

    /**
     * This is a separate method of controlling the lift of the ArmSubsystem.
     * It will move the lift at a constant speed as long as the player is holding down the desired movement button.
     * The speed of the lift is dependent on the constant LIFT_SPEED in Constants.java.
     */
    public double getArmMovement() {
        if(xboxController != null && xboxController.isConnected() == true) {
            if (xboxController.getYButton()) {
                return Constants.ArmConstants.LIFT_SPEED * -1;
            } else if (xboxController.getAButton()) {
                return Constants.ArmConstants.LIFT_SPEED;
            } else {
                return 0;
            }
        }

        if(mainJoystick != null && mainJoystick.isConnected() && secondaryJoystick != null && secondaryJoystick.isConnected()) {
            if(secondaryJoystick.getRawButtonPressed(5)) {
                return Constants.ArmConstants.LIFT_SPEED;
            } else if(secondaryJoystick.getRawButtonPressed(3)) {
                return Constants.ArmConstants.LIFT_SPEED * -1;
            }
        } else if(mainJoystick != null && mainJoystick.isConnected()) {
            if(mainJoystick.getRawButtonPressed(5)) {
                return Constants.ArmConstants.LIFT_SPEED;
            } else if(mainJoystick.getRawButtonPressed(3)) {
                return Constants.ArmConstants.LIFT_SPEED * -1;
            }
        }

        return 0;
    }


    /**
     * Returns whether or not the coral flywheels should be moving based on whether or not the player
     * is pressing the right trigger/right bumper button.
     */
    public boolean isCoralIntakeActivated() {
        if(xboxController != null &&xboxController.isConnected() == true) {
            if(xboxController.getBButton()) {
                return true;
            } else {
                return false;
            }
        }

        if(mainJoystick != null && mainJoystick.isConnected() && secondaryJoystick != null && secondaryJoystick.isConnected()) {
            if(secondaryJoystick.getTrigger()) {
                return true;
            }
        } else if(mainJoystick != null && mainJoystick.isConnected()) {
            if(mainJoystick.getTrigger()) {
                return true;
            }
        }

        return false;
    }

    /**
     * A function called every few seconds meant to detect when xbox controllers
     * or joysticks are connected or disconnected, and updates dynamically.
     *
     * <p>To avoid overwhelming the system with exceptions, callers of this
     * function should only do so occasionally.</p>
     */
    private void controllerCheck() {
        final int MAX_PORTS_TO_CHECK = 4;
        for (int i = 0; i < MAX_PORTS_TO_CHECK; i++) {

            try {
                if (xboxController == null) {
                    xboxController = new XboxController(i);
                    //System.out.printf("controllerCheck: Found xbox controller on port %d.\n", i);
                    continue;
                } else if (!xboxController.isConnected()) {
                    xboxController = null;
                    //System.out.printf("controllerCheck: Xbox controller is disconnected.\n");
                    continue;
                }
            } catch (Exception e) {
                // Xbox controller is not connected on port i, not a big deal.
            }

            // Can we still accept another joystick?
            if (!mainJoystickFound() || !secondaryJoystickFound()) {

                if ((mainJoystickFound() && mainJoystick.getPort() == i) ||
                    (secondaryJoystickFound() && secondaryJoystick.getPort() == i)) {
                    // We already know there's a working joystick on this
                    // port.
                    continue;
                }

                // We want to be able to support both 1- and 2-joystick
                // scenarios.
                try {
                    Joystick j = new Joystick(i);
                    if (assignJoystick(j)) {
                        continue;
                    }
                } catch (Exception e) {
                    // No joystick at port i, not a big deal.
                }
            }
            // TODO: Add NetworkTables support.
        }
    }

    /**
     * A simple utility function that returns true if {@link #mainJoystick} is
     * usable.
     */
    private boolean mainJoystickFound() {
        return mainJoystick != null && mainJoystick.isConnected();
    }

    /**
     * A simple utility function that returns true if {@link
     * #secondaryJoystick} is usable.
     */
    private boolean secondaryJoystickFound() {
        return secondaryJoystick != null && secondaryJoystick.isConnected();
    }

    /**
     * Helper function for {@link #controllerCheck()}.  When a new joystick
     * "comes online", this function determines where to assign the new
     * joystick (if assigning is needed at all.)
     *
     * @param j a {@link Joystick} that has just been successfully
     *          constructed.
     * @return Returns true if the joystick was successfully assigned to the
     *         main or secondary joystick and false if there was no need to
     *         assign it.
     */
    private boolean assignJoystick(Joystick j) {
        boolean success = true;
        String message = "";

        if (!mainJoystickFound()) {

            if (!secondaryJoystickFound()) {
                mainJoystick = j;
                message = "assigned new joystick on port %d to main.\n".formatted(j.getPort());
            } else {
                mainJoystick = secondaryJoystick;
                secondaryJoystick = j;
                message = "assigned secondary to the main joystick and assigned new joystick on port %d to the secondary.\n".formatted(j.getPort());
            }
        } else { // Main joystick is connected.

            if (!secondaryJoystickFound()) {
                secondaryJoystick = j;
                message = "assigned new joystick on port %d to secondary.\n".formatted(j.getPort());
            } else {
                // Already have two working joysticks, so 3rd joystick does nothing.
                // Not even worth logging.
                success = false;
            }
        }

        if (success) {
            final String prefix = "controllerCheck";

            final String mainJoystickString = mainJoystick == null ? "Main joystick is missing" :
                !mainJoystick.isConnected() ? "Main joystick on port %d was disconnected".formatted(mainJoystick.getPort()) :
                "Main joystick is connected on port %d".formatted(mainJoystick.getPort());

            final String secondaryJoystickString = secondaryJoystick == null ? "secondary joystick is missing" :
                !secondaryJoystick.isConnected() ? "secondary joystick on port %d was disconnected".formatted(secondaryJoystick.getPort()) :
                "secondary joystick is connected on port %d".formatted(secondaryJoystick.getPort());

            //System.out.printf("%s: %s and %s; %s.\n", prefix, mainJoystickString, secondaryJoystickString, message);
        }
        return success;
    }

    /**
     * Writes the input values to the shuffleboard.
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("InputSubsystem");
        builder.addDoubleProperty("Turn", this::getTurn, null);
        builder.addDoubleProperty("ForwardBack", this::getForwardBack, null);
        builder.addDoubleProperty("LeftRight", this::getLeftRight, null);
        // super.initSendable(builder);
    }

    // TODO We need to call the controller check every few seconds during
    // periodic, and we need to combine the xbox controller, the main joystick,
    // and the secondary joystick into the 3 degrees of freedom: forward-back,
    // left-right, and turn.
}
