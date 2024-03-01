package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardUpdater 
{
    private final SendableChooser<String> autoChooser = new SendableChooser<>();

    public static final String kDefaultAuto = "Default Path";
    public static final String kAuto1       = "Red Path";
    public static final String kAuto2       = "Orange Path";
    public static final String kAuto3       = "Yellow Path";
    public static final String kAuto4       = "Green Path";
    public static final String kAuto5       = "Blue Path";
    public static final String kAuto6       = "Purple Path";
    public static final String kAuto7       = "Teal Path";

    private final SendableChooser<String> testChooser = new SendableChooser<>();
    
    public static final String kDefaultTest = "Test Default";
    public static final String kTest1       = "Test Index";
    public static final String kTest2       = "Test Shooter";
    public static final String kTest3       = "Test Drivetrain";
    public static final String kTest4       = "Test Intake";
    public static final String kTest5       = "Test Climb";

    /**
     * Constructor
     */
    public SmartDashboardUpdater () {

    }

    /**
     * Populates the smartdashboard
     */
    public void setupSmartDashboard() {
        setAutonomousOptions();
        setTestOptions();
    }

    /**
     * Sets up the dropdown menu for auto paths
     */
    private void setAutonomousOptions() {
        autoChooser.setDefaultOption("Default Path", kDefaultAuto);
        autoChooser.addOption       ("Red Path", kAuto1);
        autoChooser.addOption       ("Orange Path", kAuto2);
        autoChooser.addOption       ("Yellow Path", kAuto3);
        autoChooser.addOption       ("Green Path", kAuto4);
        autoChooser.addOption       ("Blue Path", kAuto5);
        autoChooser.addOption       ("Purple Path", kAuto6);
        autoChooser.addOption       ("Teal Path", kAuto7);

        SmartDashboard.putData("Autonomuos Choices", autoChooser);
    }

    /**
     * Returns the option selected on the smartdashboard
     * @return String of the auto path selected
     */
    public String getAutoSelected() {
        return autoChooser.getSelected();
    }

    /**
     * Sets up the dropdown menu for which subsystem to test
     */
    private void setTestOptions() {
        testChooser.setDefaultOption("Test Default", kDefaultTest);
        testChooser.addOption       ("Test Index", kTest1);
        testChooser.addOption       ("Test Outtake", kTest2);
        testChooser.addOption       ("Test DriveTrain", kTest3);
        testChooser.addOption       ("Test Intake", kTest4);
        testChooser.addOption       ("Test Climb", kTest5);

        SmartDashboard.putData("Test Choices", testChooser);
    }

    /**
     * Returns the option selected on the smartdashboard
     * @return String of the test function selected
     */
    public String getTestSelected() {
        return testChooser.getSelected();
    }
}
