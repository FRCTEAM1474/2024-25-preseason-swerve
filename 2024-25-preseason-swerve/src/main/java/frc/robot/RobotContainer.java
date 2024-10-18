// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.io.File;
//import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));

  

  //NamedCommands.registerCommand("speaker shoot", new climbingPosCommand(arm));

  //private Command genArmPos = new genPosCommand(arm, desiredPos);
  
  //add in speaker shoot

  static CommandXboxController driverXbox = new CommandXboxController(0);
  static CommandXboxController operatorXbox = new CommandXboxController(1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  private final SendableChooser<Command> autoChooser;

  public RobotContainer()
  {
    
    // Configure the trigger bindings
    configureBindings();

    AbsoluteDrive closedAbsoluteDrive = new AbsoluteDrive(drivebase,
      // Applies deadbands and inverts controls because joysticks
      // are back-right positive while robot
      // controls are front-left positive
      () -> MathUtil.applyDeadband((-driverXbox.getLeftY()),
        OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband((-driverXbox.getLeftX()),
        OperatorConstants.LEFT_X_DEADBAND),
      () -> -driverXbox.getRightX(),
      () -> -driverXbox.getRightY());

    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
      () -> -MathUtil.applyDeadband(driverXbox.getRightY(),
        OperatorConstants.LEFT_Y_DEADBAND),
      () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
        OperatorConstants.LEFT_X_DEADBAND),
      () -> -driverXbox.getLeftX(),
      () -> -driverXbox.getLeftY(),
      driverXbox.povUp(),
      driverXbox.povDown(),
      driverXbox.povLeft(),
      driverXbox.povRight());
      
    AbsoluteFieldDrive closedFieldAbsoluteDrive = new AbsoluteFieldDrive(drivebase,
      () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
        OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
        OperatorConstants.LEFT_X_DEADBAND),
      () -> driverXbox.getRightX()*360);

    TeleopDrive simClosedFieldRel = new TeleopDrive(drivebase,
      () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
        OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
        OperatorConstants.LEFT_X_DEADBAND),
      () -> driverXbox.getRightX(), () -> true);
    TeleopDrive closedFieldRel = new TeleopDrive(
        drivebase,
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverXbox.getRightX(), () -> true);

    drivebase.setDefaultCommand(!RobotBase.isSimulation() ? closedAbsoluteDriveAdv : closedFieldAbsoluteDrive);

    autoChooser = AutoBuilder.buildAutoChooser(); //default auto will be "Commands.non()"
    SmartDashboard.putData("Auto Chooser", autoChooser);

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    driverXbox.start().onTrue((new InstantCommand(drivebase::zeroGyro)));
    //driverXbox.y().onTrue(new InstantCommand(drivebase::addFakeVisionReading));
    //driverXbox.x().whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
    //driverXbox.povUp().whileTrue(climberUp);
    //driverXbox.povDown().whileTrue(climberDown);
    //driverXbox.povUp().onTrue((new InstantCommand(drivebase::)))


    //operatorXbox.a().onTrue(new climbingPosCommand(arm));
    //operatorXbox.b().onTrue(new ampPosCommand(arm));
    //operatorXbox.x().onTrue(new startingPosCommand(arm));
    //operatorXbox.a().whileTrue(genArmPos);

    /*operatorXbox.a().onTrue(Commands.runOnce(
      () -> {
        arm.setGoal(Arm.kClimbingPos); 
        arm.enable();
      }, arm));

    operatorXbox.b().onTrue(Commands.runOnce(
      () -> {
        arm.setGoal(Arm.kAmpShootPos); 
        arm.enable();
      }, arm));

    operatorXbox.x().onTrue(Commands.runOnce(
      () -> {
        arm.setGoal(Arm.kStartingPos); 
        arm.enable();
      }, arm));
*/
    


  
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(){
    return autoChooser.getSelected();
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}