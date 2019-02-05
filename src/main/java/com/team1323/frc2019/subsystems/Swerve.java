package com.team1323.frc2019.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import com.team1323.frc2019.Constants;
import com.team1323.frc2019.DriveMotionPlanner;
import com.team1323.frc2019.Ports;
import com.team1323.frc2019.RobotState;
import com.team1323.frc2019.loops.ILooper;
import com.team1323.frc2019.loops.Loop;
import com.team1323.frc2019.vision.ShooterAimingParameters;
import com.team1323.lib.math.vectors.VectorField;
import com.team1323.lib.util.SwerveHeadingController;
import com.team1323.lib.util.SwerveInverseKinematics;
import com.team1323.lib.util.Util;
import com.team1323.lib.util.VisionCriteria;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.TimedView;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryGenerator;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Swerve extends Subsystem{
	//Instance declaration
	private static Swerve instance = null;
	public static Swerve getInstance(){
		if(instance == null)
			instance = new Swerve();
		return instance;
	}
	
	//Module declaration
	public SwerveDriveModule frontRight, frontLeft, rearLeft, rearRight;
	List<SwerveDriveModule> modules;
	List<SwerveDriveModule> positionModules;
	
	//Evade maneuver variables
	Translation2d clockwiseCenter = new Translation2d();
	Translation2d counterClockwiseCenter = new Translation2d();
	boolean evading = false;
	boolean evadingToggled = false;
	public void toggleEvade(){
		evading = !evading;
		evadingToggled = true;
	}
	
	//Heading controller methods
	Pigeon pigeon;
	SwerveHeadingController headingController = new SwerveHeadingController();
	public void temporarilyDisableHeadingController(){
		headingController.temporarilyDisable();
	}
	public double getTargetHeading(){
		return headingController.getTargetHeading();
	}

	//Vision dependencies
	RobotState robotState;
	Rotation2d visionTargetHeading = new Rotation2d();
	boolean visionUpdatesAllowed = true;
	public void resetVisionUpdates(){
		visionUpdatesAllowed = true;
		visionUpdateCount = 0;
		visionCriteria.reset();
	}
	double visionCurveDistance = Constants.kDefaultCurveDistance;
	Translation2d visionTargetPosition = new Translation2d();
	int visionUpdateCount = 0;
	VisionCriteria visionCriteria = new VisionCriteria();


	//Name says it all
	TrajectoryGenerator generator;

	//Odometry variables
	Pose2d pose;
	double distanceTraveled;
	double currentVelocity = 0;
	double lastUpdateTimestamp = 0;
	public Pose2d getPose(){
		return pose;
	}

	// Module configuration variables (for beginnning of auto)
	boolean modulesReady = false;
	boolean alwaysConfigureModules = false;
	boolean moduleConfigRequested = false;
	public void requireModuleConfiguration(){
		modulesReady = false;
	}
	public void alwaysConfigureModules(){
		alwaysConfigureModules = true;
	}

	//Trajectory variables
	DriveMotionPlanner motionPlanner;
	public double getRemainingProgress(){
		if(motionPlanner != null && getState() == ControlState.TRAJECTORY){
			return motionPlanner.getRemainingProgress();
		}
		return 0.0;
	}
	double rotationScalar;
	double trajectoryStartTime = 0;
	Translation2d lastTrajectoryVector = new Translation2d();
	boolean hasStartedFollowing = false;
	boolean hasFinishedPath = false;
	public boolean hasFinishedPath(){
		return hasFinishedPath;
	}
	
	//Experimental
	VectorField vf;
	
	private Swerve(){
		frontRight = new SwerveDriveModule(Ports.FRONT_RIGHT_ROTATION, Ports.FRONT_RIGHT_DRIVE,
				0, Constants.kFrontRightEncoderStartingPos, Constants.kVehicleToModuleZero);
		frontLeft = new SwerveDriveModule(Ports.FRONT_LEFT_ROTATION, Ports.FRONT_LEFT_DRIVE,
				1, Constants.kFrontLeftEncoderStartingPos, Constants.kVehicleToModuleOne);
		rearLeft = new SwerveDriveModule(Ports.REAR_LEFT_ROTATION, Ports.REAR_LEFT_DRIVE,
				2, Constants.kRearLeftEncoderStartingPos, Constants.kVehicleToModuleTwo);
		rearRight = new SwerveDriveModule(Ports.REAR_RIGHT_ROTATION, Ports.REAR_RIGHT_DRIVE,
				3, Constants.kRearRightEncoderStartingPos, Constants.kVehicleToModuleThree);
		
		modules = Arrays.asList(frontRight, frontLeft, rearLeft, rearRight);
		positionModules = Arrays.asList(frontRight, frontLeft, rearRight);
		
		//rearLeft.disableDriveEncoder();
		
		rearLeft.invertDriveMotor(true);
		frontLeft.invertDriveMotor(true);
		
		modules.forEach((m) -> m.reverseRotationSensor(true));
				
		pigeon = Pigeon.getInstance();
		
		pose = new Pose2d();
		distanceTraveled = 0;
		
		motionPlanner = new DriveMotionPlanner();

		robotState = RobotState.getInstance();
		generator = TrajectoryGenerator.getInstance();
	}
	
	//Teleop driving variables
	private Translation2d translationalVector = new Translation2d();
	private double rotationalInput = 0;
	private Translation2d lastDriveVector = new Translation2d();
	private final Translation2d rotationalVector = Translation2d.identity();
	private double maxSpeedFactor = 1.0;
	public void setMaxSpeed(double max){
		maxSpeedFactor = max;
	}
	private boolean robotCentric = false;
	
	//Swerve kinematics (exists in a separate class)
	private SwerveInverseKinematics inverseKinematics = new SwerveInverseKinematics();
	public void setCenterOfRotation(Translation2d center){
		inverseKinematics.setCenterOfRotation(center);
	}
	
	//The swerve's various control states
	public enum ControlState{
		NEUTRAL, MANUAL, POSITION, ROTATION, DISABLED, VECTORIZED,
		TRAJECTORY, VELOCITY, VISION
	}
	private ControlState currentState = ControlState.NEUTRAL;
	public ControlState getState(){
		return currentState;
	}
	public void setState(ControlState newState){
		currentState = newState;
	}
	
	/**
	 * Main function used to send manual input during teleop.
	 * @param x forward/backward input
	 * @param y left/right input
	 * @param rotate rotational input
	 * @param robotCentric gyro use
	 * @param lowPower scaled down output
	 */
	public void sendInput(double x, double y, double rotate, boolean robotCentric, boolean lowPower){
		Translation2d translationalInput = new Translation2d(x, y);
		double inputMagnitude = translationalInput.norm();
		
		/* Snap the translational input to its nearest pole, if it is within a certain threshold 
		  of it. */
		double threshold = Math.toRadians(10.0);
		if(Math.abs(translationalInput.direction().distance(translationalInput.direction().nearestPole())) < threshold){
			translationalInput = translationalInput.direction().nearestPole().toTranslation().scale(inputMagnitude);
		}
		
		double deadband = 0.25;
		if(inputMagnitude < deadband){
			translationalInput = new Translation2d();
			inputMagnitude = 0;
		}
		
		/* Scale x and y by applying a power to the magnitude of the vector they create, in order
		 to make the controls less sensitive at the lower end. */
		final double power = (lowPower) ? 1.75 : 1.5;
		Rotation2d direction = translationalInput.direction();
		double scaledMagnitude = Math.pow(inputMagnitude, power);
		translationalInput = new Translation2d(direction.cos() * scaledMagnitude,
				direction.sin() * scaledMagnitude);
		
		rotate = (Math.abs(rotate) < deadband) ? 0 : rotate;
		rotate = Math.pow(Math.abs(rotate), 1.75)*Math.signum(rotate);
		
		translationalInput = translationalInput.scale(maxSpeedFactor);
		rotate *= maxSpeedFactor;
				
		translationalVector = translationalInput;
		
		if(lowPower){
			translationalVector = translationalVector.scale(0.6);
			rotate *= 0.3;
		}else{
			rotate *= 0.8;
		}
		
		if(rotate != 0 && rotationalInput == 0){
			headingController.disable();
		}else if(rotate == 0 && rotationalInput != 0){
			headingController.temporarilyDisable();
		}
		
		rotationalInput = rotate;

		if(translationalInput.norm() != 0 || rotationalInput != 0){
			if(currentState == ControlState.VISION){
				if(Math.abs(translationalInput.direction().distance(visionTargetHeading)) > Math.toRadians(90.0)){
					setState(ControlState.MANUAL);
				}
			}else if(currentState != ControlState.MANUAL){
				setState(ControlState.MANUAL);
			}
		}

		if(inputMagnitude > 0.3)
			lastDriveVector = new Translation2d(x, y);
		else if(translationalVector.x() == 0.0 && translationalVector.y() == 0.0 && rotate != 0.0){
			lastDriveVector = rotationalVector;
		}
		
		this.robotCentric = robotCentric;
	}

	//Possible new control method for rotation
	public Rotation2d averagedDirection = Rotation2d.identity();
	public void resetAveragedDirection(){ averagedDirection = pose.getRotation(); }
	public void setAveragedDirection(double degrees){ averagedDirection = Rotation2d.fromDegrees(degrees); }
	public final double rotationDirectionThreshold = Math.toRadians(5.0);
	public final double rotationDivision = 1.0;
	public synchronized void updateControllerDirection(Translation2d input){
		if(Util.epsilonEquals(input.norm(), 1.0, 0.1)){
			Rotation2d direction = input.direction();
			double roundedDirection = Math.round(direction.getDegrees() / rotationDivision) * rotationDivision;
			averagedDirection = Rotation2d.fromDegrees(roundedDirection);
		}
	}
	
	//Various methods to control the heading controller
	public synchronized void rotate(double goalHeading){
		if(translationalVector.x() == 0 && translationalVector.y() == 0)
			rotateInPlace(goalHeading);
		else
			headingController.setStabilizationTarget(
					Util.placeInAppropriate0To360Scope(pose.getRotation().getUnboundedDegrees(), goalHeading));
	}
	
	public void rotateInPlace(double goalHeading){
		setState(ControlState.ROTATION);
		headingController.setStationaryTarget(
				Util.placeInAppropriate0To360Scope(pose.getRotation().getUnboundedDegrees(), goalHeading));
	}
	
	public void rotateInPlaceAbsolutely(double absoluteHeading){
		setState(ControlState.ROTATION);
		headingController.setStationaryTarget(absoluteHeading);
	}
	
	public void setPathHeading(double goalHeading){
		headingController.setSnapTarget(
				Util.placeInAppropriate0To360Scope(
						pose.getRotation().getUnboundedDegrees(), goalHeading));
	}
	
	public void setAbsolutePathHeading(double absoluteHeading){
		headingController.setSnapTarget(absoluteHeading);
	}
	
	/** Sets MotionMagic targets for the drive motors */
	public void setPositionTarget(double directionDegrees, double magnitudeInches){
		setState(ControlState.POSITION);
		modules.forEach((m) -> m.setModuleAngle(directionDegrees));
		modules.forEach((m) -> m.setDrivePositionTarget(magnitudeInches));
	}

	/** Locks drive motors in place with MotionMagic */
	public void lockDrivePosition(){
		modules.forEach((m) -> m.setDrivePositionTarget(0.0));
	}

	/** Puts drive motors into closed-loop velocity mode */
	public void setVelocity(Rotation2d direction, double velocityInchesPerSecond){
		setState(ControlState.VELOCITY);
		modules.forEach((m) -> m.setModuleAngle(direction.getDegrees()));
		modules.forEach((m) -> m.setVelocitySetpoint(velocityInchesPerSecond));
	}
	
	/** Configures each module to match its assigned vector */
	public void setDriveOutput(List<Translation2d> driveVectors){
		for(int i=0; i<modules.size(); i++){
    		if(Util.shouldReverse(driveVectors.get(i).direction().getDegrees(), modules.get(i).getModuleAngle().getDegrees())){
    			modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees() + 180.0);
    			modules.get(i).setDriveOpenLoop(-driveVectors.get(i).norm());
    		}else{
    			modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees());
    			modules.get(i).setDriveOpenLoop(driveVectors.get(i).norm());
    		}
    	}
	}

	public void setDriveOutput(List<Translation2d> driveVectors, double percentOutputOverride){
		for(int i=0; i<modules.size(); i++){
    		if(Util.shouldReverse(driveVectors.get(i).direction().getDegrees(), modules.get(i).getModuleAngle().getDegrees())){
    			modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees() + 180.0);
    			modules.get(i).setDriveOpenLoop(-percentOutputOverride);
    		}else{
    			modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees());
    			modules.get(i).setDriveOpenLoop(percentOutputOverride);
    		}
    	}
	}


	/** Configures each module to match its assigned vector, but puts the drive motors into closed-loop velocity mode */
	public void setVelocityDriveOutput(List<Translation2d> driveVectors){
		for(int i=0; i<modules.size(); i++){
    		if(Util.shouldReverse(driveVectors.get(i).direction().getDegrees(), modules.get(i).getModuleAngle().getDegrees())){
    			modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees() + 180.0);
    			modules.get(i).setVelocitySetpoint(-driveVectors.get(i).norm() * Constants.kSwerveMaxSpeedInchesPerSecond);
    		}else{
    			modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees());
    			modules.get(i).setVelocitySetpoint(driveVectors.get(i).norm() * Constants.kSwerveMaxSpeedInchesPerSecond);
    		}
    	}
	}

	public void setVelocityDriveOutput(List<Translation2d> driveVectors, double velocityOverride){
		for(int i=0; i<modules.size(); i++){
    		if(Util.shouldReverse(driveVectors.get(i).direction().getDegrees(), modules.get(i).getModuleAngle().getDegrees())){
    			modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees() + 180.0);
    			modules.get(i).setVelocitySetpoint(-velocityOverride);
    		}else{
    			modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees());
    			modules.get(i).setVelocitySetpoint(velocityOverride);
    		}
    	}
	}

	/** Sets only module angles to match their assigned vectors */
	public void setModuleAngles(List<Translation2d> driveVectors){
		for(int i=0; i<modules.size(); i++){
    		if(Util.shouldReverse(driveVectors.get(i).direction().getDegrees(), modules.get(i).getModuleAngle().getDegrees())){
    			modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees() + 180.0);
    		}else{
    			modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees());
    		}
    	}
	}

	/** Increases each module's rotational power cap for the beginning of auto */
	public void set10VoltRotationMode(boolean tenVolts){
		modules.forEach((m) -> m.set10VoltRotationMode(tenVolts));
	}
	
	/**
	 * @return Whether or not at least one module has reached its MotionMagic setpoint
	 */
	public boolean positionOnTarget(){
		boolean onTarget = false;
		for(SwerveDriveModule m : modules){
			onTarget |= m.drivePositionOnTarget();
		}
		return onTarget;
	}
	
	/**
	 * @return Whether or not all modules have reached their angle setpoints
	 */
	public boolean moduleAnglesOnTarget(){
		boolean onTarget = true;
		for(SwerveDriveModule m : modules){
			onTarget &= m.angleOnTarget();
		}
		return onTarget;
	}

	/**
	 * Sets a trajectory for the robot to follow
	 * @param trajectory 
	 * @param targetHeading Heading that the robot will rotate to during its path following
	 * @param rotationScalar Scalar to increase or decrease the robot's rotation speed
	 * @param followingCenter The point (relative to the robot) that will follow the trajectory
	 */
	public synchronized void setTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> trajectory, double targetHeading,
		double rotationScalar, Translation2d followingCenter){
			hasStartedFollowing = false;
			hasFinishedPath = false;
			moduleConfigRequested = false;
			motionPlanner.reset();
			motionPlanner.setTrajectory(new TrajectoryIterator<>(new TimedView<>(trajectory)));
			motionPlanner.setFollowingCenter(followingCenter);
			inverseKinematics.setCenterOfRotation(followingCenter);
			setAbsolutePathHeading(targetHeading);
			this.rotationScalar = rotationScalar;
			trajectoryStartTime = Timer.getFPGATimestamp();
			setState(ControlState.TRAJECTORY);
		}
	
	public synchronized void setTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> trajectory, double targetHeading,
			double rotationScalar){
		setTrajectory(trajectory, targetHeading, rotationScalar, Translation2d.identity());
	}

	private synchronized void setCurvedVisionTrajectory(double linearShiftDistance, Optional<ShooterAimingParameters> aimingParameters){
		visionCurveDistance = linearShiftDistance;
		System.out.println("Vision Curve Distance: " + visionCurveDistance);
		visionTargetPosition = robotState.getCaptureTimeFieldToGoal().get(2).getTranslation();
		Optional<Pose2d> orientedTarget = robotState.getOrientedTargetPosition(aimingParameters);
		Optional<Pose2d> robotScoringPosition = robotState.getRobotScoringPosition(aimingParameters);
		if(orientedTarget.isPresent() && robotState.seesTarget() && visionUpdatesAllowed){
			Translation2d deltaPosition = orientedTarget.get().transformBy(Pose2d.fromTranslation(new Translation2d(-linearShiftDistance, 0.0))).getTranslation().translateBy(pose.getTranslation().inverse());
			Rotation2d deltaPositionHeading = new Rotation2d(deltaPosition, true);
			List<Pose2d> waypoints = new ArrayList<>();
			waypoints.add(new Pose2d(pose.getTranslation(), (getState() == ControlState.VISION) ? lastTrajectoryVector.direction() : deltaPositionHeading));
			waypoints.add(robotScoringPosition.get());
			Trajectory<TimedState<Pose2dWithCurvature>> trajectory = generator.generateTrajectory(false, waypoints, Arrays.asList(), 72.0, 72.0, 48.0, 9.0, 45.0, 1);	
			motionPlanner.reset();
			motionPlanner.setTrajectory(new TrajectoryIterator<>(new TimedView<>(trajectory)));
			rotationScalar = 0.25;
			//QuinticPathTransmitter.getInstance().addPath(trajectory);
			setPathHeading(robotScoringPosition.get().getRotation().getDegrees());
			visionTargetHeading = robotScoringPosition.get().getRotation();
			setState(ControlState.VISION);
			visionUpdateCount++;
			System.out.println("Vision trajectory updated " + visionUpdateCount + " times.");
		}else{
			DriverStation.reportError("Vision update refused!", false);
		}
	}

	/** Creates and sets a trajectory for the robot to follow, in order to approach a target and score a game piece */
	public synchronized void setVisionTrajectory(){
		Optional<ShooterAimingParameters> aim = robotState.getAimingParameters();
		if(aim.isPresent()){
			if(aim.get().getRange() > Constants.kDefaultCurveDistance){
				setCurvedVisionTrajectory(Constants.kDefaultCurveDistance, aim);
			}else{
				setCurvedVisionTrajectory(aim.get().getRange() / 2.0, aim);
			}
		}
	}
	
	/****************************************************/
	/* Vector Fields */
	public synchronized void setVectorField(VectorField vf_) {
		vf = vf_;
		setState(ControlState.VECTORIZED);
	}
	
	/** Determines which wheels the robot should rotate about in order to perform an evasive maneuver */
	public synchronized void determineEvasionWheels(){
		Translation2d here = lastDriveVector.rotateBy(pose.getRotation().inverse());
		List<Translation2d> wheels = Constants.kModulePositions;
		clockwiseCenter = wheels.get(0);
		counterClockwiseCenter = wheels.get(wheels.size()-1);
		for(int i = 0; i < wheels.size()-1; i++) {
			Translation2d cw = wheels.get(i);
			Translation2d ccw = wheels.get(i+1);
			if(here.isWithinAngle(cw,ccw)) {
				clockwiseCenter = ccw;
				counterClockwiseCenter = cw;
			}
		}
	}
	
	/** The tried and true algorithm for keeping track of position */
	public synchronized void updatePose(double timestamp){
		double x = 0.0;
		double y = 0.0;
		Rotation2d heading = pigeon.getYaw();
		
		double averageDistance = 0.0;
		double[] distances = new double[4];
		for(SwerveDriveModule m : positionModules){
			m.updatePose(heading);
			double distance = m.getEstimatedRobotPose().getTranslation().translateBy(pose.getTranslation().inverse()).norm();
			distances[m.moduleID] = distance;
			averageDistance += distance;
		}
		averageDistance /= positionModules.size();
		
		int minDevianceIndex = 0;
		double minDeviance = 100.0;
		List<SwerveDriveModule> modulesToUse = new ArrayList<>();
		for(SwerveDriveModule m : positionModules){
				double deviance = Math.abs(distances[m.moduleID] - averageDistance);
				if(deviance < minDeviance){
					minDeviance = deviance;
					minDevianceIndex = m.moduleID;
				}
				if(deviance <= 0.01){
					modulesToUse.add(m);
				}
			}
		
		if(modulesToUse.isEmpty()){
			modulesToUse.add(modules.get(minDevianceIndex));
		}
		
		//SmartDashboard.putNumber("Modules Used", modulesToUse.size());
		
		for(SwerveDriveModule m : modulesToUse){
			x += m.getEstimatedRobotPose().getTranslation().x();
			y += m.getEstimatedRobotPose().getTranslation().y();
		}
		Pose2d updatedPose = new Pose2d(new Translation2d(x / modulesToUse.size(), y / modulesToUse.size()), heading);
		double deltaPos = updatedPose.getTranslation().translateBy(pose.getTranslation().inverse()).norm();
		distanceTraveled += deltaPos;
		currentVelocity = deltaPos / (timestamp - lastUpdateTimestamp);
		pose = updatedPose;
		modules.forEach((m) -> m.resetPose(pose));
	}

	/** Playing around with different methods of odometry. This will require the use of all four modules, however. */
	public synchronized void alternatePoseUpdate(){
		double x = 0.0;
		double y = 0.0;
		Rotation2d heading = pigeon.getYaw();
		
		double[][] distances = new double[4][2];
		for(SwerveDriveModule m : modules){
			m.updatePose(heading);
			double distance = m.getEstimatedRobotPose().getTranslation().distance(pose.getTranslation());
			distances[m.moduleID][0] = m.moduleID;
			distances[m.moduleID][1] = distance;
		}
		
		Arrays.sort(distances, new java.util.Comparator<double[]>() {
			public int compare(double[] a, double[] b) {
				return Double.compare(a[1], b[1]);
			}
		});
		List<SwerveDriveModule> modulesToUse = new ArrayList<>();
		double firstDifference = distances[1][1] - distances[0][1];
		double secondDifference = distances[2][1] - distances[1][1];
		double thirdDifference = distances[3][1] - distances[2][1];
		if(secondDifference > (1.5 * firstDifference)){
			modulesToUse.add(modules.get((int)distances[0][0]));
			modulesToUse.add(modules.get((int)distances[1][0]));
		}else if(thirdDifference > (1.5 * firstDifference)){
			modulesToUse.add(modules.get((int)distances[0][0]));
			modulesToUse.add(modules.get((int)distances[1][0]));
			modulesToUse.add(modules.get((int)distances[2][0]));
		}else{
			modulesToUse.add(modules.get((int)distances[0][0]));
			modulesToUse.add(modules.get((int)distances[1][0]));
			modulesToUse.add(modules.get((int)distances[2][0]));
			modulesToUse.add(modules.get((int)distances[3][0]));
		}
		
		SmartDashboard.putNumber("Modules Used", modulesToUse.size());
		
		for(SwerveDriveModule m : modulesToUse){
			x += m.getEstimatedRobotPose().getTranslation().x();
			y += m.getEstimatedRobotPose().getTranslation().y();
		}

		Pose2d updatedPose = new Pose2d(new Translation2d(x / modulesToUse.size(), y / modulesToUse.size()), heading);
		double deltaPos = updatedPose.getTranslation().distance(pose.getTranslation());
		distanceTraveled += deltaPos;
		pose = updatedPose;
		modules.forEach((m) -> m.resetPose(pose));
	}

	/** Called every cycle to update the swerve based on its control state */
	public synchronized void updateControlCycle(double timestamp){
		double rotationCorrection = headingController.updateRotationCorrection(pose.getRotation().getUnboundedDegrees(), timestamp);

		switch(currentState){
		case MANUAL:
			if(evading && evadingToggled){
				determineEvasionWheels();
				double sign = Math.signum(rotationalInput);
				if(sign == 1.0){
					inverseKinematics.setCenterOfRotation(clockwiseCenter);
				}else if(sign == -1.0){
					inverseKinematics.setCenterOfRotation(counterClockwiseCenter);
				}
				evadingToggled = false;
			}else if(evading){
				double sign = Math.signum(rotationalInput);
				if(sign == 1.0){
					inverseKinematics.setCenterOfRotation(clockwiseCenter);
				}else if(sign == -1.0){
					inverseKinematics.setCenterOfRotation(counterClockwiseCenter);
				}
			}else if(evadingToggled){
				inverseKinematics.setCenterOfRotation(Translation2d.identity());
				evadingToggled = false;
			}
			if(translationalVector.equals(Translation2d.identity()) && rotationalInput == 0.0){
				if(lastDriveVector.equals(rotationalVector)){
					stop();
				}else{
					setDriveOutput(inverseKinematics.updateDriveVectors(lastDriveVector,
					rotationCorrection, pose, robotCentric), 0.0);
				}
			}else{
				setDriveOutput(inverseKinematics.updateDriveVectors(translationalVector,
						rotationalInput + rotationCorrection, pose, robotCentric));
			}
			break;
		case POSITION:
			if(positionOnTarget())
				rotate(headingController.getTargetHeading());
			break;
		case ROTATION:
			setDriveOutput(inverseKinematics.updateDriveVectors(new Translation2d(), Util.deadBand(rotationCorrection, 0.1), pose, false));
			break;
		case VECTORIZED:
			Translation2d outputVectorV = vf.getVector(pose.getTranslation()).scale(0.25);
			SmartDashboard.putNumber("Vector Direction", outputVectorV.direction().getDegrees());
			SmartDashboard.putNumber("Vector Magnitude", outputVectorV.norm());
//			System.out.println(outputVector.x()+" "+outputVector.y());
			setDriveOutput(inverseKinematics.updateDriveVectors(outputVectorV, rotationCorrection, getPose(), false));
			break;
		case TRAJECTORY:
			if(!motionPlanner.isDone()){
				Translation2d driveVector = motionPlanner.update(timestamp, pose);

				if(modulesReady){
					if(!hasStartedFollowing){
						if(moduleConfigRequested){
							zeroSensors(Constants.kRobotLeftStartingPose);
							System.out.println("Position reset for auto");
						}
						hasStartedFollowing = true;
					}
					double rotationInput = Util.deadBand(Util.limit(rotationCorrection*rotationScalar*driveVector.norm(), motionPlanner.getMaxRotationSpeed()), 0.01);
					if(Util.epsilonEquals(driveVector.norm(), 0.0, Constants.kEpsilon)){
						driveVector = lastTrajectoryVector;
						setVelocityDriveOutput(inverseKinematics.updateDriveVectors(driveVector, 
							rotationInput, pose, false), 0.0);
					}else{
						setVelocityDriveOutput(inverseKinematics.updateDriveVectors(driveVector, 
						rotationInput, pose, false));
					}
				}else if(!moduleConfigRequested){
					set10VoltRotationMode(true);
					setModuleAngles(inverseKinematics.updateDriveVectors(driveVector, 
						0.0, pose, false));
					moduleConfigRequested = true;
				}

				if(moduleAnglesOnTarget() && !modulesReady){
					set10VoltRotationMode(false);
					modules.forEach((m) -> m.resetLastEncoderReading());
					modulesReady = true;
					System.out.println("Modules Ready");
				}
				
				lastTrajectoryVector = driveVector;
			}else{
				if(!hasFinishedPath){ 
					System.out.println("Path completed in: " + (timestamp - trajectoryStartTime));
					hasFinishedPath = true;
					if(alwaysConfigureModules) requireModuleConfiguration();
				}
			}
			break;
		case VISION:
			if(!motionPlanner.isDone()){
				/*if((!robotState.seesTarget() || robotState.getCaptureTimeFieldToGoal().size() < robotState.minimumTargetQuantity) && visionUpdatesAllowed){
					visionUpdatesAllowed = false;
				}*/
				Optional<ShooterAimingParameters> aim = robotState.getAimingParameters();
				if(aim.isPresent() && visionUpdatesAllowed){
					if(aim.get().getRange() < (Constants.kVisionUpdateDistance - (Constants.kVisionDistanceStep * visionCriteria.successfulUpdates(VisionCriteria.Criterion.DISTANCE))) 
						&& visionCriteria.updateAllowed(VisionCriteria.Criterion.DISTANCE) && aim.get().getRange() >= Constants.kClosestVisionDistance){
						setVisionTrajectory();
						visionCriteria.addSuccessfulUpdate(VisionCriteria.Criterion.DISTANCE);
					}else if(Math.abs(pose.getRotation().distance(visionTargetHeading)) < Math.toRadians(2.0) && visionCriteria.updateAllowed(VisionCriteria.Criterion.HEADING)){
						setVisionTrajectory();
						visionCriteria.addSuccessfulUpdate(VisionCriteria.Criterion.HEADING);
					}
					/*if(aim.get().getRange() <= visionCurveDistance){
						Optional<Pose2d> robotScoringPosition = robotState.getRobotScoringPosition(aim);
						setPathHeading(robotScoringPosition.get().getRotation().getDegrees());
						visionTargetHeading = robotScoringPosition.get().getRotation();
					}*/
				}
				Translation2d driveVector = motionPlanner.update(timestamp, pose);
				if(Util.epsilonEquals(driveVector.norm(), 0.0, Constants.kEpsilon)){
					driveVector = lastTrajectoryVector;
					setVelocityDriveOutput(inverseKinematics.updateDriveVectors(driveVector, 
						rotationCorrection*rotationScalar, pose, false), 0.0);
				}else{
					setVelocityDriveOutput(inverseKinematics.updateDriveVectors(driveVector, 
						rotationCorrection*rotationScalar, pose, false));
				}
				lastTrajectoryVector = driveVector;
			}
			break;
		case VELOCITY:

			break;
		case NEUTRAL:
			stop();
			break;
		case DISABLED:
			
			break;
		default:
			break;
		}
	}
	
	private final Loop loop = new Loop(){

		@Override
		public void onStart(double timestamp) {
			synchronized(Swerve.this){
				translationalVector = new Translation2d();
				lastDriveVector = rotationalVector;
				rotationalInput = 0;
				resetAveragedDirection();
				headingController.temporarilyDisable();
				stop();
				lastUpdateTimestamp = timestamp;
			}
		}

		@Override
		public void onLoop(double timestamp) {
			synchronized(Swerve.this){
				if(modulesReady || (getState() != ControlState.TRAJECTORY)){
					//updatePose(timestamp);
					alternatePoseUpdate();
				}
				updateControlCycle(timestamp);
				lastUpdateTimestamp = timestamp;
			}
		}

		@Override
		public void onStop(double timestamp) {
			synchronized(Swerve.this){
				translationalVector = new Translation2d();
				rotationalInput = 0;
				stop();
			}
		}
		
	};
	
	public void setNominalDriveOutput(double voltage){
		modules.forEach((m) -> m.setNominalDriveOutput(voltage));
	}
	
	/** Sets the maximum rotation speed opf the modules, based on the robot's velocity */
	public void setMaxRotationSpeed(){
		double currentDriveSpeed = translationalVector.norm() * Constants.kSwerveMaxSpeedInchesPerSecond;
		double newMaxRotationSpeed = Constants.kSwerveRotationMaxSpeed / 
				((Constants.kSwerveRotationSpeedScalar * currentDriveSpeed) + 1.0);
		modules.forEach((m) -> m.setMaxRotationSpeed(newMaxRotationSpeed));
	}

	@Override
	public synchronized void readPeriodicInputs() {
		modules.forEach((m) -> m.readPeriodicInputs());
	}

	@Override
	public synchronized void writePeriodicOutputs() {
		modules.forEach((m) -> m.writePeriodicOutputs());
	}
	
	@Override
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(loop);
	}
	
	/** Puts all rotation and drive motors into open-loop mode */
	public synchronized void disable(){
		modules.forEach((m) -> m.disable());
		setState(ControlState.DISABLED);
	}

	@Override
	public synchronized void stop() {
		setState(ControlState.NEUTRAL);
		modules.forEach((m) -> m.stop());
	}

	@Override
	public synchronized void zeroSensors() {
		zeroSensors(Constants.kRobotLeftStartingPose);
	}
	
	/** Zeroes the drive motors, and sets the robot's internal position and heading to match that of the fed pose */
	public synchronized void zeroSensors(Pose2d startingPose){
		pigeon.setAngle(startingPose.getRotation().getUnboundedDegrees());
		modules.forEach((m) -> m.zeroSensors(startingPose));
		pose = startingPose;
		distanceTraveled = 0;
	}
	
	public synchronized void resetPosition(Pose2d newPose){
		pose = new Pose2d(newPose.getTranslation(), pose.getRotation());
		modules.forEach((m) -> m.zeroSensors(pose));
		distanceTraveled = 0;
	}
	
	public synchronized void setXCoordinate(double x){
		pose.getTranslation().setX(x);
		modules.forEach((m) -> m.zeroSensors(pose));
		System.out.println("X coordinate reset to: " + pose.getTranslation().x());
	}
	
	public synchronized void setYCoordinate(double y){
		pose.getTranslation().setY(y);
		modules.forEach((m) -> m.zeroSensors(pose));
	}

	@Override
	public void outputTelemetry() {
		modules.forEach((m) -> m.outputTelemetry());
		if(Constants.kDebuggingOutput){
			SmartDashboard.putNumber("Robot X", pose.getTranslation().x());
			SmartDashboard.putNumber("Robot Y", pose.getTranslation().y());
			SmartDashboard.putNumberArray("Robot Pose", new double[]{pose.getTranslation().x(), pose.getTranslation().y(), pose.getRotation().getUnboundedDegrees()});
			SmartDashboard.putNumber("Robot Heading", pose.getRotation().getUnboundedDegrees());
			SmartDashboard.putString("Heading Controller", headingController.getState().toString());
			SmartDashboard.putNumber("Target Heading", headingController.getTargetHeading());
			SmartDashboard.putNumber("Distance Traveled", distanceTraveled);
			SmartDashboard.putNumber("Robot Velocity", currentVelocity);
			SmartDashboard.putString("Swerve State", currentState.toString());
			SmartDashboard.putBoolean("Vision Updates Allowed", visionUpdatesAllowed);
		}
	}
}