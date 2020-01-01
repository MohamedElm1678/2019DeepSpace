package com.team1323.frc2019.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.team1323.frc2019.Constants;
import com.team1323.frc2019.RobotState;
import com.team1323.frc2019.loops.ILooper;
import com.team1323.frc2019.loops.LimelightProcessor;
import com.team1323.frc2019.loops.Loop;
import com.team1323.frc2019.subsystems.Swerve.VisionState;
import com.team1323.frc2019.subsystems.requests.LambdaRequest;
import com.team1323.frc2019.subsystems.requests.ParallelRequest;
import com.team1323.frc2019.subsystems.requests.Request;
import com.team1323.frc2019.subsystems.requests.SequentialRequest;
import com.team1323.lib.util.InterpolatingDouble;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Superstructure extends Subsystem {

	private Compressor compressor;
	
	public Swerve swerve;

	private LimelightProcessor limelight;

	private RobotState robotState;

	private boolean isClimbing = false;
	public boolean isClimbing(){ return isClimbing; }
	public void enableInterpolator(boolean enable){ 
		isClimbing = enable;
		swerve.setLowPowerScalar(0.25);
	}
	public void stopClimbing(){ 
		isClimbing = false; 
		swerve.setLowPowerScalar(0.6);
	}
	
	public Superstructure(){		
		compressor = new Compressor(20);
		
		swerve = Swerve.getInstance();

		limelight = LimelightProcessor.getInstance();

		robotState = RobotState.getInstance();
		
		queuedRequests = new ArrayList<>(0);
	}
	private static Superstructure instance = null;
	public static Superstructure getInstance(){
		if(instance == null)
			instance = new Superstructure();
		return instance;
	}

	private Request activeRequest = null;
	private List<Request> queuedRequests = new ArrayList<>();
	
	private boolean newRequest = false;
	private boolean allRequestsCompleted = false;
	public boolean requestsCompleted(){ return allRequestsCompleted; }
	
	private void setActiveRequest(Request request){
		activeRequest = request;
		newRequest = true;
		allRequestsCompleted = false;
	}
	
	private void setQueue(List<Request> requests){
		clearQueue();
		for(Request request : requests) {
			queuedRequests.add(request);
		}
	}

	private void setQueue(Request request) {
		setQueue(Arrays.asList(request));
	}

	private void clearQueue() {
		queuedRequests.clear();
	}
	
	public void request(Request r){
		setActiveRequest(r);
		clearQueue();
	}
	
	public void request(Request active, Request queue){
		setActiveRequest(active);
		setQueue(queue);
	}
	
	public void queue(Request request){
		queuedRequests.add(request);
	}
	
	public void replaceQueue(Request request){
		setQueue(request);
	}
	
	private final Loop loop = new Loop(){

		@Override
		public void onStart(double timestamp) {
			stop();
		}

		@Override
		public void onLoop(double timestamp) {
			synchronized(Superstructure.this){

				if(newRequest) {
					activeRequest.act();
					newRequest = false;
				} 

				if(activeRequest == null) {
					if(queuedRequests.isEmpty()) {
						allRequestsCompleted = true;
					} else {
						setActiveRequest(queuedRequests.remove(0));
					}
				} else if(activeRequest.isFinished()) {
					activeRequest = null;
				}
			
			}
		}

		@Override
		public void onStop(double timestamp) {
			disabledState();
		}
		
	};
	


	
	
	public void enableCompressor(boolean enable){
		compressor.setClosedLoopControl(enable);
	}

	@Override
	public void stop() {
	}

	@Override
	public void zeroSensors() {
		
	}

	@Override
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(loop);
	}

	@Override
	public void outputTelemetry() {
	}

	public Request waitRequest(double seconds){
		return new Request(){
			double startTime = 0.0;
			double waitTime = 1.0;
		
			@Override
			public void act() {
				startTime = Timer.getFPGATimestamp();
				waitTime = seconds;
			}

			@Override
			public boolean isFinished(){
				return (Timer.getFPGATimestamp() - startTime) > waitTime;
			}
		};
	}

	public Request waitForVisionRequest(){
		return new Request(){

			@Override
			public void act() {

			}

			@Override
			public boolean isFinished(){
				return robotState.seesTarget();
			}

		};
	}

	/////States/////

	public void disabledState(){
		
	}

	public void neutralState(){
		request(new ParallelRequest(
			
		));
	}

	public void ballIntakingState(){


	}

	public void ballHoldingState(){
		
	}

	public void ballFeedingState(){
		
	}

	public void fullBallCycleState(){
		
	}

	public void ballScoringState(double elevatorHeight){
		
	}

	/**
	 * Uses old tracking tech
	 */
	public void ballTrackingState(double elevatorHeight){
		
	}

	public void diskIntakingState(){
		
	}

	public void diskReceivingState(){
		
	}

	public void diskScoringState(){
		
	}

	public void diskScoringState(double elevatorHeight, boolean resuck){
		
	}

	/** Used for driver tracking */
	public void diskTrackingState(){
		
	}

	/** Used for codriver tracking */
	public void diskTrackingState(double elevatorHeight){
		
	}

	/**
	 * Old Tracking Tech
	 */
	public void diskTrackingState(double elevatorHeight, Rotation2d fixedOrientation){
		
	}

	/**
	 * Old Tracking tech
	 */
	public void diskTrackingState(double elevatorHeight, Rotation2d fixedOrientation, double trackingSpeed){


	/**
	 * Old tracking tech
	 */
	public void diskTrackingState(double elevatorHeight, Rotation2d fixedOrientation, double cutoffDistance, Translation2d endTranslation, double trackingSpeed){
		
	}

	/**
	 * Uses old tracking tech
	 */
	public void humanLoaderTrackingState(){

	}

	/**
	 * Uses new tracking tech
	 */
	public void humanLoaderRetrievingState(){
		robotState.clearVisionTargets();
		request(new SequentialRequest(
			
			swerve.visionPIDRequest(new Translation2d(4.0, 0.0), Rotation2d.fromDegrees(180.0), 66.0), //TODO change this rotation back to 180 for normal driver practice
			swerve.trajectoryRequest(new Translation2d(-60.0, 0.0), 180.0, 60.0),
			swerve.openLoopRequest(new Translation2d(), 0.0)
		));
	}

	//Testing new vision system
	public void visionPIDState() {

	}

	public void shortClimbingState(){

	}

	public void climbingState(){
		
	}

	public void postClimbingState(){
		isClimbing = false;
		
	}

	public void lockedJackState(){
		
	}

	public void jackState(double jackHeight){
		
	}

}
