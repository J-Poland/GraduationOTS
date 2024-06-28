package sim.demo;

/**
 * Class to bundle input parameters for convenient communication to the actual model.
 */
public class VehicleAutomationModelParameters
{
	
	// simulation parameters
	private boolean headless;
	private long seed;
	private double simTime;
    private double avFraction;
    private double leftFraction;
    private double mainDemand;
    private double rampDemand;
    private boolean additionalIncentives;
    private double tMin;
    private double tMax;
    private String singleOutputFilePath;
    private String sequenceOutputFilePath;

    // define simulation parameters in constructor
    public VehicleAutomationModelParameters(boolean headless, long seed, double simTime, double avFraction, double leftFraction,
    						  double mainDemand, double rampDemand, boolean additionalIncentives, double tMin,
    						  double tMax, String singleOutputFilePath, String sequenceOutputFilePath) {
    	this.headless = headless;
    	this.seed = seed;
        this.simTime = simTime;
        this.avFraction = avFraction;
        this.leftFraction = leftFraction;
        this.mainDemand = mainDemand;
        this.rampDemand = rampDemand;
        this.additionalIncentives = additionalIncentives;
        this.tMin = tMin;
        this.tMax = tMax;
        this.singleOutputFilePath = singleOutputFilePath;
        this.sequenceOutputFilePath = sequenceOutputFilePath;
    }
    
    // create getters and setters for the available parameters
    public boolean getHeadless() {
        return headless;
    }
    public void setHeadless(boolean headless) {
        this.headless = headless;
    }
    
    public long getSeed() {
        return seed;
    }
    public void setSeed(long seed) {
        this.seed = seed;
    }
    
    public double getSimTime() {
        return simTime;
    }
    public void setSimTime(double simTime) {
        this.simTime = simTime;
    }

    public double getAvFraction() {
        return avFraction;
    }
    public void setAvFraction(double avFraction) {
        this.avFraction = avFraction;
    }
    
    public double getLeftFraction() {
        return leftFraction;
    }
    public void setLeftFraction(double leftFraction) {
        this.leftFraction = leftFraction;
    }
    
    public double getMainDemand() {
        return mainDemand;
    }
    public void setMainDemand(double mainDemand) {
        this.mainDemand = mainDemand;
    }
    
    public double getRampDemand() {
        return rampDemand;
    }
    public void setRampDemand(double rampDemand) {
        this.rampDemand = rampDemand;
    }
    
    public boolean getAdditionalIncentives() {
        return additionalIncentives;
    }
    public void setAdditionalIncentives(boolean additionalIncentives) {
        this.additionalIncentives = additionalIncentives;
    }
    
    public double getTMin() {
        return tMin;
    }
    public void setTMin(double tMin) {
        this.tMin = tMin;
    }
    
    public double getTMax() {
        return tMax;
    }
    public void setTMax(double tMax) {
        this.tMax = tMax;
    }
    
    public String getSingleOutputFilePath() {
        return singleOutputFilePath;
    }
    public void setSingleOutPutFilePath(String singleOutputFilePath) {
        this.singleOutputFilePath = singleOutputFilePath;
    }
    
    public String getSequenceOutputFilePath() {
        return sequenceOutputFilePath;
    }
    public void setSequenceOutPutFilePath(String sequenceOutputFilePath) {
        this.sequenceOutputFilePath = sequenceOutputFilePath;
    }

}
