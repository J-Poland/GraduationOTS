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
    private double level0Fraction;
    private double level1Fraction;
    private double level2Fraction;
    private double level3Fraction;
    private double leftFraction;
    private double mainDemand;
    private double rampDemand;
    private boolean additionalIncentives;
    private String outputFolderPath;
    private String inputValuesFileName;
    private String singleOutputFileName;
    private String intermediateMeanValuesFileName;
    private String sequenceOutputFileName;
    private String getlaneChangeOutputFileName;

    // define simulation parameters in constructor
    public VehicleAutomationModelParameters(boolean headless, long seed, double simTime, double level0Fraction, 
    										double level1Fraction, double level2Fraction, double level3Fraction, 
    										double leftFraction, double mainDemand, double rampDemand, 
    										boolean additionalIncentives,
    										String outputFolderPath, String inputValuesFilePath,
    										String singleOutputFileName, String intermediateMeanValuesFileName,
    										String sequenceOutputFileName, String getlaneChangeOutputFileName) {
    	this.headless = headless;
    	this.seed = seed;
        this.simTime = simTime;
        this.level0Fraction = level0Fraction;
        this.level1Fraction = level1Fraction;
        this.level2Fraction = level2Fraction;
        this.level3Fraction = level3Fraction;
        this.leftFraction = leftFraction;
        this.mainDemand = mainDemand;
        this.rampDemand = rampDemand;
        this.additionalIncentives = additionalIncentives;
        this.outputFolderPath = outputFolderPath;
        this.inputValuesFileName = inputValuesFilePath;
        this.singleOutputFileName = singleOutputFileName;
        this.intermediateMeanValuesFileName = intermediateMeanValuesFileName;
        this.sequenceOutputFileName = sequenceOutputFileName;
        this.getlaneChangeOutputFileName = getlaneChangeOutputFileName;
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

    public double getLevel0Fraction() {
        return level0Fraction;
    }
    public void setLevel0Fraction(double level0Fraction) {
        this.level0Fraction = level0Fraction;
    }
    
    public double getLevel1Fraction() {
        return level1Fraction;
    }
    public void setLevel1Fraction(double level1Fraction) {
        this.level1Fraction = level1Fraction;
    }
    
    public double getLevel2Fraction() {
        return level2Fraction;
    }
    public void setLevel2Fraction(double level2Fraction) {
        this.level2Fraction = level2Fraction;
    }
    
    public double getLevel3Fraction() {
        return level3Fraction;
    }
    public void setLevel3Fraction(double level3Fraction) {
        this.level3Fraction = level3Fraction;
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
    
    
    // getter and setters for file locations
    public String getOutputFolderPath() {
        return outputFolderPath;
    }
    public void setOutputFolderPath(String outputFolderPath) {
        this.outputFolderPath = outputFolderPath;
    }
    
    public String getInputValuesFileName() {
        return inputValuesFileName;
    }
    public void setInputValuesFilePath(String inputValuesFileName) {
        this.inputValuesFileName = inputValuesFileName;
    }
    
    public String getSingleOutputFileName() {
        return singleOutputFileName;
    }
    public void setSingleOutPutFileName(String singleOutputFileName) {
        this.singleOutputFileName = singleOutputFileName;
    }
    
    public String getIntermediateMeanValuesFileName() {
        return intermediateMeanValuesFileName;
    }
    public void setIntermediateMeanValuesFileName(String intermediateMeanValuesFileName) {
        this.intermediateMeanValuesFileName = intermediateMeanValuesFileName;
    }
    
    public String getSequenceOutputFileName() {
        return sequenceOutputFileName;
    }
    public void setSequenceOutPutFileName(String sequenceOutputFileName) {
        this.sequenceOutputFileName = sequenceOutputFileName;
    }
    
    public String getlaneChangeOutputFileName() {
        return getlaneChangeOutputFileName;
    }
    public void setlaneChangeOutputFileName(String getlaneChangeOutputFileName) {
        this.getlaneChangeOutputFileName = getlaneChangeOutputFileName;
    }

}
