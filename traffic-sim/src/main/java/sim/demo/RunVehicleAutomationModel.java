package sim.demo;

import picocli.CommandLine;
import picocli.CommandLine.Option;

/**
 * Class to enable command line simulation input parameters by using picocli.
 */
public class RunVehicleAutomationModel implements Runnable
{

	/**
	 *  Create picocli @Option input variables
	 */
	@Option(names = "-headless", description = "Run without animations.", negatable=true, defaultValue = "false")
    private boolean headless;
	
	@Option(names = "-seed", description = "Set simulation seed.", defaultValue = "0")
    private long seed;
	
    @Option(names = "-warmUpTime", description = "Simulation warm-up time.", defaultValue = "200.0")
    private double warmUpTime;
	
    @Option(names = "-sampleTime", description = "Simulation time for sampling data.", defaultValue = "1000.0")
    private double sampleTime;

    @Option(names = "-level0Fraction", description = "Fraction of level 0 vehicles.", defaultValue = "0.25")
    private double level0Fraction;
    
    @Option(names = "-level1Fraction", description = "Fraction of level 1 vehicles.", defaultValue = "0.25")
    private double level1Fraction;
    
    @Option(names = "-level2Fraction", description = "Fraction of level 2 vehicles.", defaultValue = "0.25")
    private double level2Fraction;
    
    @Option(names = "-level3Fraction", description = "Fraction of level 3 vehicles.", defaultValue = "0.25")
    private double level3Fraction;
    
    @Option(names = "-mainDemand", description = "Traffic demand on main road.", defaultValue = "1000") // 2000
    private double mainDemand;
    
    @Option(names = "-rampDemand", description = "Traffic demand for on-ramp.", defaultValue = "400") // 400
    private double rampDemand;
    
    @Option(names = "-inVehicleDistraction", description = "Enable in-vehicle distraction from secondary tasks.", negatable=false, defaultValue = "true")
    private boolean inVehicleDistraction;
    
    @Option(names = "-roadSideDistraction", description = "Enable distraction at the side of the road.", negatable=false, defaultValue = "false")
    private boolean roadSideDistraction;
    
    @Option(names = "-sensitivityAnalysisValue", description = "Varying parameter value for sensitivity analysis.", defaultValue = "0")
    private double sensitivityAnalysisValue;
    
    
    @Option(names = "-outputFolderPath", description = "Folder location for simulation output storage.", 
    		defaultValue = "C:\\Users\\jesse\\Documents\\Java\\TrafficSimulation-workspace\\traffic-sim\\"
    					   + "src\\main\\resources\\output")
    private String outputFolderPath;
    
    @Option(names = "-inputValuesFileName", description = "File name for input values.", 
    		defaultValue = "inputValues.csv")
    private String inputValuesFileName;
    
    @Option(names = "-singleOutputFileName", description = "File name for single value data.", 
    		defaultValue = "singleOutputData.csv")
    private String singleOutputFileName;
    
    @Option(names = "-intermediateMeanValuesFileName", description = "File name to store intermediate values of calculated mean values.", 
    		defaultValue = "intermediateOutputData.csv")
    private String intermediateMeanValuesFileName;
    
    @Option(names = "-sequenceOutputFileName", description = "File location for simulation output storage.", 
    		defaultValue = "sequenceOutputData.csv")
    private String sequenceOutputFileName;
    
    @Option(names = "-laneChangeOutputFileName", description = "File location for simulation output storage.", 
    		defaultValue = "laneChangeOutputData.csv")
    private String laneChangeOutputFileName;
    
    @Option(names = "-collisionOutputFileName", description = "File location for simulation output storage.", 
    		defaultValue = "collisionOutputData.csv")
    private String collisionOutputFileName;
    
    /**
     * Runnable method that will be executed by picocli from main.
     */
    @Override
    public void run() {
    	// bundle input parameters
        VehicleAutomationModelParameters simConfig = createConfig();
        
        // check headless mode and start simulation
        if (simConfig.getHeadless()) {
        	VehicleAutomationHeadlessApplication.start(simConfig);
        }
        else {
        	VehicleAutomationApplication.start(simConfig);
        }
    }

    /**
     * Instance of this class and load command line from picocli.
     * 
     * @param args
     */
    public static void main(final String[] args) {
    	RunVehicleAutomationModel parser = new RunVehicleAutomationModel();
        CommandLine cmd = new CommandLine(parser);
        cmd.execute(args);
    }
    
    // method to bundle the input variables into one configuration object
    public VehicleAutomationModelParameters createConfig() {
        return new VehicleAutomationModelParameters(headless, seed, warmUpTime, sampleTime,
        											level0Fraction, level1Fraction, level2Fraction, level3Fraction,
        											mainDemand, rampDemand, inVehicleDistraction, roadSideDistraction,
        											sensitivityAnalysisValue,
        											outputFolderPath, inputValuesFileName, singleOutputFileName, intermediateMeanValuesFileName, 
        											sequenceOutputFileName, laneChangeOutputFileName, collisionOutputFileName);
    }
    
}
