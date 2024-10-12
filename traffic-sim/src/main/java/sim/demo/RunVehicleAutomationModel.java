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
	
    @Option(names = "-simTime", description = "Simulation time.", defaultValue = "1800.0")
    private double simTime;

    @Option(names = "-level0Fraction", description = "Fraction of level 0 vehicles.", defaultValue = "0.25")
    private double level0Fraction;
    
    @Option(names = "-level1Fraction", description = "Fraction of level 1 vehicles.", defaultValue = "0.25")
    private double level1Fraction;
    
    @Option(names = "-level2Fraction", description = "Fraction of level 2 vehicles.", defaultValue = "0.25")
    private double level2Fraction;
    
    @Option(names = "-level3Fraction", description = "Fraction of level 3 vehicles.", defaultValue = "0.25")
    private double level3Fraction;
    
    @Option(names = "-leftFraction", description = "Left traffic fraction.", defaultValue = "0.8")
    private double leftFraction;
    
    @Option(names = "-mainDemand", description = "Traffic demand on main road.", defaultValue = "5027.217824437574") // 2000
    private double mainDemand;
    
    @Option(names = "-rampDemand", description = "Traffic demand for on-ramp.", defaultValue = "731.3108993050043") // 500
    private double rampDemand;
    
    @Option(names = "-additionalIncentives", description = "Enable additional lane change and speed incentives.", negatable=false, defaultValue = "true")
    private boolean additionalIncentives;
    
    
    
    @Option(names = "-inputValuesFilePath", description = "File location for simulation output storage.", 
    		defaultValue = "C:\\Users\\jesse\\Documents\\Java\\TrafficSimulation-workspace\\traffic-sim\\"
    					   + "src\\main\\resources\\inputValues.csv")
    private String inputValuesFilePath;
    
    @Option(names = "-singleOutputFilePath", description = "File location for simulation output storage.", 
    		defaultValue = "C:\\Users\\jesse\\Documents\\Java\\TrafficSimulation-workspace\\traffic-sim\\"
    					   + "src\\main\\resources\\singleOutputData.csv")
    private String singleOutputFilePath;
    
    @Option(names = "-intermediateMeanValuesFilePath", description = "File location for simulation output storage.", 
    		defaultValue = "C:\\Users\\jesse\\Documents\\Java\\TrafficSimulation-workspace\\traffic-sim\\"
    					   + "src\\main\\resources\\intermediateOutputData.csv")
    private String intermediateMeanValuesFilePath;
    
    @Option(names = "-sequenceOutputFilePath", description = "File location for simulation output storage.", 
    		defaultValue = "C:\\Users\\jesse\\Documents\\Java\\TrafficSimulation-workspace\\traffic-sim\\"
    					   + "src\\main\\resources\\sequenceOutputData.csv")
    private String sequenceOutputFilePath;
    
    @Option(names = "-laneChangeOutputFilePath", description = "File location for simulation output storage.", 
    		defaultValue = "C:\\Users\\jesse\\Documents\\Java\\TrafficSimulation-workspace\\traffic-sim\\"
    					   + "src\\main\\resources\\laneChangeOutputData.csv")
    private String laneChangeOutputFilePath;
    
    /**
     * Runnable method that will be executed by picocli from main.
     */
    @Override
    public void run() {
    	// bundle input parameters
        VehicleAutomationModelParameters simConfig = createConfig();
        
        // check headless mode and start simulation
        if (simConfig.getHeadless()) {
        	VehicleAutomationHeadless.start(simConfig);
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
        return new VehicleAutomationModelParameters(headless, seed, simTime,
        											level0Fraction, level1Fraction, level2Fraction, level3Fraction,
        											leftFraction, mainDemand, rampDemand,
        											additionalIncentives,
        											inputValuesFilePath, singleOutputFilePath, intermediateMeanValuesFilePath,
        											sequenceOutputFilePath, laneChangeOutputFilePath);
    }
    
}
