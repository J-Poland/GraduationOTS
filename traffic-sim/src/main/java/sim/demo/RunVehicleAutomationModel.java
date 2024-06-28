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
	
    @Option(names = "-simTime", description = "Simulation time.", defaultValue = "600.0")
    private double simTime;

    @Option(names = "-avFraction", description = "Fraction of AVs.", defaultValue = "0.15")
    private double avFraction;
    
    @Option(names = "-leftFraction", description = "Left traffic fraction.", defaultValue = "0.8")
    private double leftFraction;
    
    @Option(names = "-mainDemand", description = "Left traffic fraction.", defaultValue = "1000")
    private double mainDemand;
    
    @Option(names = "-rampDemand", description = "Left traffic fraction.", defaultValue = "500")
    private double rampDemand;
    
    @Option(names = "-additionalIncentives", description = "...", negatable=false, defaultValue = "true")
    private boolean additionalIncentives;
    
    @Option(names = "-tMin", description = "...", negatable=false, defaultValue = "0.56")
    private double tMin;
    
    @Option(names = "-tMax", description = "...", negatable=false, defaultValue = "1.2")
    private double tMax;
    
    @Option(names = "-singleOutputFilePath", description = "File location for simulation output storage.", 
    		defaultValue = "C:\\Users\\jesse\\Documents\\Java\\TrafficSimulation-workspace\\traffic-sim\\"
    					   + "src\\main\\resources\\singleOutputData.csv")
    private String singleOutputFilePath;
    
    @Option(names = "-sequenceOutputFilePath", description = "File location for simulation output storage.", 
    		defaultValue = "C:\\Users\\jesse\\Documents\\Java\\TrafficSimulation-workspace\\traffic-sim\\"
    					   + "src\\main\\resources\\sequenceOutputData.csv")
    private String sequenceOutputFilePath;
    
    
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
        return new VehicleAutomationModelParameters(headless, seed, simTime, avFraction, leftFraction, mainDemand, rampDemand,
        							  additionalIncentives, tMin, tMax, singleOutputFilePath, sequenceOutputFilePath);
    }
    
}
