package sim.demo;

import javax.naming.NamingException;

import org.djunits.value.vdouble.scalar.Duration;
import org.djunits.value.vdouble.scalar.Time;
import org.opentrafficsim.core.dsol.OtsAnimator;
import org.opentrafficsim.draw.OtsDrawingException;
import org.opentrafficsim.swing.gui.OtsAnimationPanel;
import org.opentrafficsim.swing.gui.OtsSimulationApplication;

import nl.tudelft.simulation.dsol.SimRuntimeException;


public class VehicleAutomationHeadlessApplication extends OtsSimulationApplication<VehicleAutomationModel> {
	
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

	/**
     * Constructor.
     * @param model model
     * @param panel panel
     * @throws OtsDrawingException on animation error
     */
    private VehicleAutomationHeadlessApplication(final VehicleAutomationModel model, final OtsAnimationPanel panel) throws OtsDrawingException
    {
        super(model, panel);
    }
    
    /**
     * Start method to invoke the simulation.
     * 
     * @param simConfig; containing input parameters
     */
    public static void start(VehicleAutomationModelParameters simConfig)
    {
    	main(simConfig);
    }

	/**
     * Main program.
     * @param simConfig
     */
    public static void main(VehicleAutomationModelParameters simConfig)
    {
        try
        {
            OtsAnimator simulator = new OtsAnimator("VehicleAutomationSimulation");
            final VehicleAutomationModel otsModel = new VehicleAutomationModel(simulator, simConfig);
            double simulationTime = simConfig.getWarmUpTime() + simConfig.getSampleTime();
            simulator.initialize(Time.ZERO, Duration.ZERO, Duration.instantiateSI(simulationTime), otsModel);
            simulator.setSpeedFactor(1000.0);
            System.out.println("Run headless simulation with a speed factor of " + simulator.getSpeedFactor() + ".\n");
            simulator.start();
        }
        catch (SimRuntimeException | NamingException exception)
        {
            exception.printStackTrace();
        }
    }
    
}
