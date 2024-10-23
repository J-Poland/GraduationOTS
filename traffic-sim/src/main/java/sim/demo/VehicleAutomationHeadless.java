package sim.demo;

import javax.naming.NamingException;

import org.djunits.value.vdouble.scalar.Duration;
import org.djunits.value.vdouble.scalar.Time;
import org.opentrafficsim.core.dsol.AbstractOtsSimulationApplication;
import org.opentrafficsim.core.dsol.OtsSimulator;
import org.opentrafficsim.draw.OtsDrawingException;

import nl.tudelft.simulation.dsol.SimRuntimeException;


/**
 * <p>
 * Copyright (c) 2013-2024 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved. <br>
 * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
 * </p>
 * @author <a href="https://github.com/averbraeck">Alexander Verbraeck</a>
 * @author <a href="https://tudelft.nl/staff/p.knoppers-1">Peter Knoppers</a>
 * @author <a href="https://github.com/wjschakel">Wouter Schakel</a>
 */
public class VehicleAutomationHeadless extends AbstractOtsSimulationApplication
{
    /** */
	private static final long serialVersionUID = 1L;

	/**
     * Create a ShortMerge Simulation.
     * @param model ShortMergeModel; the model
     * @throws OtsDrawingException on animation error
     */
    public VehicleAutomationHeadless(final VehicleAutomationModel model) throws OtsDrawingException
    {
        super(model);
    }

    /**
     * Start method to invoke the simulation.
     * 
     * @param simConfig; containing input parameters
     */
    public static void start(VehicleAutomationModelParameters simConfig)
    {
    	demo(simConfig);
    }

    /**
     * Start the demo.
     * 
     * @param exitOnClose boolean; when running stand-alone: true; when running as part of a demo: false
     */
    public static void demo(VehicleAutomationModelParameters simConfig)
    {
        try
        {
        	OtsSimulator simulator = new OtsSimulator("VehicleAutomation");
            final VehicleAutomationModel otsModel = new VehicleAutomationModel(simulator, simConfig);
            simulator.initialize(Time.ZERO, Duration.ZERO, Duration.instantiateSI(simConfig.getSimTime()), otsModel);
            simulator.start();
        }
        catch (SimRuntimeException | NamingException | IndexOutOfBoundsException exception)
        {
            exception.printStackTrace();
        }
    }
    
}

    