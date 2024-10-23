package sim.demo.mental;

import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.base.parameters.ParameterTypes;
import org.opentrafficsim.base.parameters.Parameters;
import org.opentrafficsim.core.gtu.GtuException;
import org.opentrafficsim.road.gtu.lane.LaneBasedGtu;
import org.opentrafficsim.road.gtu.lane.perception.LanePerception;
import org.opentrafficsim.road.gtu.lane.perception.mental.AbstractTask;

import nl.tudelft.simulation.jstats.distributions.DistUniform;
import nl.tudelft.simulation.jstats.streams.StreamInterface;
import sim.demo.VehicleConfigurations;


/**
 * Task-demand for in-vehicle distraction which simulates secondary driver tasks.
 * <p>
 * Copyright (c) 2013-2024 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved. <br>
 * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
 * </p>
 * @author <a href="https://github.com/averbraeck">Alexander Verbraeck</a>
 * @author <a href="https://github.com/peter-knoppers">Peter Knoppers</a>
 * @author <a href="https://github.com/wjschakel">Wouter Schakel</a>
 */
public class TaskInVehicleDistraction extends AbstractTask
{
	/** Percentage of distracted drivers by secondary driver tasks. */
	static double fractionOfDistractedDrivers = 0.25;
	
	/** Demand of secondary driving tasks. */
	static double secondaryTaskDemand = 0.2;
	
	/** Length of secondary tasks. */
	static double secondaryTaskLength = 10.0;
	
	/** Distribution for random values. */
	private DistUniform randomValueDist;
	
	/** Start time of distraction. */
	double distractionStartTime = 0;
	
	/** Distraction demand. */
	double demand = 0;
	
	boolean distracted = false;
	

    /** Constructor. */
    public TaskInVehicleDistraction(StreamInterface stream)
    {
        super("road-side distraction");
        
        randomValueDist = new DistUniform(stream, 0, 1);
    }

    /** {@inheritDoc} */
    @Override
    public double calculateTaskDemand(final LanePerception perception, final LaneBasedGtu gtu, final Parameters parameters)
            throws ParameterException, GtuException
    {
    	// get current simulation time
    	double currentTime = gtu.getSimulator().getSimulatorAbsTime().si;
    	
    	// was this GTU is already experiencing a distraction?
    	if (demand > 0) {
    		// check whether the distraction time should be over
    		if (distractionStartTime + secondaryTaskLength <= currentTime) {
    			// reset distraction variables
    			distractionStartTime = 0;
    			demand = 0;
    		}
    	}
    	
    	// should this GTU become distracted?
    	else {
    		// randomly assign whether this GTU is distracted
    		// determine the threshold by dividing the fraction of distracted drivers by (task length / time step)
    		double gtuTimeStep = gtu.getParameters().getParameter(ParameterTypes.DT).si;
    		double distractionThreshold = fractionOfDistractedDrivers / (secondaryTaskLength / gtuTimeStep);
    		if (randomValueDist.draw() <= distractionThreshold) {
    			distractionStartTime = currentTime;
    			demand = secondaryTaskDemand;
    		}
    	}
    	
    	// update GTU parameter
    	gtu.getParameters().setParameter(VehicleConfigurations.SECONDARY_TASK_DEMAND, demand);
    	
		// return distraction demand
		return demand;
    }

}
