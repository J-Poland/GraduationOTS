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
import sim.demo.vehicleconfigurations.VehicleAutomationConfigurations;

/**
 * Task-demand for road-side distraction.
 * <p>
 * Copyright (c) 2013-2024 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved.
 * <br>
 * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
 * </p>
 * @author <a href="https://github.com/averbraeck">Alexander Verbraeck</a>
 * @author <a href="https://tudelft.nl/staff/p.knoppers-1">Peter Knoppers</a>
 * @author <a href="https://github.com/wjschakel">Wouter Schakel</a>
 */
public class TaskDriverDistraction extends AbstractTask
{
	// in-vehicle distraction variables
	
	/** Distraction state for in-vehicle distractions. */
	boolean isInVehicleDistracted;
	
	/** In-vehcile distraction enabled or disabled. */
	boolean inVehicleDistractionEnabled;
	
	/** Percentage of distracted drivers by secondary driver tasks. */
	static double fractionOfDistractedDrivers = 0.25;
	
	/** Length of secondary tasks. */
	static double secondaryTaskLength = 10.0;
	
	/** Distribution for random values. */
	private DistUniform randomValueDist;
	
	/** Start time of distraction. */
	double distractionStartTime = 0;
	
	
	// road side distraction variables
	
	/** Distraction state for road side distraction. */
	boolean isRoadSideDistracted;
	
	/** Road side distraction enabled or disabled. */
	boolean roadSideDistractionEnabled;
	
	/** Distraction location x along side the road. */
	double distractionX;
	

    /** Constructor. */
    public TaskDriverDistraction(StreamInterface stream, boolean inVehicleDistractionEnabled, boolean roadSideDistractionEnabled, double distractionX)
    {
        super("driver distraction");
        
        // in-vehicle distraction
        randomValueDist = new DistUniform(stream, 0, 1);
        this.inVehicleDistractionEnabled = inVehicleDistractionEnabled;
        
        // road side distraction
        this.roadSideDistractionEnabled = roadSideDistractionEnabled;
        this.distractionX = distractionX;
    }
    
    public boolean getInVehicleDistractionState(final LanePerception perception, final LaneBasedGtu gtu, final Parameters parameters)
            throws ParameterException, GtuException
    {
    	// check whether in-vehicle distraction is enabled
    	if (!inVehicleDistractionEnabled) {
    		return false;
    	}
    	
    	// get current simulation time
    	double currentTime = gtu.getSimulator().getSimulatorAbsTime().si;
    	
    	// was this GTU is already experiencing a distraction?
    	if (isInVehicleDistracted) {
    		// check whether the distraction time should be over
    		if (distractionStartTime + secondaryTaskLength <= currentTime) {
    			// reset distraction variables
    			distractionStartTime = 0;
    			isInVehicleDistracted = false;
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
    			isInVehicleDistracted = true;
    		}
    	}
    	
    	// update GTU parameter
    	gtu.getParameters().setParameter(VehicleAutomationConfigurations.SECONDARY_TASK_DISTRACTED, isInVehicleDistracted);
    	
		// return distraction demand
		return isInVehicleDistracted;
    }

    /**
     * Return state of road side distraction.
     * @param perception
     * @param gtu
     * @param parameters
     * @return
     * @throws ParameterException
     * @throws GtuException
     */
    public boolean getRoadSideDistractionState(final LanePerception perception, final LaneBasedGtu gtu, final Parameters parameters)
            throws ParameterException, GtuException
    {
    	// check whether road side distraction is enabled
    	if (!roadSideDistractionEnabled) {
    		return false;
    	}
    	
    	// GTU perception range
    	double lookAhead = gtu.getParameters().getParameterOrNull(ParameterTypes.LOOKAHEAD).si;
    	double lookBack = gtu.getParameters().getParameterOrNull(ParameterTypes.LOOKBACK).si;
    	// GTU location
        double gtuX = gtu.getLocation().getX();
        // distraction range
        double startDistractionX = distractionX - lookAhead;
        double endDistractionX = distractionX + lookBack;
        
        // set distraction state
    	if (startDistractionX < gtuX && gtuX < endDistractionX) {
    		isRoadSideDistracted = true;
    	}
    	else {
    		isRoadSideDistracted = false;
    	}
    	
    	// update GTU parameter
    	gtu.getParameters().setParameter(VehicleAutomationConfigurations.ROAD_SIDE_DISTRACTED, isRoadSideDistracted);

        return isRoadSideDistracted;
    }

	@Override
	public double calculateTaskDemand(LanePerception perception, LaneBasedGtu gtu, Parameters parameters)
			throws ParameterException, GtuException {
		System.err.println("Driver distraction demand is managed in the TaskManagerAR class.");
		return 0;
	}

}
