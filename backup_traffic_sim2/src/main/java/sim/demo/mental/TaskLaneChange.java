package sim.demo.mental;

import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.base.parameters.Parameters;
import org.opentrafficsim.core.gtu.GtuException;
import org.opentrafficsim.road.gtu.lane.LaneBasedGtu;
import org.opentrafficsim.road.gtu.lane.perception.LanePerception;
import org.opentrafficsim.road.gtu.lane.perception.mental.AbstractTask;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.LmrsParameters;

import sim.demo.vehicleconfigurations.VehicleAutomationConfigurations;

/**
 * Lane change task demand depending on lane change desire.
 */
public class TaskLaneChange extends AbstractTask
{
    /**
     * Constructor.
     */
    public TaskLaneChange()
    {
        super("lane-changing");
    }

    /** {@inheritDoc} */
    @Override
    public double calculateTaskDemand(final LanePerception perception, final LaneBasedGtu gtu, final Parameters parameters)
            throws ParameterException, GtuException
    {
    	double demand = Math.max(0.0, Math.max(parameters.getParameter(LmrsParameters.DLEFT), parameters.getParameter(LmrsParameters.DRIGHT)));
    	
    	// update task demand parameter
        gtu.getParameters().setParameter(VehicleAutomationConfigurations.CF_TASK_DEMAND, demand);
        
        // return demand
        return demand;
    }
}
