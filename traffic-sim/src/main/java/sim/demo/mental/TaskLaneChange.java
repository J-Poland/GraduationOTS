package sim.demo.mental;

import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.base.parameters.Parameters;
import org.opentrafficsim.core.gtu.GtuException;
import org.opentrafficsim.road.gtu.lane.LaneBasedGtu;
import org.opentrafficsim.road.gtu.lane.perception.LanePerception;
import org.opentrafficsim.road.gtu.lane.perception.mental.AbstractTask;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.LmrsParameters;

import sim.demo.VehicleConfigurations;

/**
 * Lane change task. Task demand is equal to the maximum of <i>d<sub>left</sub></i> and <i>d<sub>right</sub></i>.
 * @author wjschakel
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
    public double calculateTaskDemand(final LanePerception perception, final LaneBasedGtu gtu,
            final Parameters parameters) throws ParameterException, GtuException
    {
    	// only add task demand for non automated lane changing (automation levels 0, 1)
    	String gtuType = gtu.getParameters().getParameterOrNull(VehicleConfigurations.AUTOMATION_LEVEL);
    	if (gtuType != null && (gtuType.contains("LEVEL0") || gtuType.contains("LEVEL1"))) {
    		double demand = Math.max(0.0,
                    Math.max(parameters.getParameter(LmrsParameters.DLEFT), parameters.getParameter(LmrsParameters.DRIGHT)));
    		
    		// update GTU parameter
            gtu.getParameters().setParameter(VehicleConfigurations.LC_TASK_DEMAND, demand);
            
            // return demand
            return demand;
    	}
    	else {
    		double demand = 0.0;
    		
    		// update GTU parameter
            gtu.getParameters().setParameter(VehicleConfigurations.LC_TASK_DEMAND, demand);
            
            // return demand
            return demand;
    	}
    }
}
