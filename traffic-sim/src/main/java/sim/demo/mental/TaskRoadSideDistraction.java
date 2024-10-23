package sim.demo.mental;

import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.djunits.value.vdouble.scalar.Length;
import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.base.parameters.ParameterTypes;
import org.opentrafficsim.base.parameters.Parameters;
import org.opentrafficsim.core.gtu.GtuException;
import org.opentrafficsim.core.gtu.RelativePosition;
import org.opentrafficsim.core.network.Link;
import org.opentrafficsim.road.gtu.lane.LaneBasedGtu;
import org.opentrafficsim.road.gtu.lane.perception.LanePerception;
import org.opentrafficsim.road.gtu.lane.perception.RelativeLane;
import org.opentrafficsim.road.gtu.lane.perception.mental.AbstractTask;
import org.opentrafficsim.road.gtu.lane.perception.structure.LaneStructure.Entry;
import org.opentrafficsim.road.network.lane.CrossSectionLink;
import org.opentrafficsim.road.network.lane.Lane;
import org.opentrafficsim.road.network.lane.object.LaneBasedObject;

import sim.demo.VehicleConfigurations;
import sim.demo.object.CustomDistraction;

/**
 * Task-demand for road-side distraction.
 * <p>
 * Copyright (c) 2013-2024 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved. <br>
 * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
 * </p>
 * @author <a href="https://github.com/averbraeck">Alexander Verbraeck</a>
 * @author <a href="https://github.com/peter-knoppers">Peter Knoppers</a>
 * @author <a href="https://github.com/wjschakel">Wouter Schakel</a>
 */
public class TaskRoadSideDistraction extends AbstractTask
{

    /** Odometer values at distraction. */
    private Map<CustomDistraction, Double> odos = new LinkedHashMap<>();

//    private Map<String, Double> distractions = new LinkedHashMap<>();

    /** Constructor. */
    public TaskRoadSideDistraction()
    {
        super("road-side distraction");
    }

    /** {@inheritDoc} */
    @Override
    public double calculateTaskDemand(final LanePerception perception, final LaneBasedGtu gtu, final Parameters parameters)
            throws ParameterException, GtuException
    {
        double odo = gtu.getOdometer().si;
        
//        if (gtu.getLocation().getX() > 580.0 && gtu.getLocation().getX() < 1080.0) {
//	        System.out.println("\nGTU on lane: " + gtu.getReferencePosition().lane().getFullId() + " - x=" + gtu.getLocation().getX());
//	        System.out.println("Distraction objects:");
//        }
        
//        for (Link link : gtu.getNetwork().getLinkMap().values()) {
//        	CrossSectionLink csLink = (CrossSectionLink) link;
//        	List<Lane> allLanes = csLink.getLanes();
//        	for (Lane lane : allLanes) {
//        		List<LaneBasedObject> objects = lane.getLaneBasedObjects();
//        		for (LaneBasedObject object : objects) {
//        			System.out.println("Object: " + object.getFullId());
//        			
//        			String objectName = object.getFullId();
//        			if (objectName.contains("distraction")) {
//        				double distractionPos = object.getLocation().getX();
//        				this.distractions.put(objectName, distractionPos);
//        			}
//        		}
//        	}
//        }
        
        for (RelativeLane lane : perception.getLaneStructure().getRootCrossSection())
        {
            for (Entry<CustomDistraction> distraction : perception.getLaneStructure().getDownstreamObjects(lane, CustomDistraction.class,
                    RelativePosition.FRONT, false))
            {
                this.odos.put(distraction.object(), odo + distraction.distance().si);
//                System.out.println(distraction.object().getFullId());
            }
        }

        // loop over all distractions in odos
        Iterator<CustomDistraction> it = this.odos.keySet().iterator();
        double demand = 0.0;
        while (it.hasNext())
        {
            CustomDistraction next = it.next();
            Double distraction = next.getDistraction(gtu , Length.instantiateSI(odo - this.odos.get(next)));
            if (distraction == null)
            {
                it.remove();
            }
            else
            {
                demand += distraction;
            }
        }
        
        
//        // new
//        double demand2 = 0;
//        for (double distractionPos : this.distractions.values()) {
//        	double gtuPos = gtu.getLocation().getX();
//        	double lookAhead = gtu.getParameters().getParameterOrNull(ParameterTypes.LOOKAHEAD).si;
//        	double lookBack = gtu.getParameters().getParameterOrNull(ParameterTypes.LOOKBACK).si;
//        	double distance = gtuPos - distractionPos;
//            if (distance < -lookAhead || distance > lookBack)
//            {
//                demand2 += 0;
//            }
//            else if (distance <= 0)
//            {
//            	double distraction = 0.1 * (Math.abs(distractionPos - gtuPos) / lookAhead);
//                demand2 += distraction;
//            }
//            else if (distance > 0) {
//            	double distraction = 0.1 * (Math.abs(distractionPos - gtuPos) / lookBack);
//                demand2 += distraction;
//            }
//        }

        
//        if (gtu.getLocation().getX() > 580.0 && gtu.getLocation().getX() < 1080.0) {
//	        System.out.println("demand=" + demand);
//        }
        
        // update GTU parameter
        gtu.getParameters().setParameter(VehicleConfigurations.ROAD_SIDE_DISTRACTION_DEMAND, demand);
        
        // return demand
        return demand;
    }

}
