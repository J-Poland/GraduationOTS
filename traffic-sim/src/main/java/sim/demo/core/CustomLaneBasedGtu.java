package sim.demo.core;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import java.util.SortedMap;

import org.djunits.unit.DirectionUnit;
import org.djunits.unit.DurationUnit;
import org.djunits.unit.LengthUnit;
import org.djunits.unit.PositionUnit;
import org.djunits.value.vdouble.scalar.Acceleration;
import org.djunits.value.vdouble.scalar.Direction;
import org.djunits.value.vdouble.scalar.Duration;
import org.djunits.value.vdouble.scalar.Length;
import org.djunits.value.vdouble.scalar.Speed;
import org.djunits.value.vdouble.scalar.Time;
import org.djunits.value.vdouble.vector.PositionVector;
import org.djutils.draw.line.PolyLine2d;
import org.djutils.draw.point.OrientedPoint2d;
import org.djutils.draw.point.Point2d;
import org.djutils.event.EventType;
import org.djutils.exceptions.Throw;
import org.djutils.exceptions.Try;
import org.djutils.immutablecollections.ImmutableLinkedHashSet;
import org.djutils.immutablecollections.ImmutableSet;
import org.djutils.logger.CategoryLogger;
import org.djutils.metadata.MetaData;
import org.djutils.metadata.ObjectDescriptor;
import org.djutils.multikeymap.MultiKeyMap;
import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.base.parameters.ParameterTypes;
import org.opentrafficsim.core.dsol.OtsSimulatorInterface;
import org.opentrafficsim.core.geometry.OtsGeometryException;
import org.opentrafficsim.core.geometry.OtsLine2d;
import org.opentrafficsim.core.geometry.OtsLine2d.FractionalFallback;
import org.opentrafficsim.core.gtu.Gtu;
import org.opentrafficsim.core.gtu.GtuException;
import org.opentrafficsim.core.gtu.GtuType;
import org.opentrafficsim.core.gtu.RelativePosition;
import org.opentrafficsim.core.gtu.TurnIndicatorStatus;
import org.opentrafficsim.core.gtu.perception.EgoPerception;
import org.opentrafficsim.core.gtu.plan.operational.OperationalPlan;
import org.opentrafficsim.core.gtu.plan.operational.OperationalPlanException;
import org.opentrafficsim.core.gtu.plan.operational.Segments;
import org.opentrafficsim.core.gtu.plan.tactical.TacticalPlanner;
import org.opentrafficsim.core.network.LateralDirectionality;
import org.opentrafficsim.core.network.Link;
import org.opentrafficsim.core.network.NetworkException;
import org.opentrafficsim.core.perception.Historical;
import org.opentrafficsim.core.perception.HistoricalValue;
import org.opentrafficsim.core.perception.HistoryManager;
import org.opentrafficsim.core.perception.collections.HistoricalArrayList;
import org.opentrafficsim.core.perception.collections.HistoricalList;
import org.opentrafficsim.road.gtu.lane.LaneBasedGtu;
import org.opentrafficsim.road.gtu.lane.VehicleModel;
import org.opentrafficsim.road.gtu.lane.perception.LanePerception;
import org.opentrafficsim.road.gtu.lane.perception.PerceptionCollectable;
import org.opentrafficsim.road.gtu.lane.perception.RelativeLane;
import org.opentrafficsim.road.gtu.lane.perception.categories.InfrastructurePerception;
import org.opentrafficsim.road.gtu.lane.perception.categories.neighbors.NeighborsPerception;
import org.opentrafficsim.road.gtu.lane.perception.headway.HeadwayGtu;
import org.opentrafficsim.road.gtu.lane.plan.operational.LaneBasedOperationalPlan;
import org.opentrafficsim.road.gtu.lane.tactical.LaneBasedTacticalPlanner;
import org.opentrafficsim.road.gtu.strategical.LaneBasedStrategicalPlanner;
import org.opentrafficsim.road.network.RoadNetwork;
import org.opentrafficsim.road.network.lane.CrossSectionLink;
import org.opentrafficsim.road.network.lane.Lane;
import org.opentrafficsim.road.network.lane.LanePosition;
import org.opentrafficsim.road.network.lane.object.detector.LaneDetector;
import org.opentrafficsim.road.network.speed.SpeedLimitInfo;
import org.opentrafficsim.road.network.speed.SpeedLimitTypes;

import nl.tudelft.simulation.dsol.SimRuntimeException;
import nl.tudelft.simulation.dsol.formalisms.eventscheduling.SimEvent;
import nl.tudelft.simulation.dsol.formalisms.eventscheduling.SimEventInterface;

import java.lang.reflect.Field;

/**
 * This class contains most of the code that is needed to run a lane based GTU. <br>
 * The starting point of a LaneBasedTU is that it can be in <b>multiple lanes</b> at the same time. This can be due to a lane
 * change (lateral), or due to crossing a link (front of the GTU is on another Lane than rear of the GTU). If a Lane is shorter
 * than the length of the GTU (e.g. when we do node expansion on a crossing, this is very well possible), a GTU could occupy
 * dozens of Lanes at the same time.
 * <p>
 * When calculating a headway, the GTU has to look in successive lanes. When Lanes (or underlying CrossSectionLinks) diverge,
 * the headway algorithms have to look at multiple Lanes and return the minimum headway in each of the Lanes. When the Lanes (or
 * underlying CrossSectionLinks) converge, "parallel" traffic is not taken into account in the headway calculation. Instead, gap
 * acceptance algorithms or their equivalent should guide the merging behavior.
 * <p>
 * To decide its movement, an AbstractLaneBasedGtu applies its car following algorithm and lane change algorithm to set the
 * acceleration and any lane change operation to perform. It then schedules the triggers that will add it to subsequent lanes
 * and remove it from current lanes as needed during the time step that is has committed to. Finally, it re-schedules its next
 * movement evaluation with the simulator.
 * <p>
 * Copyright (c) 2013-2024 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved. <br>
 * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
 * </p>
 * @author <a href="https://github.com/averbraeck">Alexander Verbraeck</a>
 * @author <a href="https://tudelft.nl/staff/p.knoppers-1">Peter Knoppers</a>
 */
public class CustomLaneBasedGtu extends LaneBasedGtu
{
    /** */
    private static final long serialVersionUID = 20140822L;

    /** Lanes. */
    private final HistoricalList<CrossSection> crossSections;

    /** Reference lane index (0 = left or only lane, 1 = right lane). */
    private int referenceLaneIndex = 0;

    /** Time of reference position cache. */
    private double referencePositionTime = Double.NaN;

    /** Cached reference position. */
    private LanePosition cachedReferencePosition = null;

    /** Pending leave triggers for each lane. */
    private SimEventInterface<Duration> pendingLeaveTrigger;

    /** Pending enter triggers for each lane. */
    private SimEventInterface<Duration> pendingEnterTrigger;

    /** Event to finalize lane change. */
    private SimEventInterface<Duration> finalizeLaneChangeEvent;

    /** Sensor events. */
    private Set<SimEventInterface<Duration>> sensorEvents = new LinkedHashSet<>();

    /** Cached desired speed. */
    private Speed cachedDesiredSpeed;

    /** Time desired speed was cached. */
    private Time desiredSpeedTime;

    /** Cached car-following acceleration. */
    private Acceleration cachedCarFollowingAcceleration;

    /** Time car-following acceleration was cached. */
    private Time carFollowingAccelerationTime;

    /** The object to lock to make the GTU thread safe. */
    private Object lock = new Object();

    /** The threshold distance for differences between initial locations of the GTU on different lanes. */
    @SuppressWarnings("checkstyle:visibilitymodifier")
    public static Length initialLocationThresholdDifference = new Length(1.0, LengthUnit.MILLIMETER);

    /** Margin to add to plan to check of the path will enter the next section. */
    public static Length eventMargin = Length.instantiateSI(50.0);

    /** Turn indicator status. */
    private final Historical<TurnIndicatorStatus> turnIndicatorStatus;

    /** Caching on or off. */
    // TODO: should be indicated with a Parameter
    public static boolean CACHING = true;

    /** cached position count. */
    // TODO: can be removed after testing period
    public static int CACHED_POSITION = 0;

    /** cached position count. */
    // TODO: can be removed after testing period
    public static int NON_CACHED_POSITION = 0;

    /** Vehicle model. */
    private VehicleModel vehicleModel = VehicleModel.MINMAX;

    /** Whether the GTU perform lane changes instantaneously or not. */
    private boolean instantaneousLaneChange = false;

    /** Distance over which the GTU should not change lane after being created. */
    private Length noLaneChangeDistance;
    
    /** Custom variables to mirror private Gtu variables */
    protected Historical<TacticalPlanner<?, ?>> customTacticalPlanner;
    protected Historical<OperationalPlan> customOperationalPlan;
    protected double customCachedSpeedTime;
    protected double customCachedAccelerationTime;
    protected Historical<Length> customOdometer;
    protected SimEvent<Duration> customNextMoveEvent;

    /**
     * Construct a Lane Based GTU.
     * @param id String; the id of the GTU
     * @param gtuType GtuType; the type of GTU, e.g. TruckType, CarType, BusType
     * @param length Length; the maximum length of the GTU (parallel with driving direction)
     * @param width Length; the maximum width of the GTU (perpendicular to driving direction)
     * @param maximumSpeed Speed;the maximum speed of the GTU (in the driving direction)
     * @param front Length; front distance relative to the reference position
     * @param network RoadNetwork; the network that the GTU is initially registered in
     * @throws GtuException when initial values are not correct
     */
    public CustomLaneBasedGtu(final String id, final GtuType gtuType, final Length length, final Length width,
            final Speed maximumSpeed, final Length front, final RoadNetwork network) throws GtuException
    {
        super(id, gtuType, length, width, maximumSpeed, front, network);
        
        HistoryManager historyManager = network.getSimulator().getReplication().getHistoryManager(network.getSimulator());
        this.crossSections = new HistoricalArrayList<>(historyManager);
        this.turnIndicatorStatus = new HistoricalValue<>(historyManager, TurnIndicatorStatus.NOTPRESENT);
    }
    
    /**
     * Custom move method copied from Gtu.java to allow GTUs to create operational plans with dynamic durations. These durations are set by the
     * specified time step in ParameterTypes.DT.
     * 
     * Move from the current location according to an operational plan to a location that will bring us nearer to reaching the
     * location provided by the strategical planner. <br>
     * This method can be overridden to carry out specific behavior during the execution of the plan (e.g., scheduling of
     * triggers, entering or leaving lanes, etc.). Please bear in mind that the call to super.move() is essential, and that one
     * has to take care to handle the situation that the plan gets interrupted.
     * @param fromLocation OrientedPoint2d; the last known location (initial location, or end location of the previous
     *            operational plan)
     * @return boolean; whether an exception occurred
     * @throws SimRuntimeException when scheduling of the next move fails
     * @throws OperationalPlanException when there is a problem creating a good path for the GTU
     * @throws GtuException when there is a problem with the state of the GTU when planning a path
     * @throws NetworkException in case of a problem with the network, e.g., a dead end where it is not expected
     * @throws ParameterException in there is a parameter problem
     */
    @SuppressWarnings("checkstyle:designforextension")
    protected boolean customMove(final OrientedPoint2d fromLocation)
            throws SimRuntimeException, OperationalPlanException, GtuException, NetworkException, ParameterException
    {
        try
        {   
            Time now = this.getSimulator().getSimulatorAbsTime();

            // Add the odometer distance from the currently running operational plan.
            // Because a plan can be interrupted, we explicitly calculate the covered distance till 'now'
            Length currentOdometer;
            if (this.getOperationalPlan() != null)
            {
                currentOdometer = this.getOdometer().plus(this.getOperationalPlan().getTraveledDistance(now));
            }
            else
            {
                currentOdometer = this.getOdometer();
            }

            // Do we have an operational plan?
            // TODO discuss when a new tactical planner may be needed
            TacticalPlanner<?, ?> tactPlanner = this.getTacticalPlanner();
            if (tactPlanner == null)
            {
                // Tell the strategical planner to provide a tactical planner
                tactPlanner = this.getStrategicalPlanner().getTacticalPlanner();
                this.customTacticalPlanner.set(tactPlanner);
                
                updateTacticalPlanner();
            }
            synchronized (this)
            {
                tactPlanner.getPerception().perceive();
            }
            OperationalPlan newOperationalPlan = tactPlanner.generateOperationalPlan(now, fromLocation);
            synchronized (this)
            {
                this.customOperationalPlan.set(newOperationalPlan);
                this.customCachedSpeedTime = Double.NaN;
                this.customCachedAccelerationTime = Double.NaN;
                this.customOdometer.set(currentOdometer);
                
                updateOtherCustomVariables();
            }

         // TODO allow alignment at different intervals, also different between GTU's within a single simulation
            // if (ALIGNED && newOperationalPlan.getTotalDuration().si == 0.5)
            // solution: make 0.5 dynamic to GTU parameters
            double gtuTimeStep = 0.5;
            try {
                gtuTimeStep = this.getParameters().getParameter(ParameterTypes.DT).si;
            } catch (ParameterException ex) {
                this.getSimulator().getLogger().always()
                .error("No ParameterTypes.DT set for " + this.getId() + " at t=" + this.getSimulator().getSimulatorTime() + ", so default time step of 0.5 is used.");
            }
            if (ALIGNED && newOperationalPlan.getTotalDuration().si == gtuTimeStep)
            {
                // schedule the next move at exactly 0.5 seconds on the clock
                // store the event, so it can be cancelled in case the plan has to be interrupted and changed halfway
                double tNext = Math.floor(2.0 * now.si + 1.0) / 2.0;
                OrientedPoint2d p = (tNext - now.si < gtuTimeStep) ? newOperationalPlan.getEndLocation()
                        : newOperationalPlan.getLocation(new Duration(tNext - now.si, DurationUnit.SI));
                this.customNextMoveEvent =
                        new SimEvent<Duration>(new Duration(tNext - getSimulator().getStartTimeAbs().si, DurationUnit.SI), this,
                                "move", new Object[] {p});
                updateNextMoveEvent();
                ALIGN_COUNT++;
            }
            else
            {
                // schedule the next move at the end of the current operational plan
                // store the event, so it can be cancelled in case the plan has to be interrupted and changed halfway
                this.customNextMoveEvent =
                        new SimEvent<>(now.plus(newOperationalPlan.getTotalDuration()).minus(getSimulator().getStartTimeAbs()),
                                this, "move", new Object[] {newOperationalPlan.getEndLocation()});
                
                updateNextMoveEvent();
            }
            this.getSimulator().scheduleEvent(this.customNextMoveEvent);
            fireTimedEvent(Gtu.MOVE_EVENT,
                    new Object[] {getId(),
                            new PositionVector(new double[] {fromLocation.x, fromLocation.y}, PositionUnit.METER),
                            new Direction(fromLocation.getDirZ(), DirectionUnit.EAST_RADIAN), getSpeed(), getAcceleration(),
                            getOdometer()},
                    this.getSimulator().getSimulatorTime());

            return false;
        }
        catch (Exception ex)
        {
            try
            {
                this.getErrorHandler().handle(this, ex);
            }
            catch (Exception exception)
            {
                throw new GtuException(exception);
            }
            return true;
        }
    }
    
    protected void updateTacticalPlanner() throws NoSuchFieldException, SecurityException, IllegalArgumentException, IllegalAccessException
    {
    	// access private field from Gtu class
        Field tacticalPlannerField = Gtu.class.getDeclaredField("tacticalPlanner");
        // make field accessible
        tacticalPlannerField.setAccessible(true);
        // update value
        tacticalPlannerField.set(this, customTacticalPlanner);
    }
    
    protected void updateOtherCustomVariables() throws IllegalArgumentException, IllegalAccessException, NoSuchFieldException, SecurityException
    {
    	// access private field from Gtu class
        Field operationalPlanField = Gtu.class.getDeclaredField("operationalPlan");
        Field cachedSpeedTimeField = Gtu.class.getDeclaredField("cachedSpeedTime");
        Field cachedAccelerationTimeField = Gtu.class.getDeclaredField("cachedAccelerationTime");
        Field odometerField = Gtu.class.getDeclaredField("odometer");
        // make field accessible
        operationalPlanField.setAccessible(true);
        cachedSpeedTimeField.setAccessible(true);
        cachedAccelerationTimeField.setAccessible(true);
        odometerField.setAccessible(true);
        // update value
        operationalPlanField.set(this, this.customOperationalPlan);
        cachedSpeedTimeField.set(this, this.customCachedSpeedTime);
        cachedAccelerationTimeField.set(this, this.customCachedAccelerationTime);
        odometerField.set(this, this.customOdometer);
    }
    
    protected void updateNextMoveEvent() throws NoSuchFieldException, SecurityException, IllegalArgumentException, IllegalAccessException
    {
    	// access private field from Gtu class
        Field nextMoveEventField = Gtu.class.getDeclaredField("nextMoveEvent");
        // make field accessible
        nextMoveEventField.setAccessible(true);
        // update value
        nextMoveEventField.set(this, this.customNextMoveEvent);
    }

    /**
     * @param strategicalPlanner LaneBasedStrategicalPlanner; the strategical planner (e.g., route determination) to use
     * @param longitudinalPosition LanePosition; the initial position of the GTU
     * @param initialSpeed Speed; the initial speed of the car on the lane
     * @throws NetworkException when the GTU cannot be placed on the given lane
     * @throws SimRuntimeException when the move method cannot be scheduled
     * @throws GtuException when initial values are not correct
     * @throws OtsGeometryException when the initial path is wrong
     */
    @SuppressWarnings("checkstyle:designforextension")
    public void init(final LaneBasedStrategicalPlanner strategicalPlanner, final LanePosition longitudinalPosition,
            final Speed initialSpeed) throws NetworkException, SimRuntimeException, GtuException, OtsGeometryException
    {
        Throw.when(null == longitudinalPosition, GtuException.class, "InitialLongitudinalPositions is null");

        OrientedPoint2d initialLocation = longitudinalPosition.getLocation();

        // TODO: move this to super.init(...), and remove setOperationalPlan(...) method
        // Give the GTU a 1 micrometer long operational plan, or a stand-still plan, so the first move and events will work
        Time now = getSimulator().getSimulatorAbsTime();
        if (initialSpeed.si < OperationalPlan.DRIFTING_SPEED_SI)
        {
            setOperationalPlan(OperationalPlan.standStill(this, initialLocation, now, Duration.instantiateSI(1E-6)));
        }
        else
        {
            Point2d p2 = new Point2d(initialLocation.x + 1E-6 * Math.cos(initialLocation.getDirZ()),
                    initialLocation.y + 1E-6 * Math.sin(initialLocation.getDirZ()));
            OtsLine2d path = new OtsLine2d(initialLocation, p2);
            setOperationalPlan(new OperationalPlan(this, path, now,
                    Segments.off(initialSpeed, path.getLength().divide(initialSpeed), Acceleration.ZERO)));
        }

        enterLaneRecursive(longitudinalPosition.getLane(), longitudinalPosition.getPosition(), 0);

        // initiate the actual move
        super.init(strategicalPlanner, initialLocation, initialSpeed);

        this.referencePositionTime = Double.NaN; // remove cache, it may be invalid as the above init results in a lane change
    }

    /**
     * {@inheritDoc} All lanes the GTU is on will be left.
     */
    @Override
    public synchronized void setParent(final Gtu gtu) throws GtuException
    {
        leaveAllLanes();
        super.setParent(gtu);
    }

    /**
     * Removes the registration between this GTU and all the lanes.
     */
    private void leaveAllLanes()
    {
        for (CrossSection crossSection : this.crossSections)
        {
            boolean removeFromParentLink = true;
            for (Lane lane : crossSection.getLanes())
            {
                // GTU should be on this lane as we loop the registration
                Length pos = Try.assign(() -> position(lane, getReference()), "Unexpected exception.");
                lane.removeGtu(this, removeFromParentLink, pos);
                removeFromParentLink = false;
            }
        }
        this.crossSections.clear();
    }

    /**
     * Reinitializes the GTU on the network using the existing strategical planner and zero speed.
     * @param initialLongitudinalPosition LanePosition; initial position
     * @throws NetworkException when the GTU cannot be placed on the given lane
     * @throws SimRuntimeException when the move method cannot be scheduled
     * @throws GtuException when initial values are not correct
     * @throws OtsGeometryException when the initial path is wrong
     */
    public void reinit(final LanePosition initialLongitudinalPosition)
            throws NetworkException, SimRuntimeException, GtuException, OtsGeometryException
    {
        init(getStrategicalPlanner(), initialLongitudinalPosition, Speed.ZERO);
    }

    /**
     * Change lanes instantaneously.
     * @param laneChangeDirection LateralDirectionality; the direction to change to
     * @throws GtuException in case lane change fails
     */
    public synchronized void changeLaneInstantaneously(final LateralDirectionality laneChangeDirection) throws GtuException
    {

        // from info
        LanePosition from = getReferencePosition();

        // obtain position on lane adjacent to reference lane and enter lanes upstream/downstream from there
        Set<Lane> adjLanes = from.getLane().accessibleAdjacentLanesPhysical(laneChangeDirection, getType());
        Lane adjLane = adjLanes.iterator().next();
        Length position = adjLane.position(from.getLane().fraction(from.getPosition()));
        leaveAllLanes();
        enterLaneRecursive(adjLane, position, 0);

        // stored positions no longer valid
        this.referencePositionTime = Double.NaN;
        this.cachedPositions.clear();

        // fire event
        this.fireTimedEvent(
                LaneBasedGtu.LANE_CHANGE_EVENT, new Object[] {getId(), laneChangeDirection.name(),
                        from.getLane().getLink().getId(), from.getLane().getId(), from.getPosition()},
                getSimulator().getSimulatorTime());

    }

    /**
     * Enters lanes upstream and downstream of the new location after an instantaneous lane change or initialization.
     * @param lane Lane; considered lane
     * @param position Length; position to add GTU at
     * @param dir int; below 0 for upstream, above 0 for downstream, 0 for both<br>
     * @throws GtuException on exception
     */
    // TODO: the below 0 and above 0 is NOT what is tested
    private void enterLaneRecursive(final Lane lane, final Length position, final int dir) throws GtuException
    {
        List<Lane> lanes = new ArrayList<>();
        lanes.add(lane);
        int index = dir > 0 ? this.crossSections.size() : 0;
        this.crossSections.add(index, new CrossSection(lanes));
        lane.addGtu(this, position);

        // upstream
        if (dir < 1)
        {
            Length rear = position.plus(getRear().getDx());
            Length before = null;
            if (rear.si < 0.0)
            {
                before = rear.neg();
            }
            if (before != null)
            {
                ImmutableSet<Lane> upstream = new ImmutableLinkedHashSet<>(lane.prevLanes(getType()));
                if (!upstream.isEmpty())
                {
                    Lane upLane = null;
                    for (Lane nextUp : upstream)
                    {
                        for (CrossSection crossSection : this.crossSections)
                        {
                            if (crossSection.getLanes().contains(nextUp))
                            {
                                // multiple upstream lanes could belong to the same link, we pick an arbitrary lane
                                // (a conflict should solve this)
                                upLane = nextUp;
                                break;
                            }
                        }
                    }
                    if (upLane == null)
                    {
                        // the rear is on an upstream section we weren't before the lane change, due to curvature, we pick an
                        // arbitrary lane (a conflict should solve this)
                        upLane = upstream.iterator().next();
                    }
                    Lane next = upLane;
                    // TODO: this assumes lanes are perfectly attached
                    Length nextPos = next.getLength().minus(before).minus(getRear().getDx());
                    enterLaneRecursive(next, nextPos, -1);
                }
            }
        }

        // downstream
        if (dir > -1)
        {
            Length front = position.plus(getFront().getDx());
            Length passed = null;
            if (front.si > lane.getLength().si)
            {
                passed = front.minus(lane.getLength());
            }
            if (passed != null)
            {
                Lane next = getStrategicalPlanner() == null ? lane.nextLanes(getType()).iterator().next()
                        : getNextLaneForRoute(lane);
                // TODO: this assumes lanes are perfectly attached
                Length nextPos = passed.minus(getFront().getDx());
                enterLaneRecursive(next, nextPos, 1);
            }
        }
    }

    /**
     * Register on lanes in target lane.
     * @param laneChangeDirection LateralDirectionality; direction of lane change
     * @throws GtuException exception
     */
    @SuppressWarnings("checkstyle:designforextension")
    public synchronized void initLaneChange(final LateralDirectionality laneChangeDirection) throws GtuException
    {
        List<CrossSection> newLanes = new ArrayList<>();
        int index = laneChangeDirection.isLeft() ? 0 : 1;
        int numRegistered = 0;
        OrientedPoint2d point = getLocation();
        Map<Lane, Double> addToLanes = new LinkedHashMap<>();
        for (CrossSection crossSection : this.crossSections)
        {
            List<Lane> resultingLanes = new ArrayList<>();
            Lane lane = crossSection.getLanes().get(0);
            resultingLanes.add(lane);
            Set<Lane> laneSet = lane.accessibleAdjacentLanesLegal(laneChangeDirection, getType());
            if (laneSet.size() > 0)
            {
                numRegistered++;
                Lane adjacentLane = laneSet.iterator().next();
                double f = adjacentLane.getCenterLine().projectFractional(null, null, point.x, point.y, FractionalFallback.NaN);
                if (Double.isNaN(f))
                {
                    // the GTU is upstream or downstream of the lane, or on the edge and we have rounding problems
                    // in either case we add the GTU at an extreme
                    // (this is only for ordering on the lane, the position is not used otherwise)
                    Length pos = position(lane, getReference());
                    addToLanes.put(adjacentLane, pos.si < lane.getLength().si / 2 ? 0.0 : 1.0);
                }
                else
                {
                    addToLanes.put(adjacentLane, adjacentLane.getLength().times(f).si / adjacentLane.getLength().si);
                }
                resultingLanes.add(index, adjacentLane);
            }
            newLanes.add(new CrossSection(resultingLanes));
        }
        Throw.when(numRegistered == 0, GtuException.class, "Gtu %s starting %s lane change, but no adjacent lane found.",
                getId(), laneChangeDirection);
        this.crossSections.clear();
        this.crossSections.addAll(newLanes);
        for (Entry<Lane, Double> entry : addToLanes.entrySet())
        {
            entry.getKey().addGtu(this, entry.getValue());
        }
        this.referenceLaneIndex = 1 - index;
    }

    /**
     * Performs the finalization of a lane change by leaving the from lanes.
     * @param laneChangeDirection LateralDirectionality; direction of lane change
     * @throws GtuException if position or direction could not be obtained
     */
    @SuppressWarnings("checkstyle:designforextension")
    protected synchronized void finalizeLaneChange(final LateralDirectionality laneChangeDirection) throws GtuException
    {
        List<CrossSection> newLanes = new ArrayList<>();
        Lane fromLane = null;
        Length fromPosition = null;
        for (CrossSection crossSection : this.crossSections)
        {
            Lane lane = crossSection.getLanes().get(this.referenceLaneIndex);
            if (lane != null)
            {
                Length pos = position(lane, RelativePosition.REFERENCE_POSITION);
                if (0.0 <= pos.si && pos.si <= lane.getLength().si)
                {
                    fromLane = lane;
                    fromPosition = pos;
                }
                lane.removeGtu(this, false, pos);
            }
            List<Lane> remainingLane = new ArrayList<>();
            remainingLane.add(crossSection.getLanes().get(1 - this.referenceLaneIndex));
            newLanes.add(new CrossSection(remainingLane));
        }
        this.crossSections.clear();
        this.crossSections.addAll(newLanes);
        this.referenceLaneIndex = 0;

        Throw.when(fromLane == null, RuntimeException.class, "No from lane for lane change event.");
        LanePosition from = new LanePosition(fromLane, fromPosition);

        // XXX: WRONG: this.fireTimedEvent(LaneBasedGtu.LANE_CHANGE_EVENT, new Object[] {getId(), laneChangeDirection, from},
        // XXX: WRONG: getSimulator().getSimulatorTime());
        this.fireTimedEvent(
                LaneBasedGtu.LANE_CHANGE_EVENT, new Object[] {getId(), laneChangeDirection.name(),
                        from.getLane().getLink().getId(), from.getLane().getId(), from.getPosition()},
                getSimulator().getSimulatorTime());

        this.finalizeLaneChangeEvent = null;
    }

    /**
     * Sets event to finalize lane change.
     * @param event SimEventInterface&lt;SimTimeDoubleUnit&gt;; event
     */
    public void setFinalizeLaneChangeEvent(final SimEventInterface<Duration> event)
    {
        this.finalizeLaneChangeEvent = event;
    }

    /** {@inheritDoc} */
    @Override
    @SuppressWarnings("checkstyle:designforextension")
    protected synchronized boolean move(final OrientedPoint2d fromLocation)
            throws SimRuntimeException, GtuException, OperationalPlanException, NetworkException, ParameterException
    {
        if (this.isDestroyed())
        {
            return false;
        }
        try
        {
            if (this.crossSections.isEmpty())
            {
                destroy();
                return false; // Done; do not re-schedule execution of this move method.
            }

            // cancel events, if any
            // FIXME: If there are still events left, clearly something went wrong?
            // XXX: Added boolean to indicate whether warnings need to be given when events were found
            cancelAllEvents();

            // generate the next operational plan and carry it out
            // in case of an instantaneous lane change, fractionalLinkPositions will be accordingly adjusted to the new lane
            try
            {
                boolean error = super.move(fromLocation);
                if (error)
                {
                    return error;
                }
            }
            catch (Exception exception)
            {
                System.err.println(exception.getMessage());
                System.err.println("  GTU " + this + " DESTROYED AND REMOVED FROM THE SIMULATION");
                this.destroy();
                this.cancelAllEvents();
                return true;
            }

            LanePosition dlp = getReferencePosition();

            scheduleEnterEvent();
            scheduleLeaveEvent();

            // sensors
            for (CrossSection crossSection : this.crossSections)
            {
                for (Lane lane : crossSection.getLanes())
                {
                    scheduleTriggers(lane);
                }
            }

            fireTimedEvent(LaneBasedGtu.LANEBASED_MOVE_EVENT,
                    new Object[] {getId(),
                            new PositionVector(new double[] {fromLocation.x, fromLocation.y}, PositionUnit.METER),
                            new Direction(fromLocation.getDirZ(), DirectionUnit.EAST_RADIAN), getSpeed(), getAcceleration(),
                            getTurnIndicatorStatus().name(), getOdometer(), dlp.getLane().getLink().getId(),
                            dlp.getLane().getId(), dlp.getPosition()},
                    getSimulator().getSimulatorTime());

            return false;

        }
        catch (Exception ex)
        {
            try
            {
                getErrorHandler().handle(this, ex);
            }
            catch (Exception exception)
            {
                throw new GtuException(exception);
            }
            return true;
        }

    }

    /**
     * Cancels all future events.
     */
    private void cancelAllEvents()
    {
        if (this.pendingEnterTrigger != null)
        {
            getSimulator().cancelEvent(this.pendingEnterTrigger);
        }
        if (this.pendingLeaveTrigger != null)
        {
            getSimulator().cancelEvent(this.pendingLeaveTrigger);
        }
        if (this.finalizeLaneChangeEvent != null)
        {
            getSimulator().cancelEvent(this.finalizeLaneChangeEvent);
        }
        for (SimEventInterface<Duration> event : this.sensorEvents)
        {
            if (event.getAbsoluteExecutionTime().gt(getSimulator().getSimulatorTime()))
            {
                getSimulator().cancelEvent(event);
            }
        }
        this.sensorEvents.clear();
    }

    /**
     * Checks whether the GTU will enter a next cross-section during the (remainder of) the tactical plan. Only one event will
     * be scheduled. Possible additional events are scheduled upon entering the cross-section.
     * @throws GtuException exception
     * @throws OperationalPlanException exception
     * @throws SimRuntimeException exception
     */
    protected void scheduleEnterEvent() throws GtuException, OperationalPlanException, SimRuntimeException
    {
        CrossSection lastCrossSection = this.crossSections.get(this.crossSections.size() - 1);
        // heuristic to prevent geometric calculation if the next section is quite far away anyway
        Length remain = remainingEventDistance();
        Lane lane = lastCrossSection.getLanes().get(this.referenceLaneIndex);
        Length position = position(lane, getFront());
        boolean possiblyNearNextSection = lane.getLength().minus(position).lt(remain);
        if (possiblyNearNextSection)
        {
            CrossSectionLink link = lastCrossSection.getLanes().get(0).getLink();
            PolyLine2d enterLine = link.getEndLine();
            Time enterTime = timeAtLine(enterLine, getFront());
            if (enterTime != null)
            {
                if (Double.isNaN(enterTime.si))
                {
                    // NaN indicates we just missed it between moves, due to curvature and small gaps
                    enterTime = getSimulator().getSimulatorAbsTime();
                    CategoryLogger.always().error("GTU {} enters cross-section through hack.", getId());
                }
                if (enterTime.lt(getSimulator().getSimulatorAbsTime()))
                {
                    System.err.println(
                            "Time travel? enterTime=" + enterTime + "; simulator time=" + getSimulator().getSimulatorAbsTime());
                    enterTime = getSimulator().getSimulatorAbsTime();
                }
                this.pendingEnterTrigger = getSimulator().scheduleEventAbsTime(enterTime, this, "enterCrossSection", null);
            }
        }
    }

    /**
     * Appends a new cross-section at the downstream end. Possibly schedules a next enter event.
     * @throws GtuException exception
     * @throws OperationalPlanException exception
     * @throws SimRuntimeException exception
     */
    protected synchronized void enterCrossSection() throws GtuException, OperationalPlanException, SimRuntimeException
    {
        CrossSection lastCrossSection = this.crossSections.get(this.crossSections.size() - 1);
        Lane lcsLane = lastCrossSection.getLanes().get(this.referenceLaneIndex);
        Lane nextLcsLane = getNextLaneForRoute(lcsLane);
        if (nextLcsLane == null)
        {
            forceLaneChangeFinalization();
            return;
        }
        List<Lane> nextLanes = new ArrayList<>();
        for (int i = 0; i < lastCrossSection.getLanes().size(); i++)
        {
            if (i == this.referenceLaneIndex)
            {
                nextLanes.add(nextLcsLane);
            }
            else
            {
                Lane lane = lastCrossSection.getLanes().get(i);
                Set<Lane> lanes = lane.nextLanes(getType());
                if (lanes.size() == 1)
                {
                    Lane nextLane = lanes.iterator().next();
                    nextLanes.add(nextLane);
                }
                else
                {
                    boolean added = false;
                    for (Lane nextLane : lanes)
                    {
                        if (nextLane.getLink().equals(nextLcsLane.getLink())
                                && nextLane
                                        .accessibleAdjacentLanesPhysical(this.referenceLaneIndex == 0
                                                ? LateralDirectionality.LEFT : LateralDirectionality.RIGHT, getType())
                                        .contains(nextLcsLane))
                        {
                            nextLanes.add(nextLane);
                            added = true;
                            break;
                        }
                    }
                    if (!added)
                    {
                        forceLaneChangeFinalization();
                        return;
                    }
                }
            }
        }
        this.crossSections.add(new CrossSection(nextLanes));
        for (Lane lane : nextLanes)
        {
            lane.addGtu(this, 0.0);
        }
        this.pendingEnterTrigger = null;
        scheduleEnterEvent();
        for (Lane lane : nextLanes)
        {
            scheduleTriggers(lane);
        }
    }

    /**
     * Helper method for {@code enterCrossSection}. In some cases the GTU should first finalize the lane change. This method
     * checks whether such an event is scheduled, and performs it. This method then re-attempts to enter the cross-section. So
     * the calling method should return after calling this.
     * @throws GtuException exception
     * @throws OperationalPlanException exception
     * @throws SimRuntimeException exception
     */
    private void forceLaneChangeFinalization() throws GtuException, OperationalPlanException, SimRuntimeException
    {
        if (this.finalizeLaneChangeEvent != null)
        {
            // a lane change should be finalized at this time, but the event is later in the queue, force it now
            SimEventInterface<Duration> tmp = this.finalizeLaneChangeEvent;
            finalizeLaneChange(this.referenceLaneIndex == 0 ? LateralDirectionality.RIGHT : LateralDirectionality.LEFT);
            getSimulator().cancelEvent(tmp);
            enterCrossSection();
        }
        // or a sink sensor should delete us
    }

    /**
     * Checks whether the GTU will leave a cross-section during the (remainder of) the tactical plan. Only one event will be
     * scheduled. Possible additional events are scheduled upon leaving the cross-section.
     * @throws GtuException exception
     * @throws OperationalPlanException exception
     * @throws SimRuntimeException exception
     */
    protected void scheduleLeaveEvent() throws GtuException, OperationalPlanException, SimRuntimeException
    {
        if (this.crossSections.isEmpty())
        {
            CategoryLogger.always().error("GTU {} has empty crossSections", this);
            return;
        }
        CrossSection firstCrossSection = this.crossSections.get(0);
        // check, if reference lane is not in first cross section
        boolean possiblyNearNextSection =
                !getReferencePosition().getLane().equals(firstCrossSection.getLanes().get(this.referenceLaneIndex));
        if (!possiblyNearNextSection)
        {
            Length remain = remainingEventDistance();
            Lane lane = firstCrossSection.getLanes().get(this.referenceLaneIndex);
            Length position = position(lane, getRear());
            possiblyNearNextSection = lane.getLength().minus(position).lt(remain);
        }
        if (possiblyNearNextSection)
        {
            CrossSectionLink link = firstCrossSection.getLanes().get(0).getLink();
            PolyLine2d leaveLine = link.getEndLine();
            Time leaveTime = timeAtLine(leaveLine, getRear());
            if (leaveTime == null)
            {
                // no intersect, let's do a check on the rear
                Lane lane = this.crossSections.get(0).getLanes().get(this.referenceLaneIndex);
                Length pos = position(lane, getRear());
                if (pos.gt(lane.getLength()))
                {
                    pos = position(lane, getRear());
                    this.pendingLeaveTrigger = getSimulator().scheduleEventNow(this, "leaveCrossSection", null);
                    getSimulator().getLogger().always().info("Forcing leave for GTU {} on lane {}", getId(), lane.getFullId());
                }
            }
            if (leaveTime != null)
            {
                if (Double.isNaN(leaveTime.si))
                {
                    // NaN indicates we just missed it between moves, due to curvature and small gaps
                    leaveTime = getSimulator().getSimulatorAbsTime();
                    CategoryLogger.always().error("GTU {} leaves cross-section through hack.", getId());
                }
                if (leaveTime.lt(getSimulator().getSimulatorAbsTime()))
                {
                    System.err.println(
                            "Time travel? leaveTime=" + leaveTime + "; simulator time=" + getSimulator().getSimulatorAbsTime());
                    leaveTime = getSimulator().getSimulatorAbsTime();
                }
                this.pendingLeaveTrigger = getSimulator().scheduleEventAbsTime(leaveTime, this, "leaveCrossSection", null);
            }
        }
    }

    /**
     * Removes registration between the GTU and the lanes in the most upstream cross-section. Possibly schedules a next leave
     * event.
     * @throws GtuException exception
     * @throws OperationalPlanException exception
     * @throws SimRuntimeException exception
     */
    protected synchronized void leaveCrossSection() throws GtuException, OperationalPlanException, SimRuntimeException
    {

        List<Lane> lanes = this.crossSections.get(0).getLanes();
        for (int i = 0; i < lanes.size(); i++)
        {
            Lane lane = lanes.get(i);
            if (lane != null)
            {
                lane.removeGtu(this, i == lanes.size() - 1, position(lane, getReference()));
            }
        }
        this.crossSections.remove(0);
        this.pendingLeaveTrigger = null;
        scheduleLeaveEvent();
    }

    /**
     * Schedules all trigger events during the current operational plan on the lane.
     * @param lane Lane; lane
     * @throws GtuException exception
     * @throws OperationalPlanException exception
     * @throws SimRuntimeException exception
     */
    protected void scheduleTriggers(final Lane lane) throws GtuException, OperationalPlanException, SimRuntimeException
    {
        Length remain = remainingEventDistance();
        double min = position(lane, getRear()).si;
        double max = min + remain.si + getLength().si;
        SortedMap<Double, List<LaneDetector>> detectors = lane.getDetectorMap(getType()).subMap(min, max);
        for (List<LaneDetector> list : detectors.values())
        {
            for (LaneDetector detector : list)
            {
                RelativePosition pos = this.getRelativePositions().get(detector.getPositionType());
                Time time = timeAtLine(detector.getGeometry(), pos);
                if (time != null && !Double.isNaN(time.si))
                {
                    this.sensorEvents.add(getSimulator().scheduleEventAbsTime(time, detector, "trigger", new Object[] {this}));
                }
            }
        }
    }

    /**
     * Returns a safe distance beyond which a line will definitely not be crossed during the current operational plan.
     * @return Length; safe distance beyond which a line will definitely not be crossed during the current operational plan
     * @throws OperationalPlanException exception
     */
    private Length remainingEventDistance() throws OperationalPlanException
    {
        if (getOperationalPlan() instanceof LaneBasedOperationalPlan)
        {
            LaneBasedOperationalPlan plan = (LaneBasedOperationalPlan) getOperationalPlan();
            return plan.getTotalLength().minus(plan.getTraveledDistance(getSimulator().getSimulatorAbsTime()))
                    .plus(eventMargin);
        }
        return getOperationalPlan().getTotalLength().plus(eventMargin);
    }

    /**
     * Returns a set of {@code Lane}s that can be followed considering the route.
     * @param lane Lane; the lane for which we want to know the next Lane
     * @return set of {@code Lane}s that can be followed considering the route
     */
    public Set<Lane> getNextLanesForRoute(final Lane lane)
    {
        Set<Lane> next = lane.nextLanes(getType());
        if (next.isEmpty())
        {
            return null;
        }
        Link link;
        try
        {
            link = getStrategicalPlanner().nextLink(lane.getLink(), getType());
        }
        catch (NetworkException exception)
        {
            throw new RuntimeException("Strategical planner experiences exception on network.", exception);
        }
        Set<Lane> out = new LinkedHashSet<>();
        for (Lane l : next)
        {
            if (l.getLink().equals(link))
            {
                out.add(l);
            }
        }
        return out;
    }

    /**
     * Returns an estimation of when the relative position will reach the line. Returns {@code null} if this does not occur
     * during the current operational plan.
     * @param line PolyLine2d; line, i.e. lateral line at link start or lateral entrance of sensor
     * @param relativePosition RelativePosition; position to cross the line
     * @return estimation of when the relative position will reach the line, {@code null} if this does not occur during the
     *         current operational plan
     * @throws GtuException position error
     */
    private Time timeAtLine(final PolyLine2d line, final RelativePosition relativePosition) throws GtuException
    {
        Throw.when(line.size() != 2, IllegalArgumentException.class, "Line to cross with path should have 2 points.");
        OtsLine2d path = getOperationalPlan().getPath();
        Point2d[] points;
        double adjust;
        if (relativePosition.getDx().gt0())
        {
            // as the position is downstream of the reference, we need to attach some distance at the end
            points = new Point2d[path.size() + 1];
            System.arraycopy(path.getPoints(), 0, points, 0, path.size());
            points[path.size()] = path.getLocationExtendedSI(path.getLength().si + relativePosition.getDx().si);
            adjust = -relativePosition.getDx().si;
        }
        else if (relativePosition.getDx().lt0())
        {
            points = new Point2d[path.size() + 1];
            System.arraycopy(path.getPoints(), 0, points, 1, path.size());
            points[0] = path.getLocationExtendedSI(relativePosition.getDx().si);
            adjust = 0.0;
        }
        else
        {
            points = path.getPoints();
            adjust = 0.0;
        }

        // find intersection
        double cumul = 0.0;
        for (int i = 0; i < points.length - 1; i++)
        {
            Point2d intersect = Point2d.intersectionOfLineSegments(points[i], points[i + 1], line.get(0), line.get(1));

            /*
             * SKL 31-07-2023: Using the djunits code rather than the older OTS point and line code, causes an intersection on a
             * polyline to sometimes not be found, if the path has a point that is essentially on the line to cross. Clearly,
             * when entering a next lane/link, this is often the case as the GTU path is made from lane center lines that have
             * the endpoint of the lanes in it.
             */
            if (intersect == null)
            {
                double projectionFraction = line.projectOrthogonalFractionalExtended(points[i]);
                if (0.0 <= projectionFraction && projectionFraction <= 1.0)
                {
                    Point2d projection = line.getLocationFraction(projectionFraction);
                    double distance = projection.distance(points[i]);
                    if (distance < 1e-6)
                    {
                        // CategoryLogger.always().error("GTU {} enters cross-section through forced intersection of lines.",
                        // getId()); // this line pops up a lot in certain simulations making them slow
                        intersect = projection;
                    }
                }
            }

            if (intersect != null)
            {
                cumul += points[i].distance(intersect);
                cumul += adjust; // , 0.0); // possible rear is already considered in first segment
                // return time at distance
                if (cumul < 0.0)
                {
                    // return getSimulator().getSimulatorAbsTime(); // this was a mistake...
                    // relative position already crossed the point, e.g. FRONT
                    // SKL 08-02-2023: if the nose did not trigger at and of last move by mm's and due to vehicle rotation
                    // having been assumed straight, we should trigger it now. However, we should not double-trigger e.g.
                    // detectors. Let's return NaN to indicate this problem.
                    return Time.instantiateSI(Double.NaN);
                }
                if (cumul <= getOperationalPlan().getTotalLength().si)
                {
                    return getOperationalPlan().timeAtDistance(Length.instantiateSI(cumul));
                }
                // ref will cross the line, but GTU will not travel enough for rear to cross
                return null;
            }
            else if (i < points.length - 2)
            {
                cumul += points[i].distance(points[i + 1]);
            }
        }
        // no intersect
        return null;
    }

    /** Caching of time field for last stored position(s). */
    private double cachePositionsTime = Double.NaN;

    /** Caching of operation plan for last stored position(s). */
    private OperationalPlan cacheOperationalPlan = null;

    /** caching of last stored position(s). */
    private MultiKeyMap<Length> cachedPositions = new MultiKeyMap<>(Lane.class, RelativePosition.class);

    /**
     * Return the longitudinal position of a point relative to this GTU, relative to the center line of the Lane.
     * @param lane Lane; the position on this lane will be returned.
     * @param relativePosition RelativePosition; the position on the vehicle relative to the reference point.
     * @param when Time; the future time for which to calculate the positions.
     * @return DoubleScalarAbs&lt;LengthUnit&gt;; the position, relative to the center line of the Lane.
     * @throws GtuException when the vehicle is not on the given lane.
     */
    public Length position(final Lane lane, final RelativePosition relativePosition, final Time when) throws GtuException
    {
        synchronized (this)
        {
            OperationalPlan plan = getOperationalPlan(when);
            if (CACHING)
            {
                if (when.si == this.cachePositionsTime && plan == this.cacheOperationalPlan)
                {
                    Length l = this.cachedPositions.get(lane, relativePosition);
                    if (l != null && (!Double.isNaN(l.si)))
                    {
                        CACHED_POSITION++;
                        // PK verify the result; uncomment if you don't trust the cache
                        // this.cachedPositions.clear();
                        // Length difficultWay = position(lane, relativePosition, when);
                        // if (Math.abs(l.si - difficultWay.si) > 0.00001)
                        // {
                        // System.err.println("Whoops: cache returns bad value for GTU " + getId() + " cache returned " + l
                        // + ", re-computing yielded " + difficultWay);
                        // l = null; // Invalidate; to debug and try again
                        // }
                        // }
                        // if (l != null)
                        // {
                        return l;
                    }
                }
                if (when.si != this.cachePositionsTime || plan != this.cacheOperationalPlan)
                {
                    this.cachePositionsTime = Double.NaN;
                    this.cacheOperationalPlan = null;
                    this.cachedPositions.clear();
                }
            }
            NON_CACHED_POSITION++;

            synchronized (this.lock)
            {
                List<CrossSection> whenCrossSections = this.crossSections.get(when);
                double loc = Double.NaN;

                try
                {
                    int crossSectionIndex = -1;
                    int lateralIndex = -1;
                    for (int i = 0; i < whenCrossSections.size(); i++)
                    {
                        if (whenCrossSections.get(i).getLanes().contains(lane))
                        {
                            crossSectionIndex = i;
                            lateralIndex = whenCrossSections.get(i).getLanes().indexOf(lane);
                            break;
                        }
                    }
                    Throw.when(lateralIndex == -1, GtuException.class, "GTU %s is not on lane %s.", this, lane);

                    OrientedPoint2d p = plan.getLocation(when, relativePosition);
                    double f = lane.getCenterLine().projectFractional(lane.getLink().getStartNode().getHeading(),
                            lane.getLink().getEndNode().getHeading(), p.x, p.y, FractionalFallback.NaN);
                    if (!Double.isNaN(f))
                    {
                        loc = f * lane.getLength().si;
                    }
                    else
                    {
                        // the point does not project fractionally to this lane, it has to be up- or downstream of the lane
                        // try upstream
                        double distance = 0.0;
                        for (int i = crossSectionIndex - 1; i >= 0; i--)
                        {
                            Lane tryLane = whenCrossSections.get(i).getLanes().get(lateralIndex);
                            f = tryLane.getCenterLine().projectFractional(tryLane.getLink().getStartNode().getHeading(),
                                    tryLane.getLink().getEndNode().getHeading(), p.x, p.y, FractionalFallback.NaN);
                            if (!Double.isNaN(f))
                            {
                                f = 1 - f;
                                loc = distance - f * tryLane.getLength().si;
                                break;
                            }
                            distance -= tryLane.getLength().si;
                        }
                        // try downstream
                        if (Double.isNaN(loc))
                        {
                            distance = lane.getLength().si;
                            for (int i = crossSectionIndex + 1; i < whenCrossSections.size(); i++)
                            {
                                Lane tryLane = whenCrossSections.get(i).getLanes().get(lateralIndex);
                                f = tryLane.getCenterLine().projectFractional(tryLane.getLink().getStartNode().getHeading(),
                                        tryLane.getLink().getEndNode().getHeading(), p.x, p.y, FractionalFallback.NaN);
                                if (!Double.isNaN(f))
                                {
                                    loc = distance + f * tryLane.getLength().si;
                                    break;
                                }
                                distance += tryLane.getLength().si;
                            }
                        }

                    }

                    if (Double.isNaN(loc))
                    {
                        // the GTU is not on the lane with the relativePosition, nor is it registered on next/previous lanes
                        // this can occur as the GTU was generated with the rear upstream of the lane, or due to rounding errors
                        // use different fraction projection fallback
                        f = lane.getCenterLine().projectFractional(null, null, p.x, p.y, FractionalFallback.ENDPOINT);
                        if (Double.isNaN(f))
                        {
                            CategoryLogger.always().error("GTU {} at location {} cannot project itself onto {}; p is {}", this,
                                    getLocation(), lane.getCenterLine(), p);
                            plan.getLocation(when, relativePosition);
                        }
                        loc = lane.getLength().si * f;

                        // if (CACHING)
                        // {
                        // this.cachedPositions.put(cacheIndex, null);
                        // }
                        // return null;
                        // if (getOdometer().lt(getLength()))
                        // {
                        // // this occurs when the GTU is generated with the rear upstream of the lane, which we often do
                        // loc = position(lane, getFront(), when).si + relativePosition.getDx().si - getFront().getDx().si;
                        // }
                        // else
                        // {
                        // System.out.println("loc is NaN");
                        // }
                    }
                }
                catch (Exception e)
                {
                    // System.err.println(toString() + ": " + e.getMessage());
                    throw new GtuException(e);
                }

                Length length = Length.instantiateSI(loc);
                if (CACHING)
                {
                    this.cachedPositions.put(length, lane, relativePosition);
                    this.cachePositionsTime = when.si;
                    this.cacheOperationalPlan = plan;
                }
                return length;
            }
        }
    }

    /**
     * Return the current Lane, position and directionality of the GTU.
     * @return LanePosition; the current Lane, position and directionality of the GTU
     * @throws GtuException in case the reference position of the GTU cannot be found on the lanes in its current path
     */
    @SuppressWarnings("checkstyle:designforextension")
    public LanePosition getReferencePosition() throws GtuException
    {
        synchronized (this)
        {
            if (this.referencePositionTime == getSimulator().getSimulatorAbsTime().si)
            {
                return this.cachedReferencePosition;
            }
            Lane refLane = null;
            for (CrossSection crossSection : this.crossSections)
            {
                Lane lane = crossSection.getLanes().get(this.referenceLaneIndex);
                double fraction = fractionalPosition(lane, getReference());
                if (fraction >= 0.0 && fraction <= 1.0)
                {
                    refLane = lane;
                    break;
                }
            }
            if (refLane != null)
            {
                this.cachedReferencePosition = new LanePosition(refLane, position(refLane, getReference()));
                this.referencePositionTime = getSimulator().getSimulatorAbsTime().si;
                return this.cachedReferencePosition;
            }
            CategoryLogger.always().error("The reference point of GTU {} is not on any of the lanes on which it is registered",
                    this);
            for (CrossSection crossSection : this.crossSections)
            {
                Lane lane = crossSection.getLanes().get(this.referenceLaneIndex);
                double fraction = fractionalPosition(lane, getReference());
                CategoryLogger.always().error("\tGTU is on lane \"{}\" at fraction {}", lane, fraction);
            }
            throw new GtuException(
                    "The reference point of GTU " + this + " is not on any of the lanes on which it is registered");
        }
    }

    /**
     * Sets a vehicle model.
     * @param vehicleModel VehicleModel; vehicle model
     */
    public void setVehicleModel(final VehicleModel vehicleModel)
    {
        this.vehicleModel = vehicleModel;
    }

    /**
     * Returns the vehicle model.
     * @return VehicleModel; vehicle model
     */
    public VehicleModel getVehicleModel()
    {
        return this.vehicleModel;
    }

    /** {@inheritDoc} */
    @Override
    @SuppressWarnings("checkstyle:designforextension")
    public void destroy()
    {
        LanePosition dlp = null;
        try
        {
            dlp = getReferencePosition();
        }
        catch (GtuException e)
        {
            // ignore. not important at destroy
        }
        OrientedPoint2d location = this.getOperationalPlan() == null ? new OrientedPoint2d(0.0, 0.0, 0.0) : getLocation();
        synchronized (this.lock)
        {
            for (CrossSection crossSection : this.crossSections)
            {
                boolean removeFromParentLink = true;
                for (Lane lane : crossSection.getLanes())
                {
                    Length position;
                    try
                    {
                        position = position(lane, getReference());
                    }
                    catch (GtuException exception)
                    {
                        // TODO: hard remove over whole network
                        // TODO: logger notification
                        throw new RuntimeException(exception);
                    }
                    lane.removeGtu(this, removeFromParentLink, position);
                    removeFromParentLink = false;
                }
            }
        }
        if (dlp != null && dlp.getLane() != null)
        {
            Lane referenceLane = dlp.getLane();
            fireTimedEvent(LaneBasedGtu.LANEBASED_DESTROY_EVENT,
                    new Object[] {getId(), new PositionVector(new double[] {location.x, location.y}, PositionUnit.METER),
                            new Direction(location.getDirZ(), DirectionUnit.EAST_RADIAN), getOdometer(),
                            referenceLane.getLink().getId(), referenceLane.getId(), dlp.getPosition()},
                    getSimulator().getSimulatorTime());
        }
        else
        {
            fireTimedEvent(LaneBasedGtu.LANEBASED_DESTROY_EVENT,
                    new Object[] {getId(), new PositionVector(new double[] {location.x, location.y}, PositionUnit.METER),
                            new Direction(location.getDirZ(), DirectionUnit.EAST_RADIAN), getOdometer(), null, null, null},
                    getSimulator().getSimulatorTime());
        }
        cancelAllEvents();

        super.destroy();
    }

    /** @return the road network to which the LaneBasedGtu belongs */
    public RoadNetwork getNetwork()
    {
        return (RoadNetwork) super.getPerceivableContext();
    }

    /**
     * This method returns the current desired speed of the GTU. This value is required often, so implementations can cache it.
     * @return Speed; current desired speed
     */
    public Speed getDesiredSpeed()
    {
        synchronized (this)
        {
            Time simTime = getSimulator().getSimulatorAbsTime();
            if (this.desiredSpeedTime == null || this.desiredSpeedTime.si < simTime.si)
            {
                InfrastructurePerception infra =
                        getTacticalPlanner().getPerception().getPerceptionCategoryOrNull(InfrastructurePerception.class);
                SpeedLimitInfo speedInfo;
                if (infra == null)
                {
                    speedInfo = new SpeedLimitInfo();
                    speedInfo.addSpeedInfo(SpeedLimitTypes.MAX_VEHICLE_SPEED, getMaximumSpeed());
                }
                else
                {
                    // Throw.whenNull(infra, "InfrastructurePerception is required to determine the desired speed.");
                    speedInfo = infra.getSpeedLimitProspect(RelativeLane.CURRENT).getSpeedLimitInfo(Length.ZERO);
                }
                this.cachedDesiredSpeed =
                        Try.assign(() -> getTacticalPlanner().getCarFollowingModel().desiredSpeed(getParameters(), speedInfo),
                                "Parameter exception while obtaining the desired speed.");
                this.desiredSpeedTime = simTime;
            }
            return this.cachedDesiredSpeed;
        }
    }

    /**
     * This method returns the current car-following acceleration of the GTU. This value is required often, so implementations
     * can cache it.
     * @return Acceleration; current car-following acceleration
     */
    public Acceleration getCarFollowingAcceleration()
    {
        synchronized (this)
        {
            Time simTime = getSimulator().getSimulatorAbsTime();
            if (this.carFollowingAccelerationTime == null || this.carFollowingAccelerationTime.si < simTime.si)
            {
                LanePerception perception = getTacticalPlanner().getPerception();
                // speed
                EgoPerception<?, ?> ego = perception.getPerceptionCategoryOrNull(EgoPerception.class);
                Throw.whenNull(ego, "EgoPerception is required to determine the speed.");
                Speed speed = ego.getSpeed();
                // speed limit info
                InfrastructurePerception infra = perception.getPerceptionCategoryOrNull(InfrastructurePerception.class);
                Throw.whenNull(infra, "InfrastructurePerception is required to determine the desired speed.");
                SpeedLimitInfo speedInfo = infra.getSpeedLimitProspect(RelativeLane.CURRENT).getSpeedLimitInfo(Length.ZERO);
                // leaders
                NeighborsPerception neighbors = perception.getPerceptionCategoryOrNull(NeighborsPerception.class);
                Throw.whenNull(neighbors, "NeighborsPerception is required to determine the car-following acceleration.");
                PerceptionCollectable<HeadwayGtu, LaneBasedGtu> leaders = neighbors.getLeaders(RelativeLane.CURRENT);
                // obtain
                this.cachedCarFollowingAcceleration =
                        Try.assign(() -> getTacticalPlanner().getCarFollowingModel().followingAcceleration(getParameters(),
                                speed, speedInfo, leaders), "Parameter exception while obtaining the desired speed.");
                this.carFollowingAccelerationTime = simTime;
            }
            return this.cachedCarFollowingAcceleration;
        }
    }

    /**
     * Returns the lateral position of the GTU relative to the lane center line. Negative values are towards the right.
     * @param lane Lane; lane to consider (most important regarding left/right, not upstream downstream)
     * @return Length; lateral position of the GTU relative to the lane center line
     * @throws GtuException when the vehicle is not on the given lane.
     */
    public Length getLateralPosition(final Lane lane) throws GtuException
    {
        OperationalPlan plan = getOperationalPlan();
        if (plan instanceof LaneBasedOperationalPlan && !((LaneBasedOperationalPlan) plan).isDeviative())
        {
            return Length.ZERO;
        }
        LanePosition ref = getReferencePosition();
        int latIndex = -1;
        int longIndex = -1;
        for (int i = 0; i < this.crossSections.size(); i++)
        {
            List<Lane> lanes = this.crossSections.get(i).getLanes();
            if (lanes.contains(lane))
            {
                latIndex = lanes.indexOf(lane);
            }
            if (lanes.contains(ref.getLane()))
            {
                longIndex = i;
            }
        }
        Throw.when(latIndex == -1 || longIndex == -1, GtuException.class, "GTU %s is not on %s", getId(), lane);
        Lane refCrossSectionLane = this.crossSections.get(longIndex).getLanes().get(latIndex);
        OrientedPoint2d loc = getLocation();
        double f = refCrossSectionLane.getCenterLine().projectOrthogonal(loc.x, loc.y);
        OrientedPoint2d p = Try.assign(() -> refCrossSectionLane.getCenterLine().getLocationFraction(f), GtuException.class,
                "GTU %s is not orthogonal to the reference lane.", getId());
        double d = p.distance(loc);
        if (this.crossSections.get(0).getLanes().size() > 1)
        {
            return Length.instantiateSI(latIndex == 0 ? -d : d);
        }
        double x2 = p.x + Math.cos(p.getDirZ());
        double y2 = p.y + Math.sin(p.getDirZ());
        double det = (loc.x - p.x) * (y2 - p.y) - (loc.y - p.y) * (x2 - p.x);
        return Length.instantiateSI(det < 0.0 ? -d : d);
    }

    /**
     * Sets whether the GTU perform lane changes instantaneously or not.
     * @param instantaneous boolean; whether the GTU perform lane changes instantaneously or not
     */
    public void setInstantaneousLaneChange(final boolean instantaneous)
    {
        this.instantaneousLaneChange = instantaneous;
    }

    /**
     * Returns whether the GTU perform lane changes instantaneously or not.
     * @return boolean; whether the GTU perform lane changes instantaneously or not
     */
    public boolean isInstantaneousLaneChange()
    {
        return this.instantaneousLaneChange;
    }

    /** {@inheritDoc} */
    @Override
    public LaneBasedTacticalPlanner getTacticalPlanner()
    {
        return getStrategicalPlanner().getTacticalPlanner();
    }

    /** {@inheritDoc} */
    @Override
    public LaneBasedTacticalPlanner getTacticalPlanner(final Time time)
    {
        return getStrategicalPlanner(time).getTacticalPlanner(time);
    }

    /**
     * The default implementation returns {@code true} if the deceleration is larger than a speed-dependent threshold given
     * by:<br>
     * <br>
     * c0 * g(v) + c1 + c3*v^2<br>
     * <br>
     * where c0 = 0.2, c1 = 0.15 and c3 = 0.00025 (with c2 = 0 implicit) are empirically derived averages, and g(v) is 0 below
     * 25 km/h or 1 otherwise, representing that the engine is disengaged at low speeds.
     * @return boolean; whether the braking lights are on
     */
    public boolean isBrakingLightsOn()
    {
        return isBrakingLightsOn(getSimulator().getSimulatorAbsTime());
    }

    /**
     * The default implementation returns {@code true} if the deceleration is larger than a speed-dependent threshold given
     * by:<br>
     * <br>
     * c0 * g(v) + c1 + c3*v^2<br>
     * <br>
     * where c0 = 0.2, c1 = 0.15 and c3 = 0.00025 (with c2 = 0 implicit) are empirically derived averages, and g(v) is 0 below
     * 25 km/h or 1 otherwise, representing that the engine is disengaged at low speeds.
     * @param when Time; time
     * @return boolean; whether the braking lights are on
     */
    public boolean isBrakingLightsOn(final Time when)
    {
        double v = getSpeed(when).si;
        double a = getAcceleration(when).si;
        return a < (v < 6.944 ? 0.0 : -0.2) - 0.15 - 0.00025 * v * v;
    }

    /**
     * Get projected length on the lane.
     * @param lane Lane; lane to project the vehicle on
     * @return Length; the length on the lane, which is different from the actual length during deviative tactical plans
     * @throws GtuException when the vehicle is not on the given lane
     */
    public Length getProjectedLength(final Lane lane) throws GtuException
    {
        Length front = position(lane, getFront());
        Length rear = position(lane, getRear());
        return front.minus(rear);
    }

    /** {@inheritDoc} */
    @Override
    @SuppressWarnings("checkstyle:designforextension")
    public String toString()
    {
        return String.format("GTU " + getId());
    }

    /** Cross section of lanes. */
    private static class CrossSection
    {

        /** Lanes. */
        private final List<Lane> lanes;

        /**
         * @param lanes List&lt;Lane&gt;; lanes
         */
        protected CrossSection(final List<Lane> lanes)
        {
            this.lanes = lanes;
        }

        /**
         * @return lanes.
         */
        protected List<Lane> getLanes()
        {
            return this.lanes;
        }

    }

    /**
     * The lane-based event type for pub/sub indicating a move. <br>
     * Payload: [String gtuId, PositionVector currentPosition, Direction currentDirection, Speed speed, Acceleration
     * acceleration, TurnIndicatorStatus turnIndicatorStatus, Length odometer, Link id of referenceLane, Lane id of
     * referenceLane, Length positionOnReferenceLane]
     */
    public static EventType LANEBASED_MOVE_EVENT = new EventType("LANEBASEDGTU.MOVE", new MetaData("Lane based GTU moved",
            "Lane based GTU moved",
            new ObjectDescriptor[] {new ObjectDescriptor("GTU id", "GTU id", String.class),
                    new ObjectDescriptor("Position", "Position", PositionVector.class),
                    new ObjectDescriptor("Direction", "Direction", Direction.class),
                    new ObjectDescriptor("Speed", "Speed", Speed.class),
                    new ObjectDescriptor("Acceleration", "Acceleration", Acceleration.class),
                    new ObjectDescriptor("TurnIndicatorStatus", "Turn indicator status", String.class),
                    new ObjectDescriptor("Odometer", "Odometer value", Length.class),
                    new ObjectDescriptor("Link id", "Link id", String.class),
                    new ObjectDescriptor("Lane id", "Lane id", String.class),
                    new ObjectDescriptor("Longitudinal position on lane", "Longitudinal position on lane", Length.class)}));

    /**
     * The lane-based event type for pub/sub indicating destruction of the GTU. <br>
     * Payload: [String gtuId, PositionVector finalPosition, Direction finalDirection, Length finalOdometer, Link referenceLink,
     * Lane referenceLane, Length positionOnReferenceLane]
     */
    public static EventType LANEBASED_DESTROY_EVENT = new EventType("LANEBASEDGTU.DESTROY", new MetaData(
            "Lane based GTU destroyed", "Lane based GTU destroyed",
            new ObjectDescriptor[] {new ObjectDescriptor("GTU id", "GTU id", String.class),
                    new ObjectDescriptor("Position", "Position", PositionVector.class),
                    new ObjectDescriptor("Direction", "Direction", Direction.class),
                    new ObjectDescriptor("Odometer", "Odometer value", Length.class),
                    new ObjectDescriptor("Link id", "Link id", String.class),
                    new ObjectDescriptor("Lane id", "Lane id", String.class),
                    new ObjectDescriptor("Longitudinal position on lane", "Longitudinal position on lane", Length.class)}));

    // TODO: the next 2 events are never fired...
    /**
     * The event type for pub/sub indicating that the GTU entered a new lane (with the FRONT position if driving forward; REAR
     * if driving backward). <br>
     * Payload: [String gtuId, String link id, String lane id]
     */
    public static EventType LANE_ENTER_EVENT = new EventType("LANE.ENTER",
            new MetaData("Lane based GTU entered lane", "Front of lane based GTU entered lane",
                    new ObjectDescriptor[] {new ObjectDescriptor("GTU id", "GTU id", String.class),
                            new ObjectDescriptor("Link id", "Link id", String.class),
                            new ObjectDescriptor("Lane id", "Lane id", String.class)}));

    /**
     * The event type for pub/sub indicating that the GTU exited a lane (with the REAR position if driving forward; FRONT if
     * driving backward). <br>
     * Payload: [String gtuId, String link id, String lane id]
     */
    public static EventType LANE_EXIT_EVENT = new EventType("LANE.EXIT",
            new MetaData("Lane based GTU exited lane", "Rear of lane based GTU exited lane",
                    new ObjectDescriptor[] {new ObjectDescriptor("GTU id", "GTU id", String.class),
                            new ObjectDescriptor("Link id", "Link id", String.class),
                            new ObjectDescriptor("Lane id", "Lane id", String.class)}));

    /**
     * The event type for pub/sub indicating that the GTU change lane. <br>
     * Payload: [String gtuId, LateralDirectionality direction, String fromLaneId, Length position]
     */
    public static EventType LANE_CHANGE_EVENT = new EventType("LANE.CHANGE",
            new MetaData("Lane based GTU changes lane", "Lane based GTU changes lane",
                    new ObjectDescriptor[] {new ObjectDescriptor("GTU id", "GTU id", String.class),
                            new ObjectDescriptor("Lateral direction of lane change", "Lateral direction of lane change",
                                    String.class),
                            new ObjectDescriptor("Link id", "Link id", String.class),
                            new ObjectDescriptor("Lane id of vacated lane", "Lane id of vacated lane", String.class),
                            new ObjectDescriptor("Position along vacated lane", "Position along vacated lane", Length.class)}));

}
