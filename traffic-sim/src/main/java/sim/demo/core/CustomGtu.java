package sim.demo.core;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.djunits.unit.DirectionUnit;
import org.djunits.unit.DurationUnit;
import org.djunits.unit.PositionUnit;
import org.djunits.unit.TimeUnit;
import org.djunits.value.vdouble.scalar.Acceleration;
import org.djunits.value.vdouble.scalar.Direction;
import org.djunits.value.vdouble.scalar.Duration;
import org.djunits.value.vdouble.scalar.Length;
import org.djunits.value.vdouble.scalar.Speed;
import org.djunits.value.vdouble.scalar.Time;
import org.djunits.value.vdouble.vector.PositionVector;
import org.djutils.base.Identifiable;
import org.djutils.draw.line.Polygon2d;
import org.djutils.draw.point.OrientedPoint2d;
import org.djutils.draw.point.Point2d;
import org.djutils.event.EventType;
import org.djutils.event.LocalEventProducer;
import org.djutils.exceptions.Throw;
import org.djutils.exceptions.Try;
import org.djutils.immutablecollections.Immutable;
import org.djutils.immutablecollections.ImmutableHashSet;
import org.djutils.immutablecollections.ImmutableLinkedHashMap;
import org.djutils.immutablecollections.ImmutableMap;
import org.djutils.immutablecollections.ImmutableSet;
import org.djutils.metadata.MetaData;
import org.djutils.metadata.ObjectDescriptor;
import org.opentrafficsim.base.HierarchicallyTyped;
import org.opentrafficsim.base.geometry.BoundingRectangle;
import org.opentrafficsim.base.geometry.OtsBounds2d;
import org.opentrafficsim.base.geometry.OtsLocatable;
import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.base.parameters.ParameterTypes;
import org.opentrafficsim.base.parameters.Parameters;
import org.opentrafficsim.core.DynamicSpatialObject;
import org.opentrafficsim.core.animation.Drawable;
import org.opentrafficsim.core.dsol.OtsSimulatorInterface;
import org.opentrafficsim.core.geometry.OtsGeometryException;
import org.opentrafficsim.core.geometry.OtsLine2d;
import org.opentrafficsim.core.gtu.Gtu;
import org.opentrafficsim.core.gtu.GtuErrorHandler;
import org.opentrafficsim.core.gtu.GtuException;
import org.opentrafficsim.core.gtu.GtuType;
import org.opentrafficsim.core.gtu.RelativePosition;
import org.opentrafficsim.core.gtu.RelativePosition.TYPE;
import org.opentrafficsim.core.gtu.plan.operational.OperationalPlan;
import org.opentrafficsim.core.gtu.plan.operational.OperationalPlanException;
import org.opentrafficsim.core.gtu.plan.strategical.StrategicalPlanner;
import org.opentrafficsim.core.gtu.plan.tactical.TacticalPlanner;
import org.opentrafficsim.core.network.NetworkException;
import org.opentrafficsim.core.perception.Historical;
import org.opentrafficsim.core.perception.HistoricalValue;
import org.opentrafficsim.core.perception.HistoryManager;
import org.opentrafficsim.core.perception.PerceivableContext;

import nl.tudelft.simulation.dsol.SimRuntimeException;
import nl.tudelft.simulation.dsol.formalisms.eventscheduling.SimEvent;

/**
 * Implements the basic functionalities of any GTU: the ability to move on 3D-space according to a plan.
 * <p>
 * Copyright (c) 2013-2024 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved. <br>
 * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
 * </p>
 * @author <a href="https://github.com/averbraeck">Alexander Verbraeck</a>
 * @author <a href="https://tudelft.nl/staff/p.knoppers-1">Peter Knoppers</a>
 */
public class CustomGtu extends Gtu
{
    /** */
    private static final long serialVersionUID = 20140822L;

    /** The id of the GTU. */
    private final String id;

    /** unique number of the GTU. */
    private final int uniqueNumber;

    /** the unique number counter. */
    private static int staticUNIQUENUMBER = 0;

    /** The type of GTU, e.g. TruckType, CarType, BusType. */
    private final GtuType gtuType;

    /** The simulator to schedule activities on. */
    private final OtsSimulatorInterface simulator;

    /** Model parameters. */
    private Parameters parameters;

    /** The maximum acceleration. */
    private Acceleration maximumAcceleration;

    /** The maximum deceleration, stored as a negative number. */
    private Acceleration maximumDeceleration;

    /**
     * The odometer which measures how much distance have we covered between instantiation and the last completed operational
     * plan. In order to get a complete odometer reading, the progress of the current plan execution has to be added to this
     * value.
     */
    public Historical<Length> odometer;

    /** The strategical planner that can instantiate tactical planners to determine mid-term decisions. */
    public final Historical<StrategicalPlanner> strategicalPlanner;

    /** The tactical planner that can generate an operational plan. */
    public final Historical<TacticalPlanner<?, ?>> tacticalPlanner;

    /** The current operational plan, which provides a short-term movement over time. */
    public final Historical<OperationalPlan> operationalPlan;

    /** The next move event as scheduled on the simulator, can be used for interrupting the current move. */
    public SimEvent<Duration> nextMoveEvent;

    /** The model in which this GTU is registered. */
    public PerceivableContext perceivableContext;

    /** Is this GTU destroyed? */
    private boolean destroyed = false;

    /** aligned or not. */
    // TODO: should be indicated with a Parameter
    public static boolean ALIGNED = true;

    /** aligned schedule count. */
    // TODO: can be removed after testing period
    public static int ALIGN_COUNT = 0;

    /** Cached speed time. */
    private double cachedSpeedTime = Double.NaN;

    /** Cached speed. */
    private Speed cachedSpeed = null;

    /** Cached acceleration time. */
    private double cachedAccelerationTime = Double.NaN;

    /** Cached acceleration. */
    private Acceleration cachedAcceleration = null;

    /** Parent GTU. */
    private Gtu parent = null;

    /** Children GTU's. */
    private Set<Gtu> children = new LinkedHashSet<>();

    /** Error handler. */
    public GtuErrorHandler errorHandler = GtuErrorHandler.THROW;

    /** shape of the Gtu contour. */
    private Polygon2d shape = null;

    /** Sensing positions. */
    private final Map<RelativePosition.TYPE, RelativePosition> relativePositions = new LinkedHashMap<>();

    /** cached front. */
    private final RelativePosition frontPos;

    /** cached rear. */
    private final RelativePosition rearPos;

    /** contour points. */
    private final Set<RelativePosition> contourPoints = new LinkedHashSet<>();

    /** The maximum length of the GTU (parallel with driving direction). */
    private final Length length;

    /** The maximum width of the GTU (perpendicular to driving direction). */
    private final Length width;

    /** The maximum speed of the GTU (in the driving direction). */
    private final Speed maximumSpeed;

    /** Tags of the GTU, these are used for specific use cases of any sort. */
    private final Map<String, String> tags = new LinkedHashMap<>();

    /** Bounds. */
    private OtsBounds2d bounds;

    /**
     * @param id String; the id of the GTU
     * @param gtuType GtuType; the type of GTU, e.g. TruckType, CarType, BusType
     * @param simulator OtsSimulatorInterface; the simulator to schedule plan changes on
     * @param perceivableContext PerceivableContext; the perceivable context in which this GTU will be registered
     * @param length Length; the maximum length of the GTU (parallel with driving direction)
     * @param width Length; the maximum width of the GTU (perpendicular to driving direction)
     * @param maximumSpeed Speed;the maximum speed of the GTU (in the driving direction)
     * @param front Length; front distance relative to the reference position
     * @param centerOfGravity Length; distance from the center of gravity to the reference position
     * @throws GtuException when the preconditions of the constructor are not met
     */
    @SuppressWarnings("checkstyle:parameternumber")
    public CustomGtu(final String id, final GtuType gtuType, final OtsSimulatorInterface simulator,
            final PerceivableContext perceivableContext, final Length length, final Length width, final Speed maximumSpeed,
            final Length front, final Length centerOfGravity) throws GtuException
    {
    	super(id, gtuType, simulator, perceivableContext, length, width, maximumSpeed, front, centerOfGravity);
        Throw.when(id == null, GtuException.class, "id is null");
        Throw.when(gtuType == null, GtuException.class, "gtuType is null");
        Throw.when(perceivableContext == null, GtuException.class, "perceivableContext is null for GTU with id %s", id);
        Throw.when(perceivableContext.containsGtuId(id), GtuException.class,
                "GTU with id %s already registered in perceivableContext %s", id, perceivableContext.getId());
        Throw.when(simulator == null, GtuException.class, "simulator is null for GTU with id %s", id);

        this.length = length;
        this.width = width;
        if (null == maximumSpeed)
        {
            throw new GtuException("maximumSpeed may not be null");
        }
        this.maximumSpeed = maximumSpeed;

        HistoryManager historyManager = simulator.getReplication().getHistoryManager(simulator);
        this.id = id;
        this.uniqueNumber = ++staticUNIQUENUMBER;
        this.gtuType = gtuType;
        this.simulator = simulator;
        this.odometer = new HistoricalValue<>(historyManager, Length.ZERO);
        this.perceivableContext = perceivableContext;
        this.perceivableContext.addGTU(this);
        this.strategicalPlanner = new HistoricalValue<>(historyManager);
        this.tacticalPlanner = new HistoricalValue<>(historyManager, null);
        this.operationalPlan = new HistoricalValue<>(historyManager, null);

        // sensor positions.
        Length dy2 = width.times(0.5);
        this.frontPos = new RelativePosition(front, Length.ZERO, Length.ZERO, RelativePosition.FRONT);
        this.relativePositions.put(RelativePosition.FRONT, this.frontPos);
        this.rearPos = new RelativePosition(front.minus(length), Length.ZERO, Length.ZERO, RelativePosition.REAR);
        this.relativePositions.put(RelativePosition.REAR, this.rearPos);
        this.relativePositions.put(RelativePosition.REFERENCE, RelativePosition.REFERENCE_POSITION);
        this.relativePositions.put(RelativePosition.CENTER,
                new RelativePosition(Length.ZERO, Length.ZERO, Length.ZERO, RelativePosition.CENTER));
        this.bounds = new BoundingRectangle(getRear().getDx().si, getFront().getDx().si, -dy2.si, dy2.si);

        // Contour positions. For now, a rectangle with the four corners.
        for (int i = -1; i <= 1; i += 2)
        {
            Length x = i < 0 ? front.minus(length) : front;
            for (int j = -1; j <= 1; j += 2)
            {
                this.contourPoints.add(new RelativePosition(x, dy2.times(j), Length.ZERO, RelativePosition.CONTOUR));
            }
        }
    }

    /**
     * Initialize the GTU at a location and speed, and give it a mission to fulfill through the strategical planner.
     * @param strategicalPlanner StrategicalPlanner; the strategical planner responsible for the overall 'mission' of the GTU,
     *            usually indicating where it needs to go. It operates by instantiating tactical planners to do the work.
     * @param initialLocation OrientedPoint2d; the initial location (and direction) of the GTU
     * @param initialSpeed Speed; the initial speed of the GTU
     * @throws SimRuntimeException when scheduling after the first move fails
     * @throws GtuException when the preconditions of the parameters are not met or when the construction of the original
     *             waiting path fails
     */
    @SuppressWarnings({"checkstyle:hiddenfield", "hiding", "checkstyle:designforextension"})
    public void init(final StrategicalPlanner strategicalPlanner, final OrientedPoint2d initialLocation,
            final Speed initialSpeed) throws SimRuntimeException, GtuException
    {
        Throw.when(strategicalPlanner == null, GtuException.class, "strategicalPlanner is null for GTU with id %s", this.id);
        Throw.whenNull(initialLocation, "Initial location of GTU cannot be null");
        Throw.when(Double.isNaN(initialLocation.x) || Double.isNaN(initialLocation.y), GtuException.class,
                "initialLocation %s invalid for GTU with id %s", initialLocation, this.id);
        Throw.when(initialSpeed == null, GtuException.class, "initialSpeed is null for GTU with id %s", this.id);
        Throw.when(!getId().equals(strategicalPlanner.getGtu().getId()), GtuException.class,
                "GTU %s is initialized with a strategical planner for GTU %s", getId(), strategicalPlanner.getGtu().getId());

        this.strategicalPlanner.set(strategicalPlanner);
        this.tacticalPlanner.set(strategicalPlanner.getTacticalPlanner());

        try
        {
            move(initialLocation);
        }
        catch (OperationalPlanException | NetworkException | ParameterException exception)
        {
            throw new GtuException("Failed to create OperationalPlan for GTU " + this.id, exception);
        }
    }

    /**
     * Destructor. Don't forget to call with super.destroy() from any override to avoid memory leaks in the network.
     */
    @SuppressWarnings("checkstyle:designforextension")
    public void destroy()
    {
        OrientedPoint2d location = getLocation();
        fireTimedEvent(Gtu.DESTROY_EVENT,
                new Object[] {getId(), new PositionVector(new double[] {location.x, location.y}, PositionUnit.METER),
                        new Direction(location.getDirZ(), DirectionUnit.EAST_RADIAN), getOdometer()},
                this.simulator.getSimulatorTime());

        // cancel the next move
        if (this.nextMoveEvent != null)
        {
            this.simulator.cancelEvent(this.nextMoveEvent);
            this.nextMoveEvent = null;
        }

        this.perceivableContext.removeGTU(this);
        this.destroyed = true;
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
    protected boolean move(final OrientedPoint2d fromLocation)
            throws SimRuntimeException, OperationalPlanException, GtuException, NetworkException, ParameterException
    {
        try
        {
            Time now = this.simulator.getSimulatorAbsTime();

            // Add the odometer distance from the currently running operational plan.
            // Because a plan can be interrupted, we explicitly calculate the covered distance till 'now'
            Length currentOdometer;
            if (this.operationalPlan.get() != null)
            {
                currentOdometer = this.odometer.get().plus(this.operationalPlan.get().getTraveledDistance(now));
            }
            else
            {
                currentOdometer = this.odometer.get();
            }

            // Do we have an operational plan?
            // TODO discuss when a new tactical planner may be needed
            TacticalPlanner<?, ?> tactPlanner = this.tacticalPlanner.get();
            if (tactPlanner == null)
            {
                // Tell the strategical planner to provide a tactical planner
                tactPlanner = this.strategicalPlanner.get().getTacticalPlanner();
                this.tacticalPlanner.set(tactPlanner);
            }
            synchronized (this)
            {
                tactPlanner.getPerception().perceive();
            }
            OperationalPlan newOperationalPlan = tactPlanner.generateOperationalPlan(now, fromLocation);
            synchronized (this)
            {
                this.operationalPlan.set(newOperationalPlan);
                this.cachedSpeedTime = Double.NaN;
                this.cachedAccelerationTime = Double.NaN;
                this.odometer.set(currentOdometer);
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
                this.nextMoveEvent =
                        new SimEvent<Duration>(new Duration(tNext - getSimulator().getStartTimeAbs().si, DurationUnit.SI), this,
                                "move", new Object[] {p});
                ALIGN_COUNT++;
            }
            else
            {
                // schedule the next move at the end of the current operational plan
                // store the event, so it can be cancelled in case the plan has to be interrupted and changed halfway
                this.nextMoveEvent =
                        new SimEvent<>(now.plus(newOperationalPlan.getTotalDuration()).minus(getSimulator().getStartTimeAbs()),
                                this, "move", new Object[] {newOperationalPlan.getEndLocation()});
            }
            this.getSimulator().scheduleEvent(this.nextMoveEvent);
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
                this.errorHandler.handle(this, ex);
            }
            catch (Exception exception)
            {
                throw new GtuException(exception);
            }
            return true;
        }
    }

    /**
     * Interrupt the move and ask for a new plan. This method can be overridden to carry out the bookkeeping needed when the
     * current plan gets interrupted.
     * @throws OperationalPlanException when there was a problem retrieving the location from the running plan
     * @throws SimRuntimeException when scheduling of the next move fails
     * @throws OperationalPlanException when there is a problem creating a good path for the GTU
     * @throws GtuException when there is a problem with the state of the GTU when planning a path
     * @throws NetworkException in case of a problem with the network, e.g., unreachability of a certain point
     * @throws ParameterException when there is a problem with a parameter
     */
    @SuppressWarnings("checkstyle:designforextension")
    protected void interruptMove()
            throws SimRuntimeException, OperationalPlanException, GtuException, NetworkException, ParameterException
    {
        this.simulator.cancelEvent(this.nextMoveEvent);
        move(this.operationalPlan.get().getLocation(this.simulator.getSimulatorAbsTime()));
    }

    /**
     * Sets a tag, these are used for specific use cases of any sort.
     * @param tag String; name of the tag.
     * @param value String; value of the tag.
     */
    public void setTag(final String tag, final String value)
    {
        this.tags.put(tag, value);
    }

    /**
     * Returns the value for the given tag, these are used for specific use cases of any sort.
     * @param tag String; name of the tag.
     * @return String; value of the tag, or {@code null} if it is not given to the GTU.
     */
    public String getTag(final String tag)
    {
        return this.tags.get(tag);
    }

    /** {@inheritDoc} */
    @SuppressWarnings("checkstyle:designforextension")
    @Override
    public GtuType getType()
    {
        return this.gtuType;
    }

    /**
     * @return StrategicalPlanner; the planner responsible for the overall 'mission' of the GTU, usually indicating where it
     *         needs to go. It operates by instantiating tactical planners to do the work.
     */
    public StrategicalPlanner getStrategicalPlanner()
    {
        return this.strategicalPlanner.get();
    }

    /**
     * @param time Time; time to obtain the strategical planner at
     * @return StrategicalPlanner; the planner responsible for the overall 'mission' of the GTU, usually indicating where it
     *         needs to go. It operates by instantiating tactical planners to do the work.
     */
    public StrategicalPlanner getStrategicalPlanner(final Time time)
    {
        return this.strategicalPlanner.get(time);
    }

    /** @return TacticalPlanner; the current tactical planner that can generate an operational plan */
    public TacticalPlanner<?, ?> getTacticalPlanner()
    {
        return getStrategicalPlanner().getTacticalPlanner();
    }

    /**
     * @param time Time; time to obtain the tactical planner at
     * @return TacticalPlanner; the tactical planner that can generate an operational plan at the given time
     */
    public TacticalPlanner<?, ?> getTacticalPlanner(final Time time)
    {
        return getStrategicalPlanner(time).getTacticalPlanner(time);
    }

    /**
     * Set the operational plan. This method is for sub classes.
     * @param operationalPlan OperationalPlan; operational plan.
     */
    protected void setOperationalPlan(final OperationalPlan operationalPlan)
    {
        this.operationalPlan.set(operationalPlan);
    }

    /** Cache location time. */
    private Time cacheLocationTime = new Time(Double.NaN, TimeUnit.DEFAULT);

    /** Cached location at that time. */
    private OrientedPoint2d cacheLocation = null;

    /** {@inheritDoc} */
    @Override
    @SuppressWarnings("checkstyle:designforextension")
    public OrientedPoint2d getLocation()
    {
        synchronized (this)
        {
            if (this.operationalPlan.get() == null)
            {
                this.simulator.getLogger().always()
                        .error("No operational plan for GTU " + this.id + " at t=" + this.getSimulator().getSimulatorTime());
                return new OrientedPoint2d(0, 0, 0); // Do not cache it
            }
            try
            {
                // cache
                Time locationTime = this.simulator.getSimulatorAbsTime();
                if (null == this.cacheLocationTime || this.cacheLocationTime.si != locationTime.si)
                {
                    this.cacheLocationTime = null;
                    this.cacheLocation = this.operationalPlan.get().getLocation(locationTime);
                    this.cacheLocationTime = locationTime;
                }
                return this.cacheLocation;
            }
            catch (OperationalPlanException exception)
            {
                return new OrientedPoint2d(0, 0, 0);
            }
        }
    }

    /**
     * Return the shape of a dynamic object at time 'time'. Note that the getShape() method without a time returns the Minkowski
     * sum of all shapes of the spatial object for a validity time window, e.g., a contour that describes all locations of a GTU
     * for the next time step, i.e., the contour of the GTU belonging to the next operational plan.
     * @param time Time; the time for which we want the shape
     * @return OtsShape; the shape of the object at time 'time'
     */
    @Override
    public Polygon2d getShape(final Time time)
    {
        try
        {
            if (this.shape == null)
            {
                double w = getWidth().si;
                double l = getLength().si;
                this.shape = new Polygon2d(new Point2d(-0.5 * l, -0.5 * w), new Point2d(-0.5 * l, 0.5 * w),
                        new Point2d(0.5 * l, 0.5 * w), new Point2d(0.5 * l, -0.5 * w));
            }
            Polygon2d s = transformShape(this.shape, this.operationalPlan.get(time).getLocation(time));
            System.out.println("gtu " + getId() + ", shape(t)=" + s);
            return s;
        }
        catch (OtsGeometryException | OperationalPlanException exception)
        {
            throw new RuntimeException(exception);
        }
    }

    /**
     * Return the shape of the GTU for the validity time of the operational plan. Note that this method without a time returns
     * the Minkowski sum of all shapes of the spatial object for a validity time window, e.g., a contour that describes all
     * locations of a GTU for the next time step, i.e., the contour of the GTU belonging to the next operational plan.
     * @return OtsShape; the shape of the object over the validity of the operational plan
     */
    @Override
    public Polygon2d getShape()
    {
        try
        {
            // TODO: the actual contour of the GTU has to be moved over the path
            OtsLine2d path = this.operationalPlan.get().getPath();
            // part of the Gtu length has to be added before the start and after the end of the path.
            // we assume the reference point is within the contour of the Gtu.
            double rear = Math.max(0.0, getReference().getDx().si - getRear().getDx().si);
            double front = path.getLength().si + Math.max(0.0, getFront().getDx().si - getReference().getDx().si);
            Point2d p0 = path.getLocationExtendedSI(-rear);
            Point2d pn = path.getLocationExtendedSI(front);
            List<Point2d> pList = new ArrayList<>(Arrays.asList(path.getPoints()));
            pList.add(0, p0);
            pList.add(pn);
            OtsLine2d extendedPath = new OtsLine2d(pList);
            List<Point2d> swath = new ArrayList<>();
            swath.addAll(Arrays.asList(extendedPath.offsetLine(getWidth().si / 2.0).getPoints()));
            swath.addAll(Arrays.asList(extendedPath.offsetLine(-getWidth().si / 2.0).reverse().getPoints()));
            Polygon2d s = new Polygon2d(swath);
            // System.out.println("gtu " + getId() + ", w=" + getWidth() + ", path="
            // + this.operationalPlan.get().getPath().toString() + ", shape=" + s);
            return s;
        }
        catch (Exception e)
        {
            throw new RuntimeException(e);
        }
    }

    /** @return the context to which the GTU belongs */
    public PerceivableContext getPerceivableContext()
    {
        return this.perceivableContext;
    }

    /**
     * Adds the provided GTU to this GTU, meaning it moves with this GTU.
     * @param gtu Gtu; gtu to enter this GTU
     * @throws GtuException if the gtu already has a parent
     */
    public void addGtu(final Gtu gtu) throws GtuException
    {
        this.children.add(gtu);
        gtu.setParent(this);
    }

    /**
     * Removes the provided GTU from this GTU, meaning it no longer moves with this GTU.
     * @param gtu Gtu; gtu to exit this GTU
     */
    public void removeGtu(final Gtu gtu)
    {
        this.children.remove(gtu);
        try
        {
            gtu.setParent(null);
        }
        catch (GtuException exception)
        {
            // cannot happen, setting null is always ok
        }
    }

    /**
     * Set the parent GTU.
     * @param gtu Gtu; parent GTU, may be {@code null}
     * @throws GtuException if the gtu already has a parent
     */
    public void setParent(final Gtu gtu) throws GtuException
    {
        Throw.when(gtu != null && this.parent != null, GtuException.class, "GTU %s already has a parent.", this);
        this.parent = gtu;
    }

    /**
     * Returns the parent GTU, or {@code null} if this GTU has no parent.
     * @return Gtu; parent GTU, or {@code null} if this GTU has no parent
     */
    public Gtu getParent()
    {
        return this.parent;
    }

    /**
     * Returns the children GTU's.
     * @return Set&lt;GTU&gt;; children GTU's
     */
    public Set<Gtu> getChildren()
    {
        return new LinkedHashSet<>(this.children); // safe copy
    }

    /**
     * @return errorHandler.
     */
    protected GtuErrorHandler getErrorHandler()
    {
        return this.errorHandler;
    }

    /**
     * Sets the error handler.
     * @param errorHandler GTUErrorHandler; error handler
     */
    public void setErrorHandler(final GtuErrorHandler errorHandler)
    {
        this.errorHandler = errorHandler;
    }

    /** {@inheritDoc} */
    @Override
    @SuppressWarnings("designforextension")
    public int hashCode()
    {
        final int prime = 31;
        int result = 1;
        result = prime * result + ((this.id == null) ? 0 : this.id.hashCode());
        result = prime * result + this.uniqueNumber;
        return result;
    }

    /** {@inheritDoc} */
    @Override
    @SuppressWarnings({"designforextension", "needbraces"})
    public boolean equals(final Object obj)
    {
        if (this == obj)
            return true;
        if (obj == null)
            return false;
        if (getClass() != obj.getClass())
            return false;
        CustomGtu other = (CustomGtu) obj;
        if (this.id == null)
        {
            if (other.id != null)
                return false;
        }
        else if (!this.id.equals(other.id))
            return false;
        if (this.uniqueNumber != other.uniqueNumber)
            return false;
        return true;
    }

    /**
     * The event type for pub/sub indicating a move. <br>
     * Payload: [String id, DirectedPoint position, Speed speed, Acceleration acceleration, Length odometer]
     */
    public static EventType MOVE_EVENT = new EventType("GTU.MOVE",
            new MetaData("GTU move", "GTU id, position, speed, acceleration, odometer",
                    new ObjectDescriptor[] {new ObjectDescriptor("Id", "GTU Id", String.class),
                            new ObjectDescriptor("position", "position", PositionVector.class),
                            new ObjectDescriptor("direction", "direction", Direction.class),
                            new ObjectDescriptor("speed", "speed", Speed.class),
                            new ObjectDescriptor("acceleration", "acceleration", Acceleration.class),
                            new ObjectDescriptor("Odometer", "Total distance travelled since incarnation", Length.class)}));

    /**
     * The event type for pub/sub indicating destruction of the GTU. <br>
     * Payload: [String id, DirectedPoint lastPosition, Length odometer]
     */
    public static EventType DESTROY_EVENT = new EventType("GTU.DESTROY",
            new MetaData("GTU destroy", "GTU id, final position, final odometer",
                    new ObjectDescriptor[] {new ObjectDescriptor("Id", "GTU Id", String.class),
                            new ObjectDescriptor("position", "position", PositionVector.class),
                            new ObjectDescriptor("direction", "direction", Direction.class),
                            new ObjectDescriptor("Odometer", "Total distance travelled since incarnation", Length.class)}));

}

