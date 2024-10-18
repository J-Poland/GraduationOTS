package sim.demo;

import java.net.URL;
import java.rmi.RemoteException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.Set;

import org.djunits.unit.DurationUnit;
import org.djunits.unit.FrequencyUnit;
import org.djunits.unit.LengthUnit;
import org.djunits.unit.SpeedUnit;
import org.djunits.unit.TimeUnit;
import org.djunits.value.vdouble.scalar.Acceleration;
import org.djunits.value.vdouble.scalar.Duration;
import org.djunits.value.vdouble.scalar.Frequency;
import org.djunits.value.vdouble.scalar.Length;
import org.djunits.value.vdouble.scalar.Speed;
import org.djunits.value.vdouble.scalar.Time;
import org.djutils.event.Event;
import org.djutils.event.EventListener;
import org.djutils.immutablecollections.ImmutableCollection;
import org.djutils.immutablecollections.ImmutableList;
import org.djutils.io.URLResource;
import org.djutils.logger.CategoryLogger;
import org.opentrafficsim.animation.GraphLaneUtil;
import org.opentrafficsim.animation.colorer.LmrsSwitchableColorer;
import org.opentrafficsim.animation.gtu.colorer.GtuColorer;
import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.base.parameters.ParameterSet;
import org.opentrafficsim.base.parameters.ParameterType;
import org.opentrafficsim.base.parameters.ParameterTypes;
import org.opentrafficsim.base.parameters.Parameters;
import org.opentrafficsim.core.definitions.DefaultsNl;
import org.opentrafficsim.core.distributions.ConstantGenerator;
import org.opentrafficsim.core.distributions.Distribution;
import org.opentrafficsim.core.distributions.Distribution.FrequencyAndObject;
import org.opentrafficsim.core.distributions.Generator;
import org.opentrafficsim.core.distributions.ProbabilityException;
import org.opentrafficsim.core.dsol.AbstractOtsModel;
import org.opentrafficsim.core.dsol.OtsSimulatorInterface;
import org.opentrafficsim.core.gtu.Gtu;
import org.opentrafficsim.core.gtu.GtuException;
import org.opentrafficsim.core.gtu.GtuType;
import org.opentrafficsim.core.gtu.perception.DirectEgoPerception;
import org.opentrafficsim.core.gtu.plan.operational.OperationalPlanException;
import org.opentrafficsim.core.idgenerator.IdGenerator;
import org.opentrafficsim.core.network.Link;
import org.opentrafficsim.core.network.Network;
import org.opentrafficsim.core.network.NetworkException;
import org.opentrafficsim.core.network.route.ProbabilisticRouteGenerator;
import org.opentrafficsim.core.network.route.Route;
import org.opentrafficsim.core.parameters.ParameterFactory;
import org.opentrafficsim.core.parameters.ParameterFactoryByType;
import org.opentrafficsim.core.units.distributions.ContinuousDistDoubleScalar;
import org.opentrafficsim.draw.graphs.FundamentalDiagram;
import org.opentrafficsim.draw.graphs.GraphCrossSection;
import org.opentrafficsim.draw.graphs.GraphPath;
import org.opentrafficsim.draw.graphs.GraphPath.Section;
import org.opentrafficsim.kpi.interfaces.LaneData;
import org.opentrafficsim.draw.graphs.FundamentalDiagram.FdSource;
import org.opentrafficsim.road.gtu.generator.GeneratorPositions;
import org.opentrafficsim.road.gtu.generator.LaneBasedGtuGenerator;
import org.opentrafficsim.road.gtu.generator.LaneBasedGtuGenerator.RoomChecker;
import org.opentrafficsim.road.gtu.generator.TtcRoomChecker;
import org.opentrafficsim.road.gtu.generator.characteristics.LaneBasedGtuTemplate;
import org.opentrafficsim.road.gtu.generator.characteristics.LaneBasedGtuTemplateDistribution;
import org.opentrafficsim.road.gtu.lane.LaneBasedGtu;
import org.opentrafficsim.road.gtu.lane.perception.mental.TaskManager;
import org.opentrafficsim.road.gtu.lane.tactical.LaneBasedTacticalPlannerFactory;
import org.opentrafficsim.road.gtu.lane.tactical.following.AbstractIdm;
import org.opentrafficsim.road.gtu.lane.tactical.following.CarFollowingModelFactory;
import org.opentrafficsim.road.gtu.lane.tactical.following.IdmPlus;
import org.opentrafficsim.road.gtu.lane.tactical.following.IdmPlusFactory;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.AccelerationConflicts;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.AccelerationIncentive;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.AccelerationSpeedLimitTransition;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.AccelerationTrafficLights;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.IncentiveCourtesy;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.IncentiveKeep;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.IncentiveRoute;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.IncentiveSocioSpeed;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.IncentiveSpeedWithCourtesy;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.Lmrs;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.LmrsFactory;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.Cooperation;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.GapAcceptance;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.LmrsParameters;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.MandatoryIncentive;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.Synchronization;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.Tailgating;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.VoluntaryIncentive;
import org.opentrafficsim.road.gtu.strategical.LaneBasedStrategicalRoutePlannerFactory;
import org.opentrafficsim.road.network.RoadNetwork;
import org.opentrafficsim.road.network.factory.xml.parser.XmlParser;
import org.opentrafficsim.road.network.lane.CrossSectionLink;
import org.opentrafficsim.road.network.lane.Lane;
import org.opentrafficsim.road.network.lane.LanePosition;
import org.opentrafficsim.road.network.lane.object.SpeedSign;
import org.opentrafficsim.road.network.sampling.LaneDataRoad;
import org.opentrafficsim.road.network.sampling.RoadSampler;

import nl.tudelft.simulation.dsol.SimRuntimeException;
import nl.tudelft.simulation.dsol.formalisms.eventscheduling.Executable;
import nl.tudelft.simulation.dsol.simulators.SimulatorInterface;
import nl.tudelft.simulation.jstats.distributions.DistUniform;
import nl.tudelft.simulation.jstats.streams.MersenneTwister;
import nl.tudelft.simulation.jstats.streams.StreamInterface;
import sim.demo.behavior.TrafficInteractionAdaptations.CarFollowingBehavior;
import sim.demo.gtu.CustomLaneBasedGtuTemplate;
import sim.demo.gtu.CustomLaneBasedGtuTemplateDistribution;
import sim.demo.lmrs.CustomCooperation;
import sim.demo.mental.CarFollowingTask;
import sim.demo.mental.CustomAdaptationHeadway;
import sim.demo.mental.LaneChangeTask;
import sim.demo.mental.TaskManagerAr;

import org.opentrafficsim.road.gtu.lane.perception.CategoricalLanePerception;
import org.opentrafficsim.road.gtu.lane.perception.LanePerception;
import org.opentrafficsim.road.gtu.lane.perception.PerceptionCollectable;
import org.opentrafficsim.road.gtu.lane.perception.PerceptionCollectable.UnderlyingDistance;
import org.opentrafficsim.road.gtu.lane.perception.PerceptionFactory;
import org.opentrafficsim.road.gtu.lane.perception.RelativeLane;
import org.opentrafficsim.road.gtu.lane.perception.categories.AnticipationTrafficPerception;
import org.opentrafficsim.road.gtu.lane.perception.categories.DirectInfrastructurePerception;
import org.opentrafficsim.road.gtu.lane.perception.categories.DirectIntersectionPerception;
import org.opentrafficsim.road.gtu.lane.perception.categories.neighbors.Anticipation;
import org.opentrafficsim.road.gtu.lane.perception.categories.neighbors.DirectNeighborsPerception;
import org.opentrafficsim.road.gtu.lane.perception.categories.neighbors.Estimation;
import org.opentrafficsim.road.gtu.lane.perception.categories.neighbors.HeadwayGtuType;
import org.opentrafficsim.road.gtu.lane.perception.categories.neighbors.HeadwayGtuType.PerceivedHeadwayGtuType;
import org.opentrafficsim.road.gtu.lane.perception.categories.neighbors.NeighborsPerception;
import org.opentrafficsim.road.gtu.lane.perception.headway.HeadwayDistance;
import org.opentrafficsim.road.gtu.lane.perception.headway.HeadwayGtu;
import org.opentrafficsim.road.gtu.lane.perception.mental.AdaptationHeadway;
import org.opentrafficsim.road.gtu.lane.perception.mental.AdaptationSituationalAwareness;
import org.opentrafficsim.road.gtu.lane.perception.mental.AdaptationSpeed;
import org.opentrafficsim.road.gtu.lane.perception.mental.Fuller;
import org.opentrafficsim.road.gtu.lane.perception.mental.Fuller.BehavioralAdaptation;
import org.opentrafficsim.road.gtu.lane.perception.mental.Task;


/**
 * <p>
 * Copyright (c) 2013-2024 Delft University of Technology, PO Box 5, 2600 AA,
 * Delft, the Netherlands. All rights reserved. <br>
 * BSD-style license. See
 * <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim
 * License</a>.
 * </p>
 * 
 * @author <a href="https://github.com/averbraeck">Alexander Verbraeck</a>
 * @author <a href="https://tudelft.nl/staff/p.knoppers-1">Peter Knoppers</a>
 * @author <a href="https://github.com/wjschakel">Wouter Schakel</a>
 */
public class VehicleAutomationModel extends AbstractOtsModel implements EventListener
{
	
	/** */
	private static final long serialVersionUID = 1L;
	
	/** Output data manager */
	private OutputDataManager outputDataManager;
	
	/** The network. */
	private OtsSimulatorInterface simulator;
	
	/** The network. */
	private RoadNetwork network;

	/** Network. */
	// Rijkswaterstaat ROA Chapter 6 contains standards on turbulence regions (figure 6.6)
	// also the merge lane lengths are defined (table 6.9)
	static final String networkString = "twoLaneFreewayWithOnRamp2";
	
	/** Speed limit and speed variation. */
	static final double speedLimit = 130.0;
	static final double speedMin = 120.0;
	static final double speedMax = 140.0;
	
	/** Vehicle length and width. */
	static final double vehicleLength = 4.0;
	static final double vehicleWidth = 2.0;
	
	/** Vehicle max acceleration and max deceleration. */
	static final double maxAcceleration = 3.0;
	static final double maxDeceleration = -8.0;
	
	/** Synchronisation. */
	static final Synchronization synchronizationMethod = Synchronization.ALIGN_GAP;

	/** Cooperation. */
	static final Cooperation cooperationMethod = CustomCooperation.PASSIVE_MOVING;
    
    /** Sampler. */
    private RoadSampler sampler;
    
    /** Cross section for data recording. */
    GraphCrossSection<LaneDataRoad> crossSection;
    
    private List<LaneDataRoad> allLanes;
    
    /** Sources. */
    private FdSource source;
    private LinkedHashMap<String, FdSource> individualSources;
    
    /** Map for GTU add times */
    private LinkedHashMap<String, Double> gtuAddTimesMap;
    
    /** Map for GTU remove times */
    private LinkedHashMap<String, Double> gtuRemoveTimesMap;
	
    private int calculations = 0;
	
	// input parameters
	
	/** Vehicle Automation Types and Parameters. */
	static VehicleConfigurationsBundle vehicleVars;
	
	/** Simulation configuration object. */
	public VehicleAutomationModelParameters simConfig;
	
	/** Vehicle configuration object. */
	public VehicleConfigurations vehicleConfig;
	
	/** Generator seed. */
	static long seed;
	
	/** StreamInterface to control randomness. */
	public StreamInterface stream;
	
	/** Total run time of simulation. */
	static Time simTime;
	
	/** Gtu type fractions in traffic. */
	static ArrayList<Double> gtuFractions;

	/** Left traffic fraction. */
	static double leftFraction;

	/** Main demand per lane. Default = 1000 */
	static Frequency mainDemand;

	/** Ramp demand. Default = 500 */
	static Frequency rampDemand;
	
	/** Use additional incentives. */
	static boolean additionalIncentives;
	
	/** File path for output data. */
	static String outputFolderPath;
	static String inputValuesFileName;
	static String singleOutputFileName;
	static String intermediateMeanValuesFileName;
	static String sequenceOutputFileName;
	static String laneChangeOutputFileName;
	

	/**
	 * Constructor for VehicleAutomationModel class.
	 * @param simulator OtsSimulatorInterface; the simulator
	 */
	public VehicleAutomationModel(final OtsSimulatorInterface simulator, final VehicleAutomationModelParameters simConfig) {
		// parse simulator
		super(simulator);
		
		this.simulator = simulator;
		this.simConfig = simConfig;
	}
	
	/**
	 * Set all required simulation parameters by the simConfig object.
	 */
	public void setSimParameters() {
		// initialise class variables
		gtuAddTimesMap = new LinkedHashMap<String, Double>();
		gtuRemoveTimesMap = new LinkedHashMap<String, Double>();
		gtuFractions = new ArrayList<Double>();
		
		// initialise input variables
		simTime = new Time(simConfig.getSimTime(), TimeUnit.BASE_SECOND);
		seed = simConfig.getSeed();
		gtuFractions.add(simConfig.getLevel0Fraction());
		gtuFractions.add(simConfig.getLevel1Fraction());
		gtuFractions.add(simConfig.getLevel2Fraction());
		gtuFractions.add(simConfig.getLevel3Fraction());
		leftFraction = simConfig.getLeftFraction();
		mainDemand = new Frequency(simConfig.getMainDemand(), FrequencyUnit.PER_HOUR);
		rampDemand = new Frequency(simConfig.getRampDemand(), FrequencyUnit.PER_HOUR);
		additionalIncentives = simConfig.getAdditionalIncentives();
		
		// file save locations
		outputFolderPath = simConfig.getOutputFolderPath();
		inputValuesFileName = simConfig.getInputValuesFileName();
		singleOutputFileName = simConfig.getSingleOutputFileName();
		intermediateMeanValuesFileName = simConfig.getIntermediateMeanValuesFileName();
		sequenceOutputFileName = simConfig.getSequenceOutputFileName();
		laneChangeOutputFileName = simConfig.getlaneChangeOutputFileName();
	}
	
	/**
	 * Save input parameter values.
	 */
	public void saveInputParameters() {
		// save input values
		outputDataManager.saveInputValue("simulation_time", Double.toString(simConfig.getSimTime()));
		outputDataManager.saveInputValue("seed", Long.toString(simConfig.getSeed()));
		outputDataManager.saveInputValue("level0_fraction", Double.toString(simConfig.getLevel0Fraction()));
		outputDataManager.saveInputValue("level1_fraction", Double.toString(simConfig.getLevel1Fraction()));
		outputDataManager.saveInputValue("level2_fraction", Double.toString(simConfig.getLevel2Fraction()));
		outputDataManager.saveInputValue("level3_fraction", Double.toString(simConfig.getLevel3Fraction()));
		outputDataManager.saveInputValue("left_fraction", Double.toString(simConfig.getLeftFraction()));
		outputDataManager.saveInputValue("main_demand", Double.toString(simConfig.getMainDemand()));
		outputDataManager.saveInputValue("ramp_demand", Double.toString(simConfig.getRampDemand()));
		outputDataManager.saveInputValue("additional_incentives", Boolean.toString(simConfig.getAdditionalIncentives()));
		
		// export these parameters to a CSV
		outputDataManager.exportInputValues(inputValuesFileName);
	}

	/**
	 * @param network RoadNetwork; set network.
	 */
	public void setNetwork(final RoadNetwork network) {
		this.network = network;
	}

	/** {@inheritDoc} */
	@Override
	public void constructModel() throws SimRuntimeException {
		try {
			// load network from xml file
			URL xmlURL = URLResource.getResource("C:\\Users\\jesse\\Documents\\Java\\TrafficSimulation-workspace\\"
													+ "traffic-sim\\src\\main\\resources\\networks\\" + networkString + ".xml");
			this.network = new RoadNetwork("VehicleAutomationSimulation", getSimulator());
			new XmlParser(this.network).setUrl(xmlURL).build();
			
			// set simulation configuration
			setSimParameters();
			
			// create output manager and save input parameters
			outputDataManager = new OutputDataManager(outputFolderPath, sequenceOutputFileName);
			saveInputParameters();
			
			// create generators
			addGenerator();
			
			// enable event notifications by adding listeners
			addSubscriptions();
			
			// create sampler to collect FD data
			createSampler();
			
			// setup FD source
			Duration updateInterval = Duration.instantiateSI(0.5);
			individualSources = new LinkedHashMap<String, FdSource>();
			createNetworkFdSources(updateInterval);
	        
	        // sample simulation data throughout the simulation
	        sampleData(updateInterval, false);
	        
		} catch (Exception exception) {
			exception.printStackTrace();
		}
	}
	
	private void createSampler() {
		this.sampler = new RoadSampler(this.network);
	}
	
	/** Method to sample simulation data */
	private void sampleData(Duration sampleInterval, boolean headersAlreadyCreated) {
		// create initial map
		Map<String, String> gtuData = new LinkedHashMap<>();
		gtuData.put("time", "");
    	gtuData.put("gtu_id", "");
    	gtuData.put("gtu_type", "");
    	gtuData.put("acceleration", "");
    	gtuData.put("speed", "");
    	gtuData.put("desired_speed", "");
    	gtuData.put("x_position", "");
    	gtuData.put("link", "");
    	gtuData.put("lane", "");
    	gtuData.put("task_saturation", "");
    	gtuData.put("headway_pair", "");
    	gtuData.put("headway_distance", "");
    	gtuData.put("headway_time", "");
    	gtuData.put("desired_headway_time", "");
    	gtuData.put("ttc", "");
    	gtuData.put("critical_ttc", "");

		// set headers of sample data if not done already
    	if (!headersAlreadyCreated) {
    		outputDataManager.createSequenceHeaders(gtuData);
    	}
		
		// get GTU data from all vehicles in the network
		for (Gtu gtu : this.network.getGTUs()) {
			LaneBasedGtu laneBasedGtu = (LaneBasedGtu) gtu;
			Map<String, String> observedGtuData = TrackGtuData(gtuData, laneBasedGtu);
			outputDataManager.addSequenceData(observedGtuData);
		}
		
		// recalculate FD values and stop sampler lane data recording if simulation is finished
		// this is necessary to make the last FD calculation reproducible
        double currentTime = this.simulator.getSimulatorAbsTime().si;
        double nextCalculationTime = currentTime + sampleInterval.si;
        if (nextCalculationTime > simTime.si) {
        	recalculateFdValues();
        	stopLaneRecording();
        }
		
		// reschedule this method for next GTU sample
		this.simulator.scheduleEventRel(sampleInterval, this, "sampleData", new Object[] {sampleInterval, true} );
	}
	
	/** Method to add listeners to simulation events. 
	 * @throws RemoteException */
	private void addSubscriptions() throws RemoteException {
		// simulator events
		this.simulator.addListener(this, SimulatorInterface.START_EVENT);
		this.simulator.addListener(this, SimulatorInterface.STOP_EVENT);
		this.simulator.addListener(this, SimulatorInterface.TIME_CHANGED_EVENT);
		
		// traffic events
		this.network.addListener(this, Network.GTU_ADD_EVENT);
		this.network.addListener(this, Network.GTU_REMOVE_EVENT);
		
		// gtu generation
		
	}

	/** Method to track and process notifications from the simulation listener. */
	@Override
	public void notify(Event event) throws RemoteException {

		// handle start of replication event
		if (event.getType().equals(SimulatorInterface.START_EVENT)) {
			System.out.println("Start simulation.");
			System.out.println("Total simulation time: " + simTime.si);
			
			// initialise count output variables
			
		}
		
		// handle end of replication event
		if (event.getType().equals(SimulatorInterface.STOP_EVENT)) {
			System.out.println("Simulation stopped at time: " + this.simulator.getSimulatorAbsTime().si);
			
			System.out.println("Stop simulation.");
			
			System.out.println("Calculations: " + calculations);
			
			saveFdValues();
			
			outputDataManager.finalExportToCsv();
		}
		
		// handle simulation time changed event
		if (event.getType().equals(SimulatorInterface.TIME_CHANGED_EVENT)) {
			
		}
		
		// handle generation of GTUs to the simulation
		if (event.getType().equals(Network.GTU_ADD_EVENT)) {
			String gtuId = (String) event.getContent();
			LaneBasedGtu gtu = (LaneBasedGtu) this.network.getGTU(gtuId);
			
			// add GTU listeners
			gtu.addListener(this, Gtu.MOVE_EVENT);
			gtu.addListener(this, LaneBasedGtu.LANE_CHANGE_EVENT);
			gtu.addListener(this, LaneBasedGtu.LANEBASED_MOVE_EVENT);
			
			// add GTU to gtuAddTimesMap to track GTU travel time
			gtuAddTimesMap.put(gtuId, this.simulator.getSimulatorAbsTime().getSI());
		}
		
		// disable subscription to GTU lane events when GTUs leave the simulation
		if (event.getType().equals(Network.GTU_REMOVE_EVENT)) {
			String gtuId = (String) event.getContent();
			LaneBasedGtu gtu = (LaneBasedGtu) this.network.getGTU(gtuId);
			
			// remove GTU listeners
			gtu.removeListener(this, Gtu.MOVE_EVENT);
			gtu.removeListener(this, LaneBasedGtu.LANE_CHANGE_EVENT);
			
			// add GTU to gtuRemoveTimesMap and calculate travelled time
			gtuRemoveTimesMap.put(gtuId, this.simulator.getSimulatorAbsTime().getSI());
			calculateRemovedGtuTravelTime(gtu, gtuId);
		}
		
		// handle GTU move events
		if (event.getType().equals(Gtu.MOVE_EVENT)) {
			String gtuId = (String) ((Object[]) event.getContent())[0];
	        LaneBasedGtu gtu = (LaneBasedGtu) this.network.getGTU(gtuId);
	        
	        // at GTU's first move event, set distribution parameters according to automation level
			if (gtu.getOdometer().equals(Length.instantiateSI(0.0))) {
				try {
					Parameters parametersForThisType = vehicleConfig.drawDistributionParameters(this.stream, gtu.getType(), gtu.getParameters());
					gtu.setParameters(parametersForThisType);
				} catch (ParameterException e) {
					CategoryLogger.always().error("GTU distribution parameters could not be set.");
					e.printStackTrace();
				}
			}
	        
	        // change driving behaviour because of surrounding traffic
	        try {
	        	// change headway for vehicle interactions during CF
	        	CarFollowingBehavior carFollowingBehavior = new CarFollowingBehavior(gtu);
	        	carFollowingBehavior.adaptToVehicleInFront();
	        	// change LC behaviour for vehicle interactions for cooperation (because level-0 vehicles will try to gain from level-3 vehicles)
	        	// lane changing behaviour is changed when lmrs cooperation decision is made, CustomCooperation.java contains this logic
			} catch (OperationalPlanException e) {
				e.printStackTrace();
			} catch (ParameterException e) {
				e.printStackTrace();
			}
		}
		
		if (event.getType().equals(LaneBasedGtu.LANEBASED_MOVE_EVENT)) {
			
		}
		
		// process lane change event
		if (event.getType().equals(LaneBasedGtu.LANE_CHANGE_EVENT)) {
			// get lane change info
			String gtuId = (String) ((Object[]) event.getContent())[0];
			String gtuDirection = (String) ((Object[]) event.getContent())[1];
			String gtuLink = (String) ((Object[]) event.getContent())[2];
			// lane info contains the lane where the GTU departed from
			String gtuFromLane = (String) ((Object[]) event.getContent())[3];
			// save this info
			TrackLaneChanges(gtuId, gtuDirection, gtuLink, gtuFromLane);
		}
	}

	/** {@inheritDoc} */
	@Override
	public RoadNetwork getNetwork() {
		return this.network;
	}

	/**
	 * Create generators.
	 * 
	 * @throws ParameterException   on parameter exception
	 * @throws GtuException         on GTU exception
	 * @throws NetworkException     if not does not exist
	 * @throws ProbabilityException negative probability
	 * @throws SimRuntimeException  in case of sim run time exception
	 * @throws RemoteException      if no simulator
	 */
	private void addGenerator() throws ParameterException, GtuException, NetworkException, ProbabilityException,
			SimRuntimeException, RemoteException {
		
		// source settings
		Map<String, StreamInterface> streams = new LinkedHashMap<>();
		final long seedAbove0 = seed + 1;
		this.stream = new MersenneTwister(seedAbove0);
		streams.put("headwayGeneration", this.stream);
		streams.put("gtuClass", new MersenneTwister(seedAbove0));
		getStreamInformation().addStream("headwayGeneration", this.stream);
		getStreamInformation().addStream("gtuClass", streams.get("gtuClass"));
		TtcRoomChecker roomChecker = new TtcRoomChecker(new Duration(10.0, DurationUnit.SI));
		IdGenerator idGenerator = new IdGenerator("");
		
		// car following model
		CarFollowingModelFactory<IdmPlus> idmPlusFactory = new IdmPlusFactory(streams.get("gtuClass"));
		ParameterSet params = new ParameterSet();
		params.setDefaultParameter(AbstractIdm.DELTA);
		
		// driver perception
		PerceptionFactory perceptionFactory = new PerceptionFactory()
		{
			public Parameters getParameters() throws ParameterException
		    {
		        return new ParameterSet().setDefaultParameter(ParameterTypes.LOOKAHEAD).setDefaultParameter(ParameterTypes.LOOKBACKOLD)
		                .setDefaultParameter(ParameterTypes.PERCEPTION).setDefaultParameter(ParameterTypes.LOOKBACK);
		    }
			
			@Override
			public LanePerception generatePerception(final LaneBasedGtu gtu)
			{
				Set<Task> tasks = new LinkedHashSet<>();
                tasks.add(new CarFollowingTask());
                tasks.add(new LaneChangeTask());
                
                Set<BehavioralAdaptation> behavioralAdapatations = new LinkedHashSet<>();
                // these adaptations must be present! next to adjusting parameters based on task load,
                // they are responsible to set the parameters required for Fuller perception
                behavioralAdapatations.add(new AdaptationSituationalAwareness());
                behavioralAdapatations.add(new AdaptationHeadway());
                behavioralAdapatations.add(new AdaptationSpeed());
                
                TaskManager taskManager = new TaskManagerAr("car-following");
                CategoricalLanePerception perception =
                        new CategoricalLanePerception(gtu, new Fuller(tasks, behavioralAdapatations, taskManager));
                
                Estimation estimation = Estimation.NONE;
                Anticipation anticipation = Anticipation.CONSTANT_ACCELERATION;
                
                HeadwayGtuType headwayGtuType = new PerceivedHeadwayGtuType(estimation, anticipation);
                perception.addPerceptionCategory(new DirectEgoPerception<>(perception));
                perception.addPerceptionCategory(new DirectInfrastructurePerception(perception));
                perception.addPerceptionCategory(new DirectNeighborsPerception(perception, headwayGtuType));
                perception.addPerceptionCategory(new AnticipationTrafficPerception(perception));
                perception.addPerceptionCategory(new DirectIntersectionPerception(perception, HeadwayGtuType.WRAP));
                return perception;
			}
		};
		
		// drive incentives
		Set<MandatoryIncentive> mandatoryIncentives = new LinkedHashSet<>();
		Set<VoluntaryIncentive> voluntaryIncentives = new LinkedHashSet<>();
		Set<AccelerationIncentive> accelerationIncentives = new LinkedHashSet<>();
		mandatoryIncentives.add(new IncentiveRoute());
		if (additionalIncentives) {
			// mandatoryIncentives.add(new IncentiveGetInLane());
		}
		voluntaryIncentives.add(new IncentiveSpeedWithCourtesy());
		voluntaryIncentives.add(new IncentiveKeep());
		if (additionalIncentives) {
			voluntaryIncentives.add(new IncentiveCourtesy());
			voluntaryIncentives.add(new IncentiveSocioSpeed());
		}
		accelerationIncentives.add(new AccelerationSpeedLimitTransition());
		accelerationIncentives.add(new AccelerationTrafficLights());
		accelerationIncentives.add(new AccelerationConflicts());
//		LaneBasedTacticalPlannerFactory<Lmrs> tacticalFactory = new LmrsFactory(idmPlusFactory,
//				new DefaultLmrsPerceptionFactory(), synchronizationMethod, cooperationMethod, GapAcceptance.INFORMED,
//				Tailgating.NONE, mandatoryIncentives, voluntaryIncentives, accelerationIncentives);
		LaneBasedTacticalPlannerFactory<Lmrs> tacticalFactory = new LmrsFactory(idmPlusFactory,
				perceptionFactory, synchronizationMethod, cooperationMethod, GapAcceptance.INFORMED,
				Tailgating.NONE, mandatoryIncentives, voluntaryIncentives, accelerationIncentives);
		
		// retrieve automation vehicle types
		vehicleConfig = new VehicleConfigurations();
		vehicleVars = vehicleConfig.setVehicleTypes();
		ArrayList<GtuType> availableGtuTypes = vehicleVars.getAutomationGtuTypes();
		ParameterFactoryByType paramFactory = vehicleVars.getParameterFactory();
		
		// create routes on RoadNetwork (routes are the same for all gtu types)
		Route routeAD = this.network.getShortestRouteBetween(availableGtuTypes.get(0), this.network.getNode("A"), this.network.getNode("D"));
		Route routeED = this.network.getShortestRouteBetween(availableGtuTypes.get(0), this.network.getNode("E"), this.network.getNode("D"));
		
		// create route generators
		List<FrequencyAndObject<Route>> routesA = new ArrayList<>();
		routesA.add(new FrequencyAndObject<>(1.0 - leftFraction, routeAD));
//		routesA.add(new FrequencyAndObject<>(leftFraction, routeAG));
		List<FrequencyAndObject<Route>> routesE = new ArrayList<>();
		routesE.add(new FrequencyAndObject<>(1.0 - leftFraction, routeED));
//		routesF.add(new FrequencyAndObject<>(leftFraction, routeFG));
		Generator<Route> routeGeneratorA = new ProbabilisticRouteGenerator(routesA, stream);
		Generator<Route> routeGeneratorE = new ProbabilisticRouteGenerator(routesE, stream);

		// set initial speed for generators
		double rampSpeed = 50.0;	// Rijkswaterstaat ROA 6.1.1 (table 6.1)
		Speed speedA = new Speed(speedLimit, SpeedUnit.KM_PER_HOUR);
		Speed speedE = new Speed(rampSpeed, SpeedUnit.KM_PER_HOUR);
		
		// define cross sections for vehicle generation
		CrossSectionLink linkA = (CrossSectionLink) this.network.getLink("AB");
		CrossSectionLink linkF = (CrossSectionLink) this.network.getLink("EE2");
		
		// create headway generators
		Generator<Duration> headwaysA1 = new HeadwayGenerator(getSimulator(), mainDemand);
		Generator<Duration> headwaysA2 = new HeadwayGenerator(getSimulator(), mainDemand);
		Generator<Duration> headwaysE = new HeadwayGenerator(getSimulator(), rampDemand);

		// speed generators (the same for all gtu types)
		ContinuousDistDoubleScalar.Rel<Speed, SpeedUnit> gtuSpeed = new ContinuousDistDoubleScalar.Rel<>(
				new DistUniform(stream, speedMin, speedMax), SpeedUnit.KM_PER_HOUR);
		
		// strategical planner factory
		LaneBasedStrategicalRoutePlannerFactory strategicalFactory = new LaneBasedStrategicalRoutePlannerFactory(
				tacticalFactory, paramFactory);
		
		// create vehicle templates, with routes from main road and ramp
		ArrayList<LaneBasedGtuTemplate> gtuMainTemplates = new ArrayList<LaneBasedGtuTemplate>();
		ArrayList<LaneBasedGtuTemplate> gtuRampTemplates = new ArrayList<LaneBasedGtuTemplate>();
		for (GtuType gtuType : availableGtuTypes) {
			// main road
			LaneBasedGtuTemplate templateA = 
					new LaneBasedGtuTemplate(gtuType, 
					new ConstantGenerator<>(Length.instantiateSI(vehicleLength)),
					new ConstantGenerator<>(Length.instantiateSI(vehicleWidth)),
					gtuSpeed,
//					new ConstantGenerator<>(Acceleration.instantiateSI(maxAcceleration)), 
//					new ConstantGenerator<>(Acceleration.instantiateSI(maxDeceleration)),
					strategicalFactory, routeGeneratorA);
			gtuMainTemplates.add(templateA);
			// ramp
			LaneBasedGtuTemplate templateE = 
					new LaneBasedGtuTemplate(gtuType, 
					new ConstantGenerator<>(Length.instantiateSI(vehicleLength)),
					new ConstantGenerator<>(Length.instantiateSI(vehicleWidth)),
					gtuSpeed, 
//					new ConstantGenerator<>(Acceleration.instantiateSI(maxAcceleration)), 
//					new ConstantGenerator<>(Acceleration.instantiateSI(maxDeceleration)),
					strategicalFactory, routeGeneratorE);
			gtuRampTemplates.add(templateE);
		}
		
		// set vehicle distributions for
		// left lane
		Distribution<LaneBasedGtuTemplate> gtuTypeLaneA1 = new Distribution<>(streams.get("gtuClass"));
		for (int i = 0; i < gtuMainTemplates.size(); i++) {
			gtuTypeLaneA1.add(new FrequencyAndObject<>(gtuFractions.get(i), gtuMainTemplates.get(i)));
		}
		// right lane
		Distribution<LaneBasedGtuTemplate> gtuTypeLaneA2 = new Distribution<>(streams.get("gtuClass"));
		for (int i = 0; i < gtuMainTemplates.size(); i++) {
			gtuTypeLaneA2.add(new FrequencyAndObject<>(gtuFractions.get(i), gtuMainTemplates.get(i)));
		}
		// ramp lane
		Distribution<LaneBasedGtuTemplate> gtuTypeLaneE = new Distribution<>(streams.get("gtuClass"));
		for (int i = 0; i < gtuMainTemplates.size(); i++) {
			gtuTypeLaneE.add(new FrequencyAndObject<>(gtuFractions.get(i), gtuRampTemplates.get(i)));
		}

		// set vehicle colours
		GtuColorer colorer = new LmrsSwitchableColorer(DefaultsNl.GTU_TYPE_COLORS.toMap());
		
		// create generators for all lanes with corresponding objects
		// left lane
		makeGenerator(getLane(linkA, "FORWARD1"), speedA, "gen1", idGenerator, gtuTypeLaneA1, headwaysA1, colorer,
				roomChecker, paramFactory, tacticalFactory, simTime, streams.get("gtuClass"));
		// right lane
		makeGenerator(getLane(linkA, "FORWARD2"), speedA, "gen2", idGenerator, gtuTypeLaneA2, headwaysA2, colorer,
				roomChecker, paramFactory, tacticalFactory, simTime, streams.get("gtuClass"));
		// ramp lane
		makeGenerator(getLane(linkF, "FORWARD1"), speedE, "gen3", idGenerator, gtuTypeLaneE, headwaysE, colorer,
				roomChecker, paramFactory, tacticalFactory, simTime, streams.get("gtuClass"));

		new SpeedSign("sign1", getLane(linkA, "FORWARD1"), Length.instantiateSI(10), this.getSimulator(),
				new Speed(speedLimit, SpeedUnit.KM_PER_HOUR), DefaultsNl.VEHICLE, Duration.ZERO,
				new Duration(24, DurationUnit.HOUR));

	}

	/**
	 * Get lane from link by id.
	 * 
	 * @param link CrossSectionLink; link
	 * @param id   String; id
	 * @return lane
	 */
	private Lane getLane(final CrossSectionLink link, final String id) {
		return (Lane) link.getCrossSectionElement(id);
	}

	/**
	 * @param lane             Lane; the reference lane for this generator
	 * @param generationSpeed  Speed; the speed of the GTU
	 * @param id               String; the id of the generator itself
	 * @param idGenerator      IdGenerator; the generator for the ID
	 * @param distribution     Distribution&lt;LaneBasedTemplateGTUType&gt;; the
	 *                         type generator for the GTU
	 * @param headwayGenerator Generator&lt;Duration&gt;; the headway generator for
	 *                         the GTU
	 * @param gtuColorer       GtuColorer; the GTU colorer for animation
	 * @param roomChecker      RoomChecker; the checker to see if there is room for
	 *                         the GTU
	 * @param bcFactory        ParameterFactory; the factory to generate parameters
	 *                         for the GTU
	 * @param tacticalFactory  LaneBasedTacticalPlannerFactory&lt;?&gt;; the
	 *                         generator for the tactical planner
	 * @param simulationTime   Time; simulation time
	 * @param stream           StreamInterface; random numbers stream
	 * @throws SimRuntimeException  in case of scheduling problems
	 * @throws ProbabilityException in case of an illegal probability distribution
	 * @throws GtuException         in case the GTU is inconsistent
	 * @throws ParameterException   in case a parameter for the perception is
	 *                              missing
	 * @throws NetworkException     if the object could not be added to the network
	 */
	private void makeGenerator(final Lane lane, final Speed generationSpeed, final String id,
			final IdGenerator idGenerator, final Distribution<LaneBasedGtuTemplate> distribution,
			final Generator<Duration> headwayGenerator, final GtuColorer gtuColorer, final RoomChecker roomChecker,
			final ParameterFactory bcFactory, final LaneBasedTacticalPlannerFactory<?> tacticalFactory,
			final Time simulationTime, final StreamInterface stream)
			throws SimRuntimeException, ProbabilityException, GtuException, ParameterException, NetworkException {

		Set<LanePosition> initialLongitudinalPositions = new LinkedHashSet<>();
		// TODO DIR_MINUS
		initialLongitudinalPositions.add(new LanePosition(lane, new Length(5.0, LengthUnit.SI)));
		LaneBasedGtuTemplateDistribution characteristicsGenerator = new LaneBasedGtuTemplateDistribution(distribution);
		new LaneBasedGtuGenerator(id, headwayGenerator, characteristicsGenerator,
				GeneratorPositions.create(initialLongitudinalPositions, stream), this.network, getSimulator(),
				roomChecker, idGenerator);
	}

	/**
	 * <p>
	 * Copyright (c) 2013-2024 Delft University of Technology, PO Box 5, 2600 AA,
	 * Delft, the Netherlands. All rights reserved. <br>
	 * BSD-style license. See
	 * <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim
	 * License</a>.
	 * </p>
	 * 
	 * @author <a href="https://github.com/averbraeck">Alexander Verbraeck</a>
	 * @author <a href="https://tudelft.nl/staff/p.knoppers-1">Peter Knoppers</a>
	 * @author <a href="https://github.com/wjschakel">Wouter Schakel</a>
	 */
	private static class HeadwayGenerator implements Generator<Duration> {
		/** the simulator. */
		private final OtsSimulatorInterface simulator;

		/** Demand level. */
		private final Frequency demand;

		/**
		 * @param simulator OtsSimulatorInterface; the simulator
		 * @param demand    Frequency; demand
		 */
		HeadwayGenerator(final OtsSimulatorInterface simulator, final Frequency demand) {
			this.simulator = simulator;
			this.demand = demand;
		}

		/** {@inheritDoc} */
		@Override
		public Duration draw() throws ProbabilityException, ParameterException {
			return new Duration(
					-Math.log(this.simulator.getModel().getStream("headwayGeneration").nextDouble()) / this.demand.si,
					DurationUnit.SI);
		}
	}
	
	/**
	 * Function to track GTU info within the simulation.
	 * @param gtu
	 */
	public Map<String, String> TrackGtuData(Map<String, String> gtuData, LaneBasedGtu gtu)
	{
		// main GTU info
    	double time = this.simulator.getSimulatorAbsTime().si;
		String gtuId = gtu.getId();
    	String gtuType = gtu.getType().getId();
        
        // get detailed data
        double acceleration = gtu.getAcceleration().si;
        double speed = gtu.getSpeed().si;
        double desiredSpeed = gtu.getDesiredSpeed().si;
        double xPosition = gtu.getLocation().getX();
        double taskSaturation = gtu.getParameters().getParameterOrNull(Fuller.TS);
        
        // get lane and link
        String lane = null;
        String link = null;
		try {
			lane = gtu.getReferencePosition().lane().getId().toString();
			link = gtu.getReferencePosition().lane().getLink().getId().toString();
		} catch (GtuException e) {
			e.printStackTrace();
		}
		
		// get headway info
		HeadwayInfo headwayInfo = new HeadwayInfo(gtu);
    	double desiredHeadwayTime = gtu.getParameters().getParameterOrNull(ParameterTypes.T).si;
    	
    	// collect observed and calculated data
    	gtuData.put("time", Double.toString(time));
    	gtuData.put("gtu_id", gtuId);
    	gtuData.put("gtu_type", gtuType);
    	gtuData.put("acceleration", Double.toString(acceleration));
    	gtuData.put("speed", Double.toString(speed));
    	gtuData.put("desired_speed", Double.toString(desiredSpeed));
    	gtuData.put("x_position", Double.toString(xPosition));
    	gtuData.put("link", link);
    	gtuData.put("lane", lane);
    	gtuData.put("task_saturation", Double.toString(taskSaturation));
    	gtuData.put("headway_pair", headwayInfo.getGtuType() + "-" + headwayInfo.getLeaderGtuType());
    	gtuData.put("headway_distance", Double.toString(headwayInfo.getHeadwayDistance()));
    	gtuData.put("headway_time", Double.toString(headwayInfo.getHeadwayTime()));
    	gtuData.put("desired_headway_time", Double.toString(desiredHeadwayTime));
    	gtuData.put("ttc", Double.toString(headwayInfo.getTtc()));
    	gtuData.put("critical_ttc", Boolean.toString(headwayInfo.getCriticalTtc()));
    	
    	// return data from this GTU
    	return gtuData;
	}
	
	/**
	 * Function to track GTU lane changes in the simulation.
	 * @param: gtuId; ID of lane changing GTU
	 * @param: gtuDirection; Direction of lane change
	 * @param: gtuLink; Network link where lane change was started from
	 * @param: gtuLane; Lane of network link where lane change was started from
	 */
	public void TrackLaneChanges(String gtuId, String gtuDirection, String gtuLink, String gtuFromLane) {
		double time = this.simulator.getSimulatorAbsTime().si;
		outputDataManager.saveLaneChange(time, gtuId, gtuDirection, gtuLink, gtuFromLane);
	}
	
	/**
     * Method to calculate travel time of GTUs removed from the simulation.
     * @param gtuId String;
     */
	private void calculateRemovedGtuTravelTime(Gtu gtu, String gtuId) {
		
		// calculate travel time
		double addTime = gtuAddTimesMap.get(gtuId);
		double removeTime = gtuRemoveTimesMap.get(gtuId);
		double travelTime = removeTime - addTime;
		
		// get travelled distance
		// travel time per distance (traveltime/distance) gives insight into the mean time
		// the vehicle did on a specific distance, but this is just the inverse of the mean speed
		// double distance = gtu.getOdometer().getSI();
		
		// save value
		outputDataManager.addToMeanList("mean_travel_time", travelTime);
		
		// this GTU is removed from the simulation, so can also be removed from the maps
		gtuAddTimesMap.remove(gtuId);
		gtuRemoveTimesMap.remove(gtuId);
		
	}
	
	/**
     * Method to create sources for Fundamental Diagram values for specific lanes.
     * @param updateInterval Duration;
     */
    private void createLaneSpecifiFdSources(Duration updateInterval)
    {	
    	// create two data lanes
    	GraphPath<LaneDataRoad> path0;
    	GraphPath<LaneDataRoad> path1;
		try {
			// get left lane after merge ramp
			Lane start0 = ((CrossSectionLink) this.network.getLink("CD")).getLanes().get(0);
			path0 = GraphLaneUtil.createPath("Left lane", start0);
			//get right lane after merge ramp
			Lane start1 = ((CrossSectionLink) this.network.getLink("CD")).getLanes().get(1);
			path1 = GraphLaneUtil.createPath("Right lane", start1);
			
			// list all lanes (required for stopLaneRecording() method)
			this.allLanes = new ArrayList<>();
		} catch (NetworkException exception) {
			throw new RuntimeException("Could not create a path as a lane has no set speed limit.", exception);
		}
		
		// initialise data recording on lanes
		GraphPath.initRecording(this.sampler, path0);
		GraphPath.initRecording(this.sampler, path1);
        
        // create FD sources
		FdSource pathSource0 = FundamentalDiagram.sourceFromSampler(sampler, path0, true, updateInterval);
		FdSource pathSource1 = FundamentalDiagram.sourceFromSampler(sampler, path1, true, updateInterval);
		
		// combine sources of separate lanes into one FD source
		Map<String, FdSource> sourceMap = new LinkedHashMap<>();
		sourceMap.put("Source0", pathSource0);
		sourceMap.put("Source1", pathSource1);
		this.source = FundamentalDiagram.combinedSource(sourceMap);
		
    }
    		
	/**
     * Method to create sources for network Fundamental Diagram calculations.
     * @param updateInterval Duration;
     */
    private void createNetworkFdSources(Duration updateInterval)
    {	
    	// collect all network links
        List<CrossSectionLink> allLinks = new ArrayList<>();
        ImmutableCollection<Link> links = this.network.getLinkMap().values();
        for (Link link : links) {
            allLinks.add((CrossSectionLink) link);
        }
        
        // collect all lanes within these links
        this.allLanes = new ArrayList<>();
        for (CrossSectionLink link : allLinks) {
            for (Lane lane : link.getLanes()) {
                allLanes.add(new LaneDataRoad(lane));
            }
        }

        // create a sections list containing all found lanes and add required length and speed data
        // also initialise data recording for all lanes
        List<Section<LaneDataRoad>> sections = new ArrayList<>();
        for (LaneDataRoad laneData : allLanes) {
            this.sampler.initRecording(laneData);

            Length laneLength = laneData.getLength(); // get lane length
            Speed laneSpeedLimit = new Speed(speedLimit, SpeedUnit.KM_PER_HOUR); // set speed limit for this lane
            
            List<LaneDataRoad> laneList = new ArrayList<>(); // create list of this lane because Section<>() expects a list, not a single lane
            laneList.add(laneData);
            ;
			// add this lane to the list of sections
            Section<LaneDataRoad> section = new Section<LaneDataRoad>(laneLength, laneSpeedLimit, laneList);
            sections.add(section);
            
            // also create a FD source specifically for this lane
            List<Section<LaneDataRoad>> individualSectionList = new ArrayList<>();
            individualSectionList.add(section);
            GraphPath<LaneDataRoad> individualGraphPath = new GraphPath<LaneDataRoad>(laneData.getLane().getFullId(), individualSectionList);
            FdSource individualSource = FundamentalDiagram.sourceFromSampler(sampler, individualGraphPath, true, updateInterval);
            this.individualSources.put(laneData.getLane().getFullId(), individualSource);
        }

        this.source = FundamentalDiagram.combinedSource(individualSources);
    }
    
    /*
     * Method to recalculate FD values at fixed frequency.
     */
    private void recalculateFdValues() {
    	
    	calculations += 1;
    	
    	// recalculate FD values
        this.source.recalculate(this.simulator.getSimulatorAbsTime());
    }
    
    /*
     * Method to stop the sampler's lane recording for all listed network lanes.
     */
    private void stopLaneRecording() {
    	// stop recording on the sampler for the entire network, only if lanes are specified
    	if (!this.allLanes.isEmpty()) {
        	System.out.println("Stopped sampler lane recording");
    		for (LaneDataRoad laneData : this.allLanes) {
            	this.sampler.stopRecording(laneData);
            }
    	}
    }
    
    /*
     * Method to save calculated FD values.
     */
    private void saveFdValues()
    {
        // get last calculated index
        int latestIndex = this.source.getItemCount(0);
        if (latestIndex > 0) {
        	latestIndex -= 1;
        }
        
        // process all calculated values
        for (int i = 0; i < latestIndex; i++) {
        	outputDataManager.addToMeanList("mean_speed", this.source.getSpeed(0, i));
        	outputDataManager.addToMeanList("mean_density", this.source.getDensity(0, i));
        	outputDataManager.addToMeanList("mean_flow", this.source.getFlow(0, i));
        }
        
        // repeat this for individual sources
        for (String laneString : this.individualSources.keySet()) {
        	FdSource laneSource = this.individualSources.get(laneString);
        	latestIndex = laneSource.getItemCount(0);
            if (latestIndex > 0) {
            	latestIndex -= 1;
            }
        	
        	for (int i = 0; i < latestIndex; i++) {
            	outputDataManager.addToMeanList(laneString + "_mean_speed", laneSource.getSpeed(0, i));
            	outputDataManager.addToMeanList(laneString + "_mean_density", laneSource.getDensity(0, i));
            	outputDataManager.addToMeanList(laneString + "_mean_flow", laneSource.getFlow(0, i));
            }
        }
    }
    
	
	/**
     * Class for output data.
     */
    public class OutputDataManager
    {
    	/** Exporter. */
    	ExportSimulationOutput exporter = null;
    	
    	/** Map of input parameter values. */
    	private Map<String, String> inputValuesMap = new LinkedHashMap<>();
        
        /** Map of values for each constant output parameter. */
        private Map<String, Object> singleOutputMap = new LinkedHashMap<>();
        
        /** Map to temporarily store values to calculate the mean. */
        private Map<String, ArrayList<Double>> meanMap = new LinkedHashMap<>();
        
        /** Lists to store lane change data. */
        public ArrayList<String> laneChangeTime = new ArrayList<String>();
        public ArrayList<String> laneChangeIds = new ArrayList<String>();
        public ArrayList<String> laneChangeDirections = new ArrayList<String>();
        public ArrayList<String> laneChangeLinks = new ArrayList<String>();
        public ArrayList<String> laneChangeFromLanes = new ArrayList<String>();
        
        
        /** Constructor */
        public OutputDataManager(String outputFolderPath, String sequenceFileName) {
        	// create export object and perform export functions
        	exporter = new ExportSimulationOutput(outputFolderPath, inputValuesFileName, sequenceFileName, singleOutputFileName,
        										  intermediateMeanValuesFileName, laneChangeOutputFileName);
        }
        
        
        /** Export output data at end of simulation. */
        public void finalExportToCsv() {
        	
        	// calculate mean values for the variables stored in the meanMap and save them as single output (not sequence)
        	calculateAndSaveMeans();
        	
        	// perform export of mean, intermediate mean, and lane change data
        	try {
	        	exporter.exportSingleToCsv(singleOutputMap, singleOutputFileName);
	        	exporter.exportIntermediateMeanValuesToCsv(meanMap , intermediateMeanValuesFileName);
	        	exporter.exportLaneChangesToCsv(laneChangeTime, laneChangeIds, laneChangeDirections,
	        									laneChangeLinks, laneChangeFromLanes, laneChangeOutputFileName);
        	} catch (NullPointerException e) {
        		e.printStackTrace();
        	}
        	
        	// sequence data is already saved during the simulation
        	// output streams are closed when program shuts down, see ExportSimulationOutput class
        }
        
        /** Function to retrieve Map of single output data. */
        public Map<String, Object> getSingleOutputData() {
        	return singleOutputMap;
        }
        
        /** Function to retrieve Map of values used for mean calculations. */
        public Map<String, ArrayList<Double>> getValuesForMeanCalculationData() {
        	return meanMap;
        }
        
        /** Function to write headers for the sequence data file. */
        public void createSequenceHeaders(Map<String, String> dataRow) {
        	ArrayList<String> headers = new ArrayList<String>();
        	
        	// get all variable names
        	for (String variable : dataRow.keySet()) {
        		headers.add(variable);
        	}
        	
        	// write headers for sequence data file
        	exporter.writeSequenceDataHeaders(headers);
        }
        
        /** Function to save a sequence data row. */
        public void addSequenceData(Map<String, String> dataRow) {
        	// write new dataRow to file
        	exporter.writeSequenceDataRow(dataRow);
        }
        
        /** Save input parameter values */
        public void saveInputValue(String paramString, String paramValue) {
        	inputValuesMap.put(paramString, paramValue);
        }
        
        /**
         * Export saved input parameter values.
         * @param inputFilePath
         */
        public void exportInputValues(String inputFilePath) {
        	exporter.exportInputToCsv(inputValuesMap, inputFilePath);
        }
        
        /** Function to add a value to a list that is meant for post simulation calculation of the mean value */
        public void addToMeanList(String paramString, Double paramValue) {
        	
        	// check whether this parameter string already exists
        	boolean exists = meanMap.containsKey(paramString);
        	// exists? then, add value to the existing list
        	if (exists) {
        		this.meanMap.get(paramString).add(paramValue);
        	}
        	// does not exist yet? then, create new list for this first value and add it to the map
        	else {
        		ArrayList<Double> firstValueList = new ArrayList<Double>();
        		firstValueList.add(paramValue);
        		this.meanMap.put(paramString, firstValueList);
        	}
        }
        
        /** Function to set a constant output value. */
        public void setSingleValue(String paramString, Object paramValue) {
        	singleOutputMap.put(paramString, paramValue);
        }
        
        /** Function increase a constant count. */
        public void increaseSingleCount(String paramString) {
        	
        	// check whether this parameter string already exists
        	boolean exists = singleOutputMap.containsKey(paramString);
        	// exists? then, increase existing value
        	if (exists) {
        		try {
        			int newValue = (int) singleOutputMap.get(paramString) + 1;
    	        	singleOutputMap.put(paramString, newValue);      		
    	        } 
        		catch (Exception exception) {
        			exception.printStackTrace();
        		}
        	}
        	// does not exist? then, create new constant and start counting
        	else {
        		int value = 1;
        		singleOutputMap.put(paramString, value);
        	}
        }
        
        /** Function increase a constant count. */
        public void decreaseSingleCount(String paramString) {
        	
        	// check whether this parameter string already exists
        	boolean exists = singleOutputMap.containsKey(paramString);
        	// exists? then, increase existing value
        	if (exists) {
        		try {
        			int newValue = (int) singleOutputMap.get(paramString) - 1;
    	        	singleOutputMap.put(paramString, newValue);
        		} 
        		catch (Exception exception) {
        			exception.printStackTrace();
        		}
        	}
        	// does not exist? then, create new constant and start counting
        	else {
        		int value = 1;
        		singleOutputMap.put(paramString, value);
        	}
        }
        
        /** Function to calculate the means for variables stored in the mean map. */
        private void calculateAndSaveMeans() {
        	
        	// loop through stored variables
        	for (String varName : this.meanMap.keySet()) {
        		// get values
        		ArrayList<Double> varValues = meanMap.get(varName);
        		// calculate mean value if values are available
        		if (varValues.size() > 0) {
        			double mean = calculateMeanOfList(varValues);
        			// save as constant output parameter
            		this.setSingleValue(varName, mean);
        		}
        		else {
        			String mean = "No values";
        			// save as constant output parameter
            		this.setSingleValue(varName, mean);
        		}
        	}
        	
        }
        
        /** Function to calculate the mean of a list of values.
         * @param valueList ArrayList<Double>;
         * */
        private double calculateMeanOfList(ArrayList<Double> valueList) {
        	
        	// get sum of values
        	double sum = 0.0;
        	for (double value : valueList) {
        		// do not take NaN values into account, this will result in a NaN sum
        		if (Double.isNaN(value)) {
                	continue;
        		}
        		sum += value;
        	}
        	
        	// calculate mean
        	double mean = sum / valueList.size();
        	
        	// return mean value
        	return mean;
        }
        
        /** Function to add lane change data to the corresponding lists */
        public void saveLaneChange(double time, String gtuId, String gtuDirection, String gtuLink, String gtuFromLane)
        {
        	laneChangeTime.add(Double.toString(time));
        	laneChangeIds.add(gtuId);
        	laneChangeDirections.add(gtuDirection);
        	laneChangeLinks.add(gtuLink);
        	laneChangeFromLanes.add(gtuFromLane);
        }
       
    }
    
    
    /** Class for headway info */
    private class HeadwayInfo {
    	
    	// headway variables
    	private String gtuType = null;
    	private String leaderGtuType = null;
        private double headwayDistance = Double.NaN;
        private double headwayTime = Double.NaN;
        private double timeToCollision = Double.NaN;
        private boolean criticalTtc = false;
        
        // headway info constructor
        public HeadwayInfo(LaneBasedGtu gtu) {
        	
        	// get GTU perception
	        LaneBasedGtu laneBasedGtu = (LaneBasedGtu) gtu;
	        this.gtuType = gtu.getType().getId();
	        CategoricalLanePerception perception = (CategoricalLanePerception) laneBasedGtu.getTacticalPlanner().getPerception();
	        DirectNeighborsPerception neighborPerception = perception.getPerceptionCategoryOrNull(DirectNeighborsPerception.class);
	        
	        // get GTU reaction time
	        Duration reactionTime = gtu.getParameters().getParameterOrNull(ParameterTypes.TR);
	        
	        if (neighborPerception != null) {
	        	// get headway to leader
	        	PerceptionCollectable<HeadwayGtu,LaneBasedGtu> perceptionCollectable = neighborPerception.getLeaders(RelativeLane.CURRENT);
	        	HeadwayGtu headway = perceptionCollectable.first();
	        	if (headway != null) {
	        		this.leaderGtuType = headway.getGtuType().getId();
		            double distance = headway.getDistance().si;
		            this.headwayDistance = distance;
		            double speed = laneBasedGtu.getSpeed().si;
		            double leaderSpeed = headway.getSpeed().si;
		            
		            // check NaN values, 
		            if (!Double.isNaN(distance) && !Double.isNaN(speed) && !Double.isNaN(leaderSpeed)) {
			            // calculate headway time (brick wall principle)
			            double hwt = distance / speed;
			            this.headwayTime = hwt;
			            // calculate time-to-collision (relative)
			            if (speed > leaderSpeed) {
			            	double ttc = distance / (speed - leaderSpeed);
			            	this.timeToCollision = ttc;
			            	// determine whether ttc is critical
				            if (reactionTime != null) {
				            	if (ttc < reactionTime.si) {
				            		this.criticalTtc = true;
				            	}
				            }
			            }
		            }
	        	}
	        }
        }
        
        public String getGtuType() {
            return gtuType;
        }
        
        public String getLeaderGtuType() {
            return leaderGtuType;
        }

        public double getHeadwayDistance() {
            return headwayDistance;
        }

        public double getHeadwayTime() {
            return headwayTime;
        }

        public double getTtc() {
            return timeToCollision;
        }
        
        public boolean getCriticalTtc() {
            return criticalTtc;
        }

        @Override
        public String toString() {
            return "HeadwayInfo{" +
            	   ", gtuType=" + gtuType +
            	   ", headwayGtuType=" + leaderGtuType +
                   ", headwayDistance=" + headwayDistance +
                   ", ttc=" + timeToCollision +
                   ", criticalTtc=" + criticalTtc +
                   "}";
        }
    }

}
