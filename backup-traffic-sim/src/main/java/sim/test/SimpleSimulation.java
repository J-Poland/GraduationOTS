package sim.test;

import java.net.URL;
import java.rmi.RemoteException;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.Set;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;

import picocli.CommandLine.Option;

import org.djunits.unit.AccelerationUnit;
import org.djunits.unit.DurationUnit;
import org.djunits.unit.FrequencyUnit;
import org.djunits.unit.LengthUnit;
import org.djunits.unit.SpeedUnit;
import org.djunits.unit.TimeUnit;
import org.djunits.value.storage.StorageType;
import org.djunits.value.vdouble.scalar.Acceleration;
import org.djunits.value.vdouble.scalar.Duration;
import org.djunits.value.vdouble.scalar.Frequency;
import org.djunits.value.vdouble.scalar.Length;
import org.djunits.value.vdouble.scalar.Mass;
import org.djunits.value.vdouble.scalar.Speed;
import org.djunits.value.vdouble.scalar.Time;
import org.djunits.value.vdouble.vector.FrequencyVector;
import org.djunits.value.vdouble.vector.TimeVector;
import org.djunits.value.vdouble.vector.data.DoubleVectorData;
import org.djutils.cli.CliUtil;
import org.djutils.draw.DrawRuntimeException;
import org.djutils.io.URLResource;
import org.opentrafficsim.animation.colorer.LmrsSwitchableColorer;
import org.opentrafficsim.animation.gtu.colorer.GtuColorer;
import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.base.parameters.ParameterSet;
import org.opentrafficsim.base.parameters.ParameterTypes;
import org.opentrafficsim.core.definitions.Defaults;
import org.opentrafficsim.core.definitions.DefaultsNl;
import org.opentrafficsim.core.distributions.ConstantGenerator;
import org.opentrafficsim.core.distributions.Distribution;
import org.opentrafficsim.core.distributions.Generator;
import org.opentrafficsim.core.distributions.Distribution.FrequencyAndObject;
import org.opentrafficsim.core.distributions.ProbabilityException;
import org.opentrafficsim.core.dsol.OtsSimulatorInterface;
import org.opentrafficsim.core.geometry.OtsGeometryException;
import org.opentrafficsim.core.gtu.GtuCharacteristics;
import org.opentrafficsim.core.gtu.GtuException;
import org.opentrafficsim.core.gtu.GtuTemplate;
import org.opentrafficsim.core.gtu.GtuType;
import org.opentrafficsim.core.idgenerator.IdGenerator;
import org.opentrafficsim.core.network.NetworkException;
import org.opentrafficsim.road.gtu.generator.GeneratorPositions;
import org.opentrafficsim.road.gtu.generator.LaneBasedGtuGenerator;
import org.opentrafficsim.road.gtu.generator.TtcRoomChecker;
import org.opentrafficsim.road.gtu.generator.LaneBasedGtuGenerator.RoomChecker;
import org.opentrafficsim.road.gtu.generator.characteristics.DefaultLaneBasedGtuCharacteristicsGeneratorOd;
import org.opentrafficsim.road.gtu.generator.characteristics.DefaultLaneBasedGtuCharacteristicsGeneratorOd.Factory;
import org.opentrafficsim.road.gtu.generator.characteristics.LaneBasedGtuCharacteristics;
import org.opentrafficsim.road.gtu.generator.characteristics.LaneBasedGtuCharacteristicsGeneratorOd;
import org.opentrafficsim.road.gtu.generator.characteristics.LaneBasedGtuTemplate;
import org.opentrafficsim.road.gtu.generator.characteristics.LaneBasedGtuTemplateDistribution;
import org.opentrafficsim.road.gtu.lane.VehicleModel;
import org.opentrafficsim.road.gtu.lane.VehicleModelFactory;
import org.opentrafficsim.road.gtu.lane.tactical.LaneBasedTacticalPlannerFactory;
import org.opentrafficsim.road.gtu.lane.tactical.following.AbstractIdm;
import org.opentrafficsim.road.gtu.lane.tactical.following.CarFollowingModelFactory;
import org.opentrafficsim.road.gtu.lane.tactical.following.IdmPlus;
import org.opentrafficsim.road.gtu.strategical.LaneBasedStrategicalPlannerFactory;
import org.opentrafficsim.road.network.RoadNetwork;
import org.opentrafficsim.road.network.factory.xml.parser.XmlParser;
import org.opentrafficsim.road.network.lane.CrossSectionLink;
import org.opentrafficsim.road.network.lane.Lane;
import org.opentrafficsim.road.network.lane.LanePosition;
import org.opentrafficsim.road.network.lane.object.SpeedSign;
import org.opentrafficsim.road.od.Categorization;
import org.opentrafficsim.road.od.Category;
import org.opentrafficsim.road.od.Interpolation;
import org.opentrafficsim.road.od.OdMatrix;

import nl.tudelft.simulation.dsol.SimRuntimeException;
import nl.tudelft.simulation.jstats.distributions.DistNormal;
import nl.tudelft.simulation.jstats.distributions.DistUniform;
import nl.tudelft.simulation.jstats.distributions.unit.DistContinuousMass;
import nl.tudelft.simulation.jstats.streams.MersenneTwister;
import nl.tudelft.simulation.jstats.streams.StreamInterface;

import org.opentrafficsim.core.network.Node;
import org.opentrafficsim.core.network.route.ProbabilisticRouteGenerator;
import org.opentrafficsim.core.network.route.Route;
import org.opentrafficsim.core.parameters.ParameterFactory;
import org.opentrafficsim.core.parameters.ParameterFactoryByType;
import org.opentrafficsim.core.units.distributions.ContinuousDistDoubleScalar;
import org.opentrafficsim.road.gtu.lane.tactical.following.IdmPlusFactory;
import org.opentrafficsim.road.gtu.lane.perception.PerceptionFactory;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.LmrsFactory;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.Cooperation;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.GapAcceptance;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.LmrsParameters;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.MandatoryIncentive;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.Synchronization;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.Tailgating;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.VoluntaryIncentive;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.AccelerationConflicts;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.AccelerationIncentive;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.AccelerationSpeedLimitTransition;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.AccelerationTrafficLights;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.DefaultLmrsPerceptionFactory;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.IncentiveCourtesy;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.IncentiveKeep;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.IncentiveRoute;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.IncentiveSocioSpeed;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.IncentiveSpeedWithCourtesy;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.Lmrs;
import org.opentrafficsim.road.gtu.strategical.LaneBasedStrategicalRoutePlannerFactory;

// @docs/08-tutorials/simulation-setup.md#how-to-set-up-a-simulation
public class SimpleSimulation extends MyAbstractSimulationScript
{
	@Option(names = "--output", description = "Generate output.", negatable = true, defaultValue = "false")
    private boolean output;
    
    @Option(names = {"--headless"}, description = "Run simulation without animation.", negatable = false,
            defaultValue = "false")
    private boolean headless;
    
    @Option(names = "--truckFraction", description = "Fraction of trucks.", defaultValue = "0.2")
    private double truckFraction;
	
	// global variables
	static Time simTime;
	
	/** */
    private static final long serialVersionUID = 20170407L;

    /** Network. */
    static final String NETWORK = "shortMerge";
    
    /** Left traffic fraction. */
    static final double LEFT_FRACTION = 0.8;

    /** Main demand per lane. */
    static final Frequency MAIN_DEMAND = new Frequency(1000, FrequencyUnit.PER_HOUR);

    /** Ramp demand. */
    static final Frequency RAMP_DEMAND = new Frequency(500, FrequencyUnit.PER_HOUR);

    /** Synchronization. */
    static final Synchronization SYNCHRONIZATION = Synchronization.ALIGN_GAP;

    /** Cooperation. */
    static final Cooperation COOPERATION = Cooperation.PASSIVE_MOVING;

    /** Use additional incentives. */
    static final boolean ADDITIONAL_INCENTIVES = true;


    protected SimpleSimulation()
    {
		super("Simple simulation", "Example simple simulation");
    }

    public static void main(final String[] args) throws Exception
    {
        SimpleSimulation simpleSimulation = new SimpleSimulation();
        CliUtil.execute(simpleSimulation, args);
        
        // get simulation time
        Time startTime = Time.instantiateSI(0);
        Duration simDuration = simpleSimulation.getSimulationTime();
        simTime = startTime.plus(simDuration);
        
        // Set simulation parameters
        simpleSimulation.setAutoRun(simpleSimulation.headless); // Set auto run to true for headless simulation
        
        // Check parameters
        boolean setRun = simpleSimulation.isAutorun();
        System.out.println(setRun);
        
        // Run simulation
        simpleSimulation.start();
    }

    // @docs/08-tutorials/simulation-setup.md#how-to-set-up-a-simulation
    @Override
    protected RoadNetwork setupSimulation(final OtsSimulatorInterface sim)
            throws NullPointerException, DrawRuntimeException, NetworkException, OtsGeometryException
    {
    	try
        {
            URL xmlURL = URLResource.getResource("/networks/" + NETWORK + ".xml");
            this.network = new RoadNetwork("ShortMerge", getSimulator());
            new XmlParser(this.network).setUrl(xmlURL).build();
            
            // System.out.println(this.network.getNodeMap().get("A"));
            
            // setup the ODMatrix
//            OdMatrix odMatrix = createDemandMatrix();
            
        }
        catch (Exception exception)
        {
            exception.printStackTrace();
        }
        return network;
    }
    
    public OdMatrix createDemandMatrix() {
    	// Define network nodes
        List<Node> origins = new ArrayList<>();
        Node nodeA = this.network.getNodeMap().get("A");
        Node nodeF = this.network.getNodeMap().get("F");
        origins.add(nodeA);
        origins.add(nodeF);
        List<Node> destinations = new ArrayList<>();
        Node nodeE = this.network.getNodeMap().get("E");
        destinations.add(nodeE);
        
        // Create categorisation
        Categorization categorization = new Categorization("MyCategorization", GtuType.class);
        
        // Stepwise demand with half-hour periods
        DoubleVectorData data = DoubleVectorData.instantiate(new double[] {0.0, 0.5, 1.0}, TimeUnit.BASE_HOUR.getScale(), StorageType.DENSE);
        TimeVector timeVector = new TimeVector(data, TimeUnit.BASE_HOUR);
        Interpolation interpolation = Interpolation.STEPWISE;
        
        // Resulting ODMatrix without demand
        OdMatrix odMatrix = new OdMatrix("MyOD", origins, destinations, categorization, timeVector, interpolation);
        
        // Define GTU demand categories
        Category carCategory = new Category(categorization, DefaultsNl.CAR);
        Category truckCategory = new Category(categorization, DefaultsNl.TRUCK);
        
        // Set car demand AE
        data = DoubleVectorData.instantiate(new double[] {1000.0, 2000.0, 0.0}, FrequencyUnit.PER_HOUR.getScale(),
                StorageType.DENSE);
        FrequencyVector demandAECar = new FrequencyVector(data, FrequencyUnit.PER_HOUR);
        odMatrix.putDemandVector(nodeA, nodeE, carCategory, demandAECar);
        
        // Set truck demand AE
        data = DoubleVectorData.instantiate(new double[] {0.0, 1.0}, TimeUnit.BASE_HOUR.getScale(), StorageType.DENSE);
        TimeVector truckTime = new TimeVector(data, TimeUnit.BASE_HOUR);
        data = DoubleVectorData.instantiate(new double[] {100.0, 150.0}, FrequencyUnit.PER_HOUR.getScale(), StorageType.DENSE);
        FrequencyVector demandAETruck = new FrequencyVector(data, FrequencyUnit.PER_HOUR);
        odMatrix.putDemandVector(nodeA, nodeE, truckCategory, demandAETruck, truckTime, Interpolation.LINEAR);
        
        data = DoubleVectorData.instantiate(new double[] {1200.0, 1500.0, 0.0}, FrequencyUnit.PER_HOUR.getScale(),
                StorageType.DENSE);
        FrequencyVector demandFE = new FrequencyVector(data, FrequencyUnit.PER_HOUR);
        odMatrix.putDemandVector(nodeF, nodeE, carCategory, demandFE, timeVector, interpolation, 0.9);
        odMatrix.putDemandVector(nodeF, nodeE, truckCategory, demandFE, timeVector, interpolation, 0.1);
        
        return odMatrix;
    }
    
    LaneBasedGtuCharacteristicsGeneratorOd characteristicsGenerator = new LaneBasedGtuCharacteristicsGeneratorOd()
    {
        @Override
        public LaneBasedGtuCharacteristics draw(final Node origin, final Node destination, final Category category,
                final StreamInterface randomStream) throws GtuException
        {
            // implementation code
        	GtuType gtuType = category.get(GtuType.class);
            Route route = category.get(Route.class);
            GtuCharacteristics gtuCharacteristics = GtuType.defaultCharacteristics(gtuType, origin.getNetwork(), randomStream);
            VehicleModel vehicleModel = VehicleModel.NONE;
            
            CarFollowingModelFactory<?> carFollowing = new IdmPlusFactory(randomStream);
            PerceptionFactory perception = new DefaultLmrsPerceptionFactory();
            LaneBasedTacticalPlannerFactory<?> tactical = new LmrsFactory(carFollowing, perception);
            LaneBasedStrategicalPlannerFactory<?> strategical = new LaneBasedStrategicalRoutePlannerFactory(tactical);
            
            return new LaneBasedGtuCharacteristics(gtuCharacteristics, strategical, route, origin, destination, vehicleModel);
        }
    };
    
}
    
    