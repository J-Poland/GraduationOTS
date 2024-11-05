package sim.demo;

import java.awt.Color;
import java.awt.Dimension;
import java.rmi.RemoteException;

import javax.naming.NamingException;

import org.djunits.unit.SpeedUnit;
import org.djunits.value.vdouble.scalar.Acceleration;
import org.djunits.value.vdouble.scalar.Duration;
import org.djunits.value.vdouble.scalar.Speed;
import org.djunits.value.vdouble.scalar.Time;
import org.opentrafficsim.animation.colorer.FixedColor;
import org.opentrafficsim.animation.colorer.IncentiveColorer;
import org.opentrafficsim.animation.colorer.SocialPressureColorer;
import org.opentrafficsim.animation.colorer.TaskSaturationColorer;
import org.opentrafficsim.animation.gtu.colorer.AccelerationGtuColorer;
import org.opentrafficsim.animation.gtu.colorer.GtuColorer;
import org.opentrafficsim.animation.gtu.colorer.SpeedGtuColorer;
import org.opentrafficsim.animation.gtu.colorer.SwitchableGtuColorer;
import org.opentrafficsim.core.dsol.OtsAnimator;
import org.opentrafficsim.draw.OtsDrawingException;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.IncentiveSocioSpeed;
import org.opentrafficsim.swing.gui.OtsAnimationPanel;
import org.opentrafficsim.swing.gui.OtsSimulationApplication;

import nl.tudelft.simulation.dsol.SimRuntimeException;
import nl.tudelft.simulation.language.DsolException;
import sim.demo.vehicleconfigurations.VehicleAutomationConfigurations;


public class VehicleAutomationApplication extends OtsSimulationApplication<VehicleAutomationModel> {
	
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

	/**
     * Constructor.
     * @param model model
     * @param panel panel
     * @throws OtsDrawingException on animation error
     */
    private VehicleAutomationApplication(final VehicleAutomationModel model, final OtsAnimationPanel panel) throws OtsDrawingException
    {
        super(model, panel);
    }
    
    /**
     * Start method to invoke the simulation.
     * 
     * @param simConfig; containing input parameters
     */
    public static void start(VehicleAutomationModelParameters simConfig)
    {
    	main(simConfig);
    }

	/**
     * Main program.
     * @param simConfig
     */
    public static void main(VehicleAutomationModelParameters simConfig)
    {
        try
        {
            OtsAnimator simulator = new OtsAnimator("VehicleAutomationSimulation");
            final VehicleAutomationModel otsModel = new VehicleAutomationModel(simulator, simConfig);
            double simulationTime = simConfig.getWarmUpTime() + simConfig.getSampleTime();
            simulator.initialize(Time.ZERO, Duration.ZERO, Duration.instantiateSI(simulationTime), otsModel);
            // Note some relevant colorers for social interactions and task saturation
            GtuColorer colorer = new SwitchableGtuColorer(0, new FixedColor(Color.BLUE, "Blue"),
            		new AutomationColorer(VehicleAutomationConfigurations.GTU_TYPE_COLORS),
                    new SpeedGtuColorer(new Speed(150.0, SpeedUnit.KM_PER_HOUR)),
                    new AccelerationGtuColorer(Acceleration.instantiateSI(-4.0), Acceleration.instantiateSI(2.0)),
                    new SocialPressureColorer(), new IncentiveColorer(IncentiveSocioSpeed.class), new TaskSaturationColorer());
            OtsAnimationPanel animationPanel = new OtsAnimationPanel(otsModel.getNetwork().getExtent(),
                    new Dimension(800, 600), simulator, otsModel, colorer, otsModel.getNetwork());
            new VehicleAutomationApplication(otsModel, animationPanel);
            animationPanel.enableSimulationControlButtons();
        }
        catch (SimRuntimeException | NamingException | RemoteException | OtsDrawingException | DsolException exception)
        {
            exception.printStackTrace();
        }
    }
    
}
