package sim.test;

import org.djunits.value.vdouble.scalar.Duration;
import org.opentrafficsim.core.dsol.OtsSimulatorInterface;
import org.opentrafficsim.road.network.RoadNetwork;
import org.opentrafficsim.swing.script.AbstractSimulationScript;

public abstract class MyAbstractSimulationScript extends AbstractSimulationScript {

    private static final long serialVersionUID = 20200129L;

    protected boolean autorun;
    protected RoadNetwork network;
    protected Duration simulationTime;

    protected MyAbstractSimulationScript(String name, String description) {
        super(name, description);
    }

    public void setAutoRun(boolean autorun) {
        this.autorun = autorun;
    }

    public boolean isAutorun() {
        return this.autorun;
    }

    public void setNetwork(RoadNetwork network) {
        this.network = network;
    }

    protected abstract RoadNetwork setupSimulation(OtsSimulatorInterface sim) throws Exception;
}


