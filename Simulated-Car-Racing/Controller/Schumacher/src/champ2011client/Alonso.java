package champ2011client;

import java.io.File;

import champ2011client.behaviour.ClutchBehaviour;
import champ2011client.behaviour.StandardGearChangeBehaviour;

import matlabcontrol.MatlabConnectionException;
import matlabcontrol.MatlabInvocationException;
import matlabcontrol.MatlabProxy;
import matlabcontrol.MatlabProxyFactory;
import matlabcontrol.MatlabProxyFactoryOptions;
import matlabcontrol.extensions.MatlabTypeConverter;


public class Alonso extends Controller {

	private MatlabProxy proxy;
	private int t = 0;
	private int trial = 1;
	private int H = 100;
	
	protected StandardGearChangeBehaviour gearBehaviour = new StandardGearChangeBehaviour();
    protected ClutchBehaviour clutchBehaviour = new ClutchBehaviour();
	
	public Alonso() throws MatlabConnectionException, MatlabInvocationException {
		System.out.println("Starting MATLAB proxy");
		File location = new File("D:/Gebruikers/Derk/Mijn documenten/Studie/Research Project/Research-Project/Simulated-Car-Racing/Dynamics/DDP");
		
		// Starting a MATLAB proxy
		MatlabProxyFactoryOptions options = new MatlabProxyFactoryOptions.Builder()
			.setHidden(false)
			.setProxyTimeout(50000L)
			.setMatlabStartingDirectory(location)
			.setUsePreviouslyControlledSession(true)
			.build();
		MatlabProxyFactory factory = new MatlabProxyFactory(options);
		proxy = factory.getProxy();

		// Initialize new controller
	    proxy.eval("driver = Controller(" + H + ");");
	}

    public Action control(SensorModel sensorModel) {
    	Action action = new Action();

    	if (sensorModel.getCurrentLapTime() >= 0)
    		t++;
    	
    	//System.out.println(t);
    	if (t > H)
    		return action;
    	
    	try {
    		// Proxy sensor values in message string to MATLAB object
    		proxy.setVariable("message", sensorModel.getMessage());
    		proxy.eval("a = driver.control(message);");
    		
    		if (t == H) {
    			System.out.println("Restart requested...");
        		action.restartRace = true;
        		return action;
        	}
    		
    		MatlabTypeConverter processor = new MatlabTypeConverter(proxy);
    		double[][] controls = processor.getNumericArray("a").getRealArray2D();
    		
		    action.accelerate = controls[0][0];
		    action.brake = controls[1][0];
		    action.steering = controls[2][0];
		    
		    gearBehaviour.execute(sensorModel, action);
	        clutchBehaviour.execute(sensorModel, action);
		} catch (MatlabInvocationException e) {
			System.out.println("Error in MATLAB script.");
			action.restartRace = true;
		}

        return action;
    }

    public void reset() {
		System.out.println("Trial " + trial + " has ended. Restarting the race...");
		
		try {
			proxy.eval("driver.reset();");
		} catch (MatlabInvocationException e) {
			System.out.println("Error in MATLAB reset() method...");
			proxy.disconnect();
		}
		
		t = 0;
		trial++;
	}

	public void shutdown() {
		//Disconnect the proxy from MATLAB
		try {
			proxy.eval("driver.shutdown();");
		} catch (MatlabInvocationException e) {
			e.printStackTrace();
		}
		
		proxy.disconnect();
		System.out.println("Bye bye...");		
	}
}
