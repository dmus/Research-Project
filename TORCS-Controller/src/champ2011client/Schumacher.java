package champ2011client;

import champ2011client.behaviour.*;

/**
 * Our controller
 * @author Derk
 *
 */
public class Schumacher extends Controller {

    final double targetSpeed = 95;

    protected StandardGearChangeBehaviour gearBehaviour = new StandardGearChangeBehaviour();
    protected ClutchBehaviour clutchBehaviour = new ClutchBehaviour();
    
    protected float[][] actions = new float[15][2];
    
    public float getValueForNextState(SensorModel s, float[][] a) {
    	return 0.0f;
    }
    
    public Action control(SensorModel sensorModel) {
        Action action = new Action ();
        
        /*
        if (sensorModel.getSpeed () < targetSpeed) {
            action.accelerate = 1;
        }
        if (sensorModel.getAngleToTrackAxis() < 0) {
            action.steering = -0.1;
        }
        else {
            action.steering = 0.1;
        }
        */
        
        float velocity = 0;
        float steering = 0;
        
        
        
        action.accelerate = Math.max(velocity, 0.0);
        action.brake = Math.min(velocity, 0.0);
        action.steering = steering;
        
        gearBehaviour.execute(sensorModel, action);
        clutchBehaviour.execute(sensorModel, action);
        
        return action;
    }

    public void reset() {
		System.out.println("Restarting the race!");
	}

	public void shutdown() {
		System.out.println("Bye bye!");		
	}
}
