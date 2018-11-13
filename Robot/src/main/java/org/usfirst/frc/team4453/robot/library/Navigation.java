package org.usfirst.frc.team4453.robot.library;

import org.eclipse.paho.client.mqttv3.IMqttDeliveryToken;
import org.eclipse.paho.client.mqttv3.MqttCallback;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.eclipse.paho.client.mqttv3.MqttMessage;
import org.usfirst.frc.team4453.library.NavigationShared.Coordinate;

import edu.wpi.first.wpilibj.DriverStation;

public class Navigation implements MqttCallback {

    private static final String BROKER_URL = "tcp://raspberrypi.local:1883";

    private MqttClient client = null;

    private String status = new String(); 

    private Coordinate currentCoordinate;

    public Navigation() {
        try {
            client = new MqttClient(BROKER_URL, "Robot");
            client.subscribe("Vision/CurrentPosition");
            client.subscribe("Vision/Status");
        } catch (MqttException e) {
            DriverStation.reportError("Cannot start MQTT client: " + e.getMessage(), e.getStackTrace());
            client = null;
        }
    }

    public static double calculateDist(double x1, double y1, double x2, double y2){
    double x = x1 - x2;
    double y = y1 - y2;
    double dist = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    return dist;
    }

    public static double calculateCoordDist(Coordinate coord1, Coordinate coord2) {
        double x = coord1.X - coord2.X;
        double y = coord1.Y - coord2.Y;
        double coordDist = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        return coordDist;
    }
    
    public static double calculateAngle (double x1, double y1, double x2, double y2) {
        double x = x2 - x1;
        double y = y2 - y1;
        double dangle = 90 - Math.toDegrees(Math.atan2(y, x));
        return dangle;
    }

    public static double calculateCoordAngle(Coordinate coord1, Coordinate coord2) {
        double x = coord2.X - coord1.X;
        double y = coord2.Y - coord1.Y;
        double dangle = 90 - Math.toDegrees(Math.atan2(y, x));
        return dangle;
    }

    public boolean isOK() {
        return client != null && client.isConnected() && status.equals("OK");
    }

    public Coordinate getCurrentCoordinate() {
        return currentCoordinate;
    }

    @Override
    public void connectionLost(Throwable cause) {
        DriverStation.reportError("Lost connection to MQTT Broker: " + cause.getMessage(), false);
    }

    @Override
    public void messageArrived(String topic, MqttMessage message) throws Exception {
        if(topic.equals("Vision/CurrentPosition")) {
            currentCoordinate = Coordinate.deserialize(new String(message.getPayload()));
        } 
        else if(topic.equals("Vision/Status")) {
            status = new String(message.getPayload());
        }
    }

    @Override
    public void deliveryComplete(IMqttDeliveryToken token) {}
}