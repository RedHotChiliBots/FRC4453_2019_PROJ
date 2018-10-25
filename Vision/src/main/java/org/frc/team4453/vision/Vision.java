package org.frc.team4453.vision;

import java.io.File;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.List;

import org.bytedeco.javacpp.Loader;
import org.bytedeco.javacpp.opencv_core.FileStorage;
import org.bytedeco.javacpp.opencv_java;
import org.eclipse.paho.client.mqttv3.IMqttDeliveryToken;
import org.eclipse.paho.client.mqttv3.MqttCallback;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttMessage;
import org.frc.team4453.library.Utils;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Point;
import org.opencv.core.Size;
import org.opencv.core.TermCriteria;
import org.opencv.features2d.FastFeatureDetector;
import org.opencv.highgui.HighGui;
import org.opencv.imgproc.Imgproc;
import org.opencv.utils.Converters;
import org.opencv.video.SparsePyrLKOpticalFlow;
import org.opencv.videoio.VideoCapture;

class Vision implements MqttCallback {

    private Mat camera_matrix = new Mat(); // TODO How to load these?
    private Mat dist_coeff = new Mat();

    private FastFeatureDetector detector = FastFeatureDetector.create(20, true, FastFeatureDetector.TYPE_9_16);
    private TermCriteria flow_criteria = new TermCriteria(TermCriteria.COUNT + TermCriteria.EPS, 30, 0.1);
    private SparsePyrLKOpticalFlow flow = SparsePyrLKOpticalFlow.create(new Size(21, 21), 3, flow_criteria, 0, 0.001);

    private boolean debug;

    public static void main(String[] args) {
        Loader.load(opencv_java.class);

        boolean debug = true;
        for (String arg : args) {
            if (arg.equalsIgnoreCase("--no-debug")) {
                debug = false;
            } else if (arg.equalsIgnoreCase("--debug")) {
                debug = true;
            }
        }

        new Vision(debug).run();
    }

    public Vision(boolean _debug) {
        debug = _debug;
    }

    private void run() {
        try {
            FileStorage fs = new FileStorage();
            fs.open("camera_calib.xml", FileStorage.READ);
            camera_matrix = new Mat(fs.get("cameraMatrix").mat().address());
            dist_coeff = new Mat(fs.get("dist_coeffs").mat().address());
            fs.close();

            MqttClient client = new MqttClient("tcp://localhost:1883", "Vision");
            client.setCallback(this);
            client.connect();
            client.subscribe("CurrentSpeed");

            VideoCapture camera = new VideoCapture(0);
            Mat frame = new Mat();
            Mat lastFrame = null;
            List<Point> lastPoints = null;
            Mat status = new Mat();

            Mat R = new Mat();
            Mat t = new Mat();

            if(debug) {
                HighGui.namedWindow("Camera");
            }

            while (true) {

                if(camera.isOpened() && camera.retrieve(frame))
                {
                    client.publish("Vision/Status", "OK".getBytes(), 1, false);
                    final long startTime = System.currentTimeMillis();

                    process_frame(frame, lastFrame, lastPoints, status, R, t);
                    client.publish("Vision/R", Utils.matToJson(R).getBytes(), 2, false);
                    client.publish("Vision/t", Utils.matToJson(t).getBytes(), 2, false);
                    final long endTime = System.currentTimeMillis();

                    client.publish("Vision/FrameTime", String.valueOf(endTime - startTime).getBytes(), 0, false);
                } else {
                    client.publish("Vision/Status", "NO_CAMERA".getBytes(), 1, false);
                    camera.open(0);
                }
            }

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private void process_frame(Mat frame, Mat lastFrame, List<Point> lastPoints, Mat status, Mat R, Mat t) {
        
        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_BGR2GRAY);

        if(debug) {
            HighGui.imshow("Camera", frame);
        }

        Mat undistorted = new Mat();
        Imgproc.undistort(frame, undistorted, camera_matrix, dist_coeff);

        MatOfKeyPoint keypoints = new MatOfKeyPoint();
        detector.detect(frame, keypoints);

        List<KeyPoint> keypoints_list = new ArrayList<KeyPoint>();
        Converters.Mat_to_vector_KeyPoint(keypoints, keypoints_list);

        List<Point> points_list = new ArrayList<Point>();

        for(KeyPoint kp : keypoints_list) {
            points_list.add(kp.pt);
        }

        if(lastFrame==null || lastPoints==null) {
            lastFrame = frame;
            lastPoints = points_list;
            return;
        }

        Mat lastPoints_mat = Converters.vector_Point2f_to_Mat(lastPoints);
        Mat points_list_mat = Converters.vector_Point2f_to_Mat(points_list);
        flow.calc(lastFrame, frame, lastPoints_mat, points_list_mat, status);

        Converters.Mat_to_vector_Point2f(lastPoints_mat, lastPoints);
        Converters.Mat_to_vector_Point2f(points_list_mat, points_list);

        List<Integer> status_list = new ArrayList<Integer>();
        Converters.Mat_to_vector_int(status, status_list);

        int indexCorrection = 0;
        for(int i=0; i < status_list.size(); i++) {  
            Point pt = points_list.get(i - indexCorrection);
     	    if ((status_list.get(i) == 0)||(pt.x<0)||(pt.y<0))	{
     		    if((pt.x<0)||(pt.y<0))	{
     		  	    status_list.set(i, 0);
     		    }
     		    lastPoints.remove(i - indexCorrection);
     		    points_list.remove(i - indexCorrection);
     		    indexCorrection++;
            }
        }

        lastPoints_mat = Converters.vector_Point2f_to_Mat(lastPoints);
        points_list_mat = Converters.vector_Point2f_to_Mat(points_list);

        Mat E = Calib3d.findEssentialMat(lastPoints_mat, points_list_mat);

        Calib3d.recoverPose(E, lastPoints_mat, points_list_mat, camera_matrix, R, t);
    }

    @Override
    public void connectionLost(Throwable cause) {
        System.out.println("Connection lost: " + cause.getMessage());
    }

    @Override
    public void messageArrived(String topic, MqttMessage message) throws Exception {
        
    }

    @Override
    public void deliveryComplete(IMqttDeliveryToken token) {

    }

      /**
     * Export a resource embedded into a Jar file to the local file path.
     *
     * @param resourceName ie.: "/SmartLibrary.dll"
     * @return The path to the exported resource
     * @throws Exception
     */
    static public String ExportResource(String resourceName) throws Exception {
        InputStream stream = null;
        OutputStream resStreamOut = null;
        String jarFolder;
        try {
            stream = Vision.class.getResourceAsStream(resourceName);//note that each / is a directory down in the "jar tree" been the jar the root of the tree
            if(stream == null) {
                throw new Exception("Cannot get resource \"" + resourceName + "\" from Jar file.");
            }

            int readBytes;
            byte[] buffer = new byte[4096];
            jarFolder = new File(Vision.class.getProtectionDomain().getCodeSource().getLocation().toURI().getPath()).getParentFile().getPath().replace('\\', '/');
            resStreamOut = new FileOutputStream(jarFolder + resourceName);
            while ((readBytes = stream.read(buffer)) > 0) {
                resStreamOut.write(buffer, 0, readBytes);
            }
        } catch (Exception ex) {
            throw ex;
        } finally {
            stream.close();
            resStreamOut.close();
        }

        System.out.println("Exported to: "+ jarFolder + resourceName);

        return jarFolder + resourceName;
    }
}