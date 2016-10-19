package com.example.chongshao_mikasa.airport2;

import android.Manifest;
import android.app.Activity;
import android.content.Context;
import android.content.pm.PackageManager;
import android.graphics.BitmapFactory;
import android.graphics.drawable.BitmapDrawable;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.opengl.Matrix;
import android.os.Environment;
import android.support.v4.app.ActivityCompat;
import android.support.v7.app.AppCompatActivity;
import android.os.Build;
import android.os.Bundle;
import android.util.Xml;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import android.widget.Button;
import android.widget.ImageView;
import android.util.Log;
import android.content.Intent;
import android.net.Uri;
import android.database.Cursor;
import android.graphics.Bitmap;
import android.provider.MediaStore;
// stereo
import android.hardware.Camera;
//import android.hardware.Camera.Size;
import android.widget.TextView;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;
import java.util.UUID;

// opencv
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Core;
import org.opencv.calib3d.Calib3d;
import org.opencv.calib3d.StereoBM;
import org.opencv.calib3d.StereoSGBM;

import com.estimote.sdk.BeaconManager;
import com.estimote.sdk.Region;
import com.estimote.sdk.SystemRequirementsChecker;

// TODO: saving the camera iamge
// TODO: saving the depth map

//public class MainActivity extends AppCompatActivity implements SurfaceHolder.Callback {
public class MainActivity extends AppCompatActivity implements SensorEventListener {

    // water shed related stuff
    protected static final String TAG = null;
    private static int RESULT_LOAD_IMAGE = 1;
    // storage permissions
    private static final int REQUEST_EXTERNAL_STORAGE = 1;
    private static String[] PERMISSIONS_STORAGE = {
            Manifest.permission.READ_EXTERNAL_STORAGE,
            Manifest.permission.WRITE_EXTERNAL_STORAGE
    };

    public Mat img = new Mat();
    public Mat result = new Mat();
    static {
        if (!OpenCVLoader.initDebug()) {
            // Handle initialization error
        }
    }

    // beacon ranging relatd stuff
    private BeaconManager beaconManager;
    private Region region;

    Mat imgs, gray;

    // camera related stuff
    private static final int CAMERA_REQUEST = 1888;
    private ImageView leftImageView;
    private ImageView rightImageView;

    Button leftSaveButton;
    Button rightSaveButton;

    private TextView debug;

    private boolean isLeft;

    Mat leftImage = new Mat();
    Mat rightImage = new Mat();

    // IMU related stuff
    private SensorManager mSensorManager;
    private Sensor mRotationVectorSensor, mAccelerationSensor;

    private final float[] mRotationMatrix = new float[16];
    private volatile float[] mAccelerometerMatrix = new float[4];

    private float rotationX = 0;
    private float rotationY = 0;
    private float rotationZ = 0;
    private float accelX = 0;
    private float accelY = 0;
    private float accelZ = 0;

    private float leftRX = 0;
    private float leftRY = 0;
    private float leftRZ = 0;
    private float leftAX = 0;
    private float leftAY = 0;
    private float leftAZ = 0;

    private float rightRX = 0;
    private float rightRY = 0;
    private float rightRZ = 0;
    private float rightAX = 0;
    private float rightAY = 0;
    private float rightAZ = 0;


    private int rightIndex = 0;
    private String currLeftName = "";

    private TextView rotationMsg;

    // text write related stuff
    private Button writeButton;

    public static void verifyStoragePermissions(Activity activity) {
        // Check if we have write permission
        int permission = ActivityCompat.checkSelfPermission(activity, Manifest.permission.WRITE_EXTERNAL_STORAGE);

        if (permission != PackageManager.PERMISSION_GRANTED) {
            // We don't have permission so prompt the user
            ActivityCompat.requestPermissions(
                    activity,
                    PERMISSIONS_STORAGE,
                    REQUEST_EXTERNAL_STORAGE
            );
        }
    }

    private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS:
                {
                    Log.i(TAG, "OpenCV loaded successfully");
                } break;
                default:
                {
                    super.onManagerConnected(status);
                } break;
            }
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        verifyStoragePermissions(this);
        super.onCreate(savedInstanceState);
        OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_1_0, this, mLoaderCallback);
        setContentView(R.layout.activity_main);

        Button buttonLoadImage = (Button)findViewById(R.id.buttonLoadPicture);

        buttonLoadImage.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View arg0) {
                Intent i = new Intent(
                        Intent.ACTION_PICK,
                        android.provider.MediaStore.Images.Media.EXTERNAL_CONTENT_URI);
                startActivityForResult(i, RESULT_LOAD_IMAGE);
            }
        });

        Button buttonStereo = (Button)findViewById(R.id.buttonStereo);
        buttonStereo.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View arg0) {
                    Mat disparity = createDisparityMap(leftImage, rightImage);

                    ImageView imageViewStereo = (ImageView)findViewById(R.id.imgViewStereo);
                    Bitmap dispBit = Bitmap.createBitmap(leftImage.width(), leftImage.height(), Bitmap.Config.ARGB_8888);
                    Log.i(TAG, "disparity type");
                    Log.i(TAG,  String.valueOf(disparity.type()));
                    Log.i(TAG,  String.valueOf(CvType.CV_8UC1));
                    Log.i(TAG,  String.valueOf(CvType.CV_8UC3));
                    Log.i(TAG,  String.valueOf(CvType.CV_8UC4));
                    Mat disparity2 = new Mat(disparity.size(), CvType.CV_8UC1);
                    Core.MinMaxLocResult minmax = Core.minMaxLoc(disparity);
                    disparity.convertTo(disparity2, CvType.CV_8UC1, 255.0/(minmax.maxVal-minmax.minVal));
                    Utils.matToBitmap(disparity2, dispBit, true);
                    Log.i(TAG, "all okay");
                    imageViewStereo.setImageBitmap(dispBit);
            }
        });

        beaconManager = new BeaconManager(this);
        region = new Region("ranged region",
                UUID.fromString("B9407F30-F5F8-466E-AFF9-25556B57FE6D"), null, null);

        // camera related stuff
        this.leftImageView = (ImageView)this.findViewById(R.id.leftImageView);
        Button leftImageButton = (Button)this.findViewById(R.id.leftImageButton);
        leftImageButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                isLeft = true;

                Log.i(TAG, "left camera button clicked!");
                debug.setText("left camera button clicked!");
                Intent cameraIntent = new Intent(android.provider.MediaStore.ACTION_IMAGE_CAPTURE);
                startActivityForResult(cameraIntent, CAMERA_REQUEST);
            }
        });

        this.rightImageView = (ImageView)this.findViewById(R.id.rightImageView);
        Button rightImageButton = (Button)this.findViewById(R.id.rightImageButton);
        rightImageButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                isLeft =false;

                debug.setText("right camera button clicked!");
                Intent cameraIntent = new Intent(android.provider.MediaStore.ACTION_IMAGE_CAPTURE);
                startActivityForResult(cameraIntent, CAMERA_REQUEST);
            }
        });

        // saving image related stuff
        leftSaveButton = (Button)this.findViewById(R.id.leftSaveButton);
        leftSaveButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                debug.setText("left save button clicked!");
                saveLeftSensorData();
                Bitmap saveBitMap = matToBitMap(leftImage, leftImage.width());
                storeImage(saveBitMap);
                MediaStore.Images.Media.insertImage(getContentResolver(),
                        saveBitMap, "foo" , "bar");
            }
        });

        rightSaveButton = (Button)this.findViewById(R.id.rightSaveButton);
        rightSaveButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                debug.setText("right save button clicked!");
                saveRightSensorData();
                Bitmap saveBitMap = matToBitMap(rightImage, rightImage.width());
                storeImage(saveBitMap);

                MediaStore.Images.Media.insertImage(getContentResolver(),
                            saveBitMap, "foo2", "bar2"); //TODO: change this file name later
            }
        });

        // debug related stuff
        debug = (TextView)this.findViewById(R.id.debugmsg);

        // IMU related stuff
        mSensorManager = (SensorManager)getSystemService(SENSOR_SERVICE);
        mRotationVectorSensor = mSensorManager.getDefaultSensor(
                Sensor.TYPE_ROTATION_VECTOR);
        mAccelerationSensor = mSensorManager.getDefaultSensor(
                Sensor.TYPE_ACCELEROMETER);

        mSensorManager.registerListener(this, mRotationVectorSensor, 10000);
        mSensorManager.registerListener(this, mAccelerationSensor, 5000);
        rotationMsg = (TextView)this.findViewById(R.id.rotationMsg);

        // file write related stuff
        writeButton = (Button)this.findViewById(R.id.writeButton);
        writeButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {

                String filename = "mythirdfile";
                debug.setText("hi3");

                String outputString = "Hello world!";
                File myDir = getFilesDir();
                debug.setText("hi2");

                try {
                    File secondFile = new File(myDir + "/text/", filename);
                    debug.setText(secondFile.toURI().toString());
                        FileOutputStream fos = new FileOutputStream(secondFile);

                        fos.write(outputString.getBytes());
                        fos.flush();
                        fos.close();

                } catch (Exception e) {
                    debug.setText(e.getMessage());
                    e.printStackTrace();
                }
            }
        });
    }

    public void saveLeftSensorData() {
        Long tsLong = System.currentTimeMillis()/1000;
        String ts = tsLong.toString();
        currLeftName = ts;
        String filename = "sensorL"+ts+".txt";
        String outputString = String.valueOf(leftRX) + "," + String.valueOf(leftRY) + "," +
                String.valueOf(leftRZ) + "," + String.valueOf(leftAX) + "," +
                String.valueOf(leftAY) + "," + String.valueOf(leftAZ) + "\n";

        rightIndex = 0;
        File myDir = getFilesDir();

        try {
            File recordFile = new File(myDir + "/text/", filename);
            debug.setText(recordFile.toURI().toString());
            FileOutputStream fos = new FileOutputStream(recordFile);
            fos.write(outputString.getBytes());
            fos.flush();
            fos.close();
        } catch (Exception e) {
            debug.setText(e.getMessage());
            e.printStackTrace();
        }
    }

    public void saveRightSensorData() {
        String filename = "sensor"+currLeftName+"r"+String.valueOf(rightIndex)+".txt";
        rightIndex = rightIndex+1;
        String outputString = String.valueOf(rightRX-leftRX) + "," + String.valueOf(rightRY-leftRY) + "," +
                String.valueOf(rightRZ-leftRZ) + "," + String.valueOf(rightAX) + "," +
                String.valueOf(rightAY) + "," + String.valueOf(rightAZ) + "\n";
        File myDir = getFilesDir();

        try {
            File recordFile = new File(myDir + "/text/", filename);
            debug.setText(recordFile.toURI().toString());
            FileOutputStream fos = new FileOutputStream(recordFile);
            fos.write(outputString.getBytes());
            fos.flush();
            fos.close();
        } catch (Exception e) {
            debug.setText(e.getMessage());
            e.printStackTrace();
        }
    }

    public void onSensorChanged(SensorEvent event) {
        // we received a sensor event. it is a good practice to check
        // that we received the proper event

        if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
            mAccelerometerMatrix[0] = event.values[0];
            mAccelerometerMatrix[1] = event.values[1];
            mAccelerometerMatrix[2] = event.values[2];
            mAccelerometerMatrix[3] = 0;

            accelX = event.values[0];
            accelY = event.values[1];
            accelZ = event.values[2];
        }

        if (event.sensor.getType() == Sensor.TYPE_ROTATION_VECTOR) {
            // convert the rotation-vector to a 4x4 matrix. the matrix
            // is interpreted by Open GL as the inverse of the
            // rotation-vector, which is what we want.

            SensorManager.getRotationMatrixFromVector(
                    mRotationMatrix, event.values);
            rotationX = event.values[0];
            rotationY = event.values[1];
            rotationZ = event.values[2];

            rotationMsg.setText("rotation x: " + String.valueOf(rotationX) + " y: " +
                    String.valueOf(rotationY) + " z: " + String.valueOf(rotationZ));
        }
    }

    public void onAccuracyChanged(Sensor sensor, int accuracy) {
    }

    // saving image related stuff
    private void storeImage(Bitmap image) {
        File pictureFile = getOutputMediaFile();
        if (pictureFile == null) {
            Log.d(TAG,
                    "Error creating media file, check storage permissions: ");
            debug.setText("Error creating media file");
            return;
        }
        try {
            FileOutputStream fos = new FileOutputStream(pictureFile);
            image.compress(Bitmap.CompressFormat.PNG, 90, fos);
            fos.close();
        } catch (FileNotFoundException e) {
            Log.d(TAG, "File not found: " + e.getMessage());
            debug.setText("file not found");
        } catch (IOException e) {
            Log.d(TAG, "Error accessing file: " + e.getMessage());
            debug.setText("error accessing file");
        }
    }

    private File getOutputMediaFile() {
        File mediaStorageDir = new File(Environment.getExternalStorageDirectory()
        + "/Android/data/"
    + getApplicationContext().getPackageName()
    + "/Files");

    if (!mediaStorageDir.exists()) {
        if (!mediaStorageDir.mkdirs()) {
            return null;
        }
    }
    String timeStamp = new SimpleDateFormat("ddMMyyyy_HHmm").format(new Date());
    File mediaFile;
    String mImageName = "MI_" + timeStamp + ".jpg";
    mediaFile = new File(mediaStorageDir.getPath() + File.separator + mImageName);
    return mediaFile;
}

    // stereo related stuff
    public Mat createDisparityMap(Mat rectLeft, Mat rectRight){

        // Converts the images to a proper type for stereoMatching
        Mat left = new Mat();
        Mat right = new Mat();
        Imgproc.cvtColor(rectLeft, left, Imgproc.COLOR_RGBA2GRAY);
        Mat left16 = new Mat();
        left.convertTo(left16, CvType.CV_8U);
        Imgproc.cvtColor(rectRight, right, Imgproc.COLOR_RGBA2GRAY);
        Mat right16 = new Mat();
        right.convertTo(right16, CvType.CV_8U);
        leftImageView.setImageBitmap(matToBitMap(left, rectLeft.width()));
        debug.setText(String.valueOf(rectLeft.channels())+":"+String.valueOf(left.channels()));

        // Create a new image using the size and type of the left image
        Mat disparity = new Mat(left16.size(), left16.type());

        int numDisparity = (int)(left16.size().width/9);

        StereoSGBM stereoAlgo = StereoSGBM.create(
                0,    // min DIsparities
                numDisparity, // numDisparities
                11,   // SADWindowSize
                2*11*11,   // 8*number_of_image_channels*SADWindowSize*SADWindowSize   // p1 2
                5*11*11,  // 8*number_of_image_channels*SADWindowSize*SADWindowSize  // p2 5
                -1,   // disp12MaxDiff
                63,   // prefilterCap
                10,   // uniqueness ratio
                0, // sreckleWindowSize
                32, // spreckle Range
                0); // TODO:chong how about different mode?

        // create the DisparityMap - SLOW: O(Width*height*numDisparity)
        stereoAlgo.compute(left16, right16, disparity);

        Core.normalize(disparity, disparity, 0, 256, Core.NORM_MINMAX);

        return disparity;
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        super.onActivityResult(requestCode, resultCode, data);
        if (requestCode == RESULT_LOAD_IMAGE && resultCode == RESULT_OK && null!= data) {
            Uri selectedImage = data.getData();
            String[] filePathColumn = {MediaStore.Images.Media.DATA};

            Cursor cursor = getContentResolver().query(selectedImage,
                    filePathColumn, null, null, null);
            cursor.moveToFirst();

            int columnIndex = cursor.getColumnIndex(filePathColumn[0]);
            String picturePath = cursor.getString(columnIndex);
            cursor.close();

            ImageView imageView = (ImageView)findViewById(R.id.imgView);
            Bitmap bmp = BitmapFactory.decodeFile(picturePath);
            Log.i(TAG, picturePath);
            Mat img = Imgcodecs.imread(picturePath);

            result = steptowatershed(img);
            Utils.matToBitmap(result, bmp, true);
            Log.i(TAG, "all okay");
            imageView.setImageBitmap(bmp);
        }
        if (requestCode == CAMERA_REQUEST && resultCode == RESULT_OK) {
            Bitmap photo = (Bitmap) data.getExtras().get("data");
            Mat cropMat = this.cropImage(photo);
            Bitmap cropPhoto = this.matToBitMap(cropMat, photo.getWidth());

            if (isLeft) {
                leftRX = rotationX;
                leftRY = rotationY;
                leftRZ = rotationZ;
                leftAX = accelX;
                leftAY = accelY;
                leftAZ = accelZ;
                Mat resizeImage = new Mat();
                Size sz = new Size(288,288);
                Imgproc.resize(cropMat, resizeImage, sz );

                leftImage = resizeImage;
                debug.setText(String.valueOf(cropMat.channels()));
                leftImageView.setImageBitmap(cropPhoto);
            } else {

                rightRX = rotationX;
                rightRY = rotationY;
                rightRZ = rotationZ;
                rightAX = accelX;
                rightAY = accelY;
                rightAZ = accelZ;
                Mat resizeImage = new Mat();
                Size sz = new Size(288,288);
                Imgproc.resize( cropMat, resizeImage, sz );

                rightImage = resizeImage;
                rightImageView.setImageBitmap(cropPhoto);
            }
        }
    }

    private Mat cropImage(Bitmap photo) {
        Mat imgMat = new Mat();
        Bitmap bmp32 = photo.copy(Bitmap.Config.ARGB_8888, true);
        Utils.bitmapToMat(bmp32, imgMat);
        int width = photo.getWidth();
        int height= photo.getHeight();
        Rect roi = new Rect(0, (height-width)/2, width, width);
        Mat imgMatCropped = new Mat(imgMat, roi);
        return imgMatCropped;
    }

    private Bitmap matToBitMap(Mat mat, int width) {
        Bitmap resultBmp = Bitmap.createBitmap(width, width, Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(mat, resultBmp);
        return resultBmp;
    }

    public Mat steptowatershed(Mat img) {
        Mat threeChannel = new Mat();

        Imgproc.cvtColor(img, threeChannel, Imgproc.COLOR_BGR2GRAY);
        Imgproc.threshold(threeChannel, threeChannel, 100, 255, Imgproc.THRESH_BINARY);

        Mat fg = new Mat(img.size(), CvType.CV_8U);
        Imgproc.erode(threeChannel, fg, new Mat());

        Mat bg = new Mat(img.size(), CvType.CV_8U);
        Imgproc.dilate(threeChannel, bg, new Mat());
        Imgproc.threshold(bg, bg, 1, 128, Imgproc.THRESH_BINARY_INV);

        Mat markers = new Mat(img.size(), CvType.CV_8U, new Scalar(0));
        Core.add(fg, bg, markers);
        Mat result1;
        WatershedSegmenter segmenter = new WatershedSegmenter();
        segmenter.setMarkers(markers);
        result1 = segmenter.process(img);

        return result1;
    }

    public class WatershedSegmenter {
        public Mat markers = new Mat();

        public void setMarkers(Mat markerImage) {
            markerImage.convertTo(markers, CvType.CV_32SC1);
        }

        public Mat process(Mat image) {
            Imgproc.watershed(image, markers);
            markers.convertTo(markers, CvType.CV_8U);
            return markers;
        }
    }

    @Override
    protected void onResume() {
        super.onResume();
        SystemRequirementsChecker.checkWithDefaultDialogs(this);
        OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_1_0, this, mLoaderCallback);

        beaconManager.connect(new BeaconManager.ServiceReadyCallback() {
            @Override
            public void onServiceReady() {
                beaconManager.startRanging(region);
            }
        });
    }

    @Override
    protected void onPause() {
        beaconManager.stopRanging(region);
        super.onPause();
    }
}