
#include <algorithm>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <sys/time.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf/transform_listener.h>


using namespace cv;
using namespace std;

static const std::string win = "Optical Flow";

class OptFlow
{
  cv::Mat prevImg, prevH, img;
  vector<Point2f> prevPoints, prev3d, orig3d,flow_optflow;
  ros::Time prevT;

  const int _min_kp, _max_kp, _min_match;
  int _seq;
  Size _subPixWinSz, _winSz;
  TermCriteria _termcrit;
  bool _pubZero;
  double _thres, _altitude, _x, _y,x_optflow,y_optflow;

  const bool _display;
 double wall_0=get_wall_time();



  image_geometry::PinholeCameraModel _cam_model;
  const bool _cam_info_is_useful;
  ros::Publisher *_pub;
  tf::TransformListener tf_listener;


public:

  OptFlow(ros::Publisher *pub, /*string caminfo_topic*/sensor_msgs::CameraInfoConstPtr cam_info,
          bool display=false, double threshold=0.001, int min_kp=50, int max_kp=200, int min_match=10, int winSize=21)
    : _pub(pub), _display(display), _thres(threshold), _min_kp(min_kp), _max_kp(max_kp), _min_match(min_match),
      _seq(0), _pubZero(0), _cam_info_is_useful(!(cam_info->distortion_model.empty()))
  {
    _winSz = Size(winSize, winSize);
    _subPixWinSz = Size(winSize>>1, winSize>>1);
    _termcrit = TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03);

    _cam_model.fromCameraInfo( cam_info );
    if ( !_cam_model.initialized() ) ROS_ERROR("Optical Flow: error creating pinhole camera model");

    _altitude=1.0;  _x=0;  _y=0;

    if (_display) namedWindow(win);
  }

  ~OptFlow()
  {
    if (_display) destroyWindow(win);
  }

  void altitude_cb(const sensor_msgs::RangeConstPtr &msg) {
    _altitude = msg->range;
  }

/*Core OpenCV functions used:
  goodFeaturesToTrack(InputArray image, OutputArray corners, int maxCorners, double qualityLevel, double minDistance,
                      InputArray mask=noArray(), int blockSize=3, bool useHarrisDetector=false, double k=0.04 )
  calcOpticalFlowPyrLK(InputArray prevImg, InputArray nextImg, InputArray prevPts, InputOutputArray nextPts,
        OutputArray status, OutputArray err, Size winSize=Size(21,21), int maxLevel=3, TermCriteria criteria=
        TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01), int flags=0, double minEigThreshold=1e-4 )
  H = findHomography(InputArray srcPoints, InputArray dstPoints, int method=0, double ransacReprojThreshold=3,
                     OutputArray mask=noArray() )
*/
/*Optical Flow Hover procedure:
 * 1. Find up to _max_kp corner features in the previous image (goodFeaturesToTrack). Repeat until at least _min_kp
 *    features detected. Thus detection threshold and _min_kp should be as low as possible, since we have no motion
 *    estimate while repeating this step.
 * 2. Match patches in the current image to patches at the corner features in the previous image (calcOpticalFlowPyrLK)
 * 3. Prune the patch locations to those that were matched with LK.
 *    If # matches < _min_match, goto 1 to get new features if we haven't already done so, otherwise emit a warning.
 * 4. Use CameraInfo and, if available, altitude, to project the locations to coordinates on the ground plane.
 * 4. Compute homography, H, of the current matched points to the original locations of those points found in 1,
 *    filtering outliers with RANSAC (findHomography).
 *    If # inliers < _min_match, goto 1 to get new features if we haven't already so, otherwise emit a warning.
 * 5. dx = H(0,2) - H_prev(0,2),  dy=H(1,2) - H_prev(1,2).
 *    If dx or dy indicate a speed >3m/s, emit a warning.  Otherwise, x+=dx.  y+=dy.  H_prev = H.
 */
double get_wall_time(){
    struct timeval time;
    if (gettimeofday(&time,NULL)){
        //  Handle error
        return 0;
    }
    return (double)time.tv_sec + (double)time.tv_usec * .000001;
}
static void drawOptFlowMap(const Mat& flow, Mat& cflowmap, int step,
                    double, const Scalar& color)
{
    //double flow_x,flow_y=0.0;
    for(int y = 0; y < cflowmap.rows; y += step)
        for(int x = 0; x < cflowmap.cols; x += step)
        {
            const Point2f& fxy = flow.at<Point2f>(y, x);
            line(cflowmap, Point(x,y), Point(cvRound(x+fxy.x), cvRound(y+fxy.y)),
                 color);
            //flow_x+=fxy.x;
            //flow_y+=fxy.y;

            circle(cflowmap, Point(x,y), 1, color, -1);
        }
    //return flow_x/cflowmap.cols*cflowmap.rows,flow_y/cflowmap.cols*cflowmap.rows
}
  void img_cb(const sensor_msgs::ImageConstPtr & msg)
  {

    //ROS_INFO("in the callback so");
    vector<Point2f> currPoints, curr3d;  vector<uchar> match, match1;  vector<float> err;
    cv::Mat disp_img, resize_img = cv_bridge::toCvCopy(msg,"mono8")->image;
    cv::Size size(64,64);
    //cv::Size shift_area(1,1);
    //cv::Size search_area(4,4);
    //cv::Size block_size(8,8);
    cv::resize(resize_img,img,size);
    //CvMat *velx,*vely;
    //velx=cvCreateMat(8,8,CV_32FC1);
    //vely=cvCreateMat(8,8,CV_32FC1);
    //cvSetZero(velx);
    //cvSetZero(vely);
    //Ptr<DenseOpticalFlow> tvl1 = createOptFlow_DualTVL1();

    cv::Mat flow,cflow;
    double dt=msg->header.stamp.toSec()-prevT.toSec();
    prevT=msg->header.stamp;
    if(prevImg.data)
    {

        calcOpticalFlowFarneback(prevImg,img,flow,0.5,5,13,10,5,1.1,0);
        //cvCalcOpticalFlowBM(prevImg,img,block_size,shift_area,search_area,0,velx,vely);
        //calcOpticalFlowSF(prevImg,img,flow,3, 2, 4, 4.1, 25.5, 18, 55.0, 25.5, 0.35, 18, 55.0, 25.5, 10);
        //tvl1->calc(prevImg,img,flow);
        //cv::DenseOpticalFlow::calc(prevImg,img,flow);

        //For display
        /*
        cvtColor(prevImg,cflow,COLOR_GRAY2BGR);
        drawOptFlowMap(flow,cflow,16,1.5,Scalar(255,255,255));
        imshow(win,cflow);
        waitKey(1);
        */


        double flow_x,flow_y=0.0;
        int step=1.0;
        for(int y = 0; y < prevImg.rows; y += step)
        {
            for(int x = 0; x < prevImg.cols; x += step)
            {
                const Point2f& fxy = flow.at<Point2f>(y, x);
                //line(cflowmap, Point(x,y), Point(cvRound(x+fxy.x), cvRound(y+fxy.y)),
                //     color);
                // pass every point through camera model
                //Point3d curr=_cam_model.projectPixelTo3dRay(_cam_model.rectifyPoint(fxy));
                //curr3d.push_back(_altitude*Point2f(curr.x,curr.y));
                flow_x+=fxy.x;
                flow_y+=fxy.y;


                //circle(cflowmap, Point(x,y), 2, color, -1);
            }

        }
        flow_x=(flow_x/(prevImg.cols*prevImg.rows));
        flow_y=(flow_y/(prevImg.cols*prevImg.rows));
        //ROS_INFO("Flow compouted %f, %f",flow_x,flow_y);
        // in world coordinates


        //
        x_optflow+=flow_x;
        y_optflow+=flow_y;
        ROS_INFO("Position estimated in pixel space %f, %f",x_optflow,y_optflow);
        Point3d curr=_cam_model.projectPixelTo3dRay(_cam_model.rectifyPoint(Point2f(flow_x,flow_y)));
        //curr.x=curr.x*_altitude;
        //curr.y=curr.y*_altitude;

        ROS_INFO("Pose estimates %f , %f",curr.x,curr.y);





    }

    std::swap(prevImg,img);

    //imshow(win,flow);
    //waitKey(1);

}
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "optical_flow");
  ros::NodeHandle nh("~");
  image_transport::ImageTransport it(nh);
  string cam_topic, caminfo_topic, altitude_topic, pub_topic;
  bool display;
  double threshold;

  nh.param("cam_topic", cam_topic, std::string("/downward_cam/image_raw"));
  nh.param("caminfo_topic", caminfo_topic, std::string("/downward_cam/camera_info"));
  nh.param("altitude_topic", altitude_topic, std::string("/altitude"));
  nh.param("pub_topic", pub_topic, std::string("/optflow"));
  nh.param("display", display, false);
  nh.param("threshold", threshold, 0.001);

  sensor_msgs::CameraInfoConstPtr cam_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(caminfo_topic, nh);
  ros::Publisher pub = nh.advertise<geometry_msgs::PointStamped>(pub_topic, 1);
  //ros::Publisher pub = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>(pub_topic, 100);
  OptFlow optflow(&pub, cam_info/*caminfo_topic*/, display, threshold);
  ros::Subscriber altitude_sub = nh.subscribe(altitude_topic, 1, &OptFlow::altitude_cb, &optflow);
  image_transport::Subscriber it_sub = it.subscribe(cam_topic, 1, &OptFlow::img_cb, &optflow);
  ros::spin();
  return 0;
}
