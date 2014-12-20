#include <algorithm>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <ros/time.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Range.h>

using namespace cv;
using namespace std;

static const std::string win = "Optical Flow";

class OptFlow
{
  cv::Mat prevImg, prevH;
  vector<Point2f> prevPoints, prev3d, orig3d;
  ros::Time prevT;
  
  const int _min_kp, _max_kp, _min_match;
  int _seq;
  Size _subPixWinSz, _winSz;
  TermCriteria _termcrit;
  bool _pubZero;
  double _thres, _altitude, _x, _y;

  const bool _display;
  
  image_geometry::PinholeCameraModel _cam_model;
  const bool _cam_info_is_useful;
  ros::Publisher *_pub;
  
  
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
    
    _altitude=1;  _x=0;  _y=0;
    
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
  void img_cb(const sensor_msgs::ImageConstPtr & msg)
  {
    vector<Point2f> currPoints, curr3d;  vector<uchar> match, match1;  vector<float> err;
    cv::Mat disp_img, img = cv_bridge::toCvCopy(msg,"mono8")->image;

    const bool reinit = prevPoints.size() < _min_kp;
    if (reinit) {
      prevT = msg->header.stamp;
      if (prevImg.empty()) { cv::swap(prevImg,img); ROS_INFO("OptFlow image rows=%i, cols=%i", img.rows, img.cols); return; }
      prevH = Mat::eye(3, 3, CV_32FC1);
      goodFeaturesToTrack(prevImg, prevPoints, _max_kp, 0.02, prevImg.cols >> 6); //min 10pix separation for 640x480
      if (prevPoints.size() < _min_kp) {
        //goodFeaturesToTrack(prevImg, prevPoints, _max_kp, 0.01, prevImg.cols >> 6);
        //if (prevPoints.size() < _min_kp) {
        //TODO: make use of time stamps to compensate for this situation by assuming velocity was constant.
        //      i.e. when we get enough features, scale delta position by delta time.
        ROS_WARN("\nOptical Flow: not enough (%i < %i) good features to track\n", prevPoints.size(), _min_kp);
        cv::swap(prevImg, img);
        return;
      }
      cornerSubPix(prevImg, prevPoints, _subPixWinSz, Size(-1,-1), _termcrit); //over-kill?
      prev3d.clear();  prev3d.resize(prevPoints.size());
      for( size_t i = 0; i < prevPoints.size(); i++ ) {
        if ( _cam_info_is_useful ) {
          Point3d prev = _cam_model.projectPixelTo3dRay( _cam_model.rectifyPoint(prevPoints[i]) );
          prev3d[i] = _altitude * Point2f(prev.x,prev.y);
        } else {
          prev3d[i] = (prevPoints[i] - Point2f(img.rows>>1, img.cols>>1)) * (1.0f/img.rows);
        }
      }
      orig3d = prev3d;
    }
    
    calcOpticalFlowPyrLK(prevImg, img, prevPoints, currPoints, match, err, _winSz, 3, _termcrit, 0, 1E-3);
    
    if (_display) img.copyTo(disp_img);
    size_t nmatch = 0;
    for( size_t i = 0; i < currPoints.size(); i++ ) {
      if ( !match[i] ) continue;
      if ( _cam_info_is_useful ) {
        Point3d curr = _cam_model.projectPixelTo3dRay( _cam_model.rectifyPoint(currPoints[i]) );
        curr3d.push_back(_altitude * Point2f(curr.x,curr.y));
      } else {
        curr3d.push_back((    currPoints[i]-Point2f(img.rows>>1, img.cols>>1)) * (1.0f/img.rows));
      }
      if ( _display ) {
        //circle( disp_img, currPoints[i], 2, Scalar(255), -1, 8);
        line( disp_img, prevPoints[i], currPoints[i], Scalar(255), 1);
      }
      currPoints[nmatch] = currPoints[i];  orig3d[nmatch] = orig3d[i];  prev3d[nmatch] = prev3d[i];
      nmatch++;
    }
    currPoints.resize(nmatch);  orig3d.resize(nmatch);  prev3d.resize(nmatch);
    
    cv::swap(prevImg, img);  prevPoints = currPoints;
    if (nmatch < _min_match) {
      if (reinit) { ROS_WARN("OptFlow: not enough LK matches to previous image, tracking lost."); } 
                   //reset _x,_y? H_prev?  publish previous dx,dy?
      else        { img_cb(msg); }
      return; 
    }
    
    //If Homography to last feature detection gives translation close to homography to previous image,
    //assume the former is ok and use it to reduce integration errors, otherwise use the latter.
    Mat H = findHomography(orig3d, curr3d, RANSAC, 4, match ),
        H1= findHomography(prev3d, curr3d, RANSAC, 4, match1);
    double dx = H.at<double>(0,2) - prevH.at<double>(0,2),
           dy = H.at<double>(1,2) - prevH.at<double>(1,2);
    if ( abs( H1.at<double>(0,2) - dx ) > .01 || abs( H1.at<double>(1,2) - dy ) > .01 ) {
      H = H1;  match = match1;  orig3d = prev3d;
      dx = H.at<double>(0,2);  dy = H.at<double>(1,2);
    }
    nmatch = 0;
    for( size_t i = 0; i < prev3d.size(); i++ ) {
      if ( !match[i] ) continue;
      if ( _display ) { circle( disp_img, currPoints[i], 2, Scalar(255), -1, 8); }
      currPoints[nmatch] = currPoints[i];  curr3d[nmatch] = curr3d[i];  orig3d[nmatch] = orig3d[i];
      nmatch++;
    }
    curr3d.resize(nmatch);  currPoints.resize(nmatch);  orig3d.resize(nmatch);
    prev3d = curr3d;
    
    //TODO find a better indicator that the homography is bogus than speed>.1m/frame (3m/s @ 30fps)
    if (nmatch < _min_match || dx*dx+dy*dy > .01) {
      if (reinit) { 
        //reset _x,_y? H_prev?  publish previous dx,dy?
        if (nmatch < _min_match) {
          ROS_WARN("OptFlow: Not enough Homography RANSAC matches to previous image, tracking lost.");
        } else {
          ROS_WARN("OptFlow: Homography indicates velocity > 3m/s (.1m/frame). Ignoring."); 
        }
      } else { 
        img_cb(msg); 
      }
      return;
    }

    _x += dx;  _y += dy;  prevH = H;
  /*  
    geometry_msgs::PointStamped p;
    p.header.seq = _seq++;
    p.header.stamp = msg->header.stamp; //ros::Time::now(); //
    //nmatch as surrogate for (inverse) variance ?
    p.point.x = _x; p.point.y = _y; p.point.z = _altitude;
    _pub->publish(p);
  */
    geometry_msgs::TwistWithCovarianceStamped twist;
    double dt = msg->header.stamp.toSec() - prevT.toSec();
    twist.header.seq = _seq++;
    twist.header.stamp = msg->header.stamp;
    twist.twist.twist.linear.x = dx / dt;
    twist.twist.twist.linear.y = dy / dt;
    //twist.twist.twist.angular.z =
    //twist.twist.covariance[36]
    twist.twist.covariance[0*6+0] = 0.025; //TODO: tune covariance
    twist.twist.covariance[1*6+1] = 0.025;
    for (int i=2; i<6; i++) twist.twist.covariance[i*6+i] = 1000.;
    _pub->publish(twist);
    
    if (_display) {
      if (_seq%100 == 0) {
        ROS_INFO("OptFlow pose: cumulative = %.3f,%.3f . since key frame = %.3f, %.3f . since prev = %.3f, %.3f",
                 _x, _y , H.at<double>(0,2), H.at<double>(1,2), dx, dy);
      }
      line( disp_img, Point2d(30.0,30.0), Point2d(30.0+50.*_x,30.0+50.*_y), Scalar(255), 2);
      std::ostringstream os; os << nmatch;//FONT_HERSHEY_PLAIN
      putText(disp_img, os.str(), Point(0,15), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0), 2);//, 8);
      imshow(win, disp_img);  waitKey(1);
    }
    
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
//ros::Publisher pub = nh.advertise<geometry_msgs::PointStamped>(pub_topic, 100);
  ros::Publisher pub = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>(pub_topic, 100);
  OptFlow optflow(&pub, cam_info/*caminfo_topic*/, display, threshold);                
  ros::Subscriber altitude_sub = nh.subscribe(altitude_topic, 1, &OptFlow::altitude_cb, &optflow);
  image_transport::Subscriber it_sub = it.subscribe(cam_topic, 1, &OptFlow::img_cb, &optflow);
  ros::spin();
  return 0;
}
