/*
Software License Agreement (BSD License)

Copyright (c) 2013, Southwest Research Institute
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
 * Neither the name of the Southwest Research Institute, nor the names
     of its contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
 */

#include "open_ptrack/detection/haardispada.h"
#include "open_ptrack/opt_utils/conversions.h"

#include "ros/ros.h"
#include <sstream>

//Publish Messages
#include "opt_msgs/RoiRect.h"
#include "opt_msgs/Rois.h"
#include "opt_msgs/DetectionArray.h"
#include "std_msgs/String.h"

//Time Synchronizer
// NOTE: Time Synchronizer conflicts with QT includes may need to investigate
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

//Subscribe Messages
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// Image Transport
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

// Used to display OPENCV images
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// Dynamic reconfigure:
#include <dynamic_reconfigure/server.h>
#include <detection/HaarDispAdaDetectorConfig.h>


using namespace message_filters::sync_policies;
using namespace opt_msgs;
using namespace sensor_msgs;
using namespace sensor_msgs::image_encodings;
using sensor_msgs::Image;
using sensor_msgs::PointCloud2;
using cv_bridge::CvImagePtr;
using std::vector;
using std::string;
using cv::Rect;
using cv::Mat;

class HaarDispAdaNode
{

    typedef detection::HaarDispAdaDetectorConfig Config;
    typedef dynamic_reconfigure::Server<Config> ReconfigureServer;

  private:
    // Define Node
    ros::NodeHandle node_;

    // Subscribe to Messages
    message_filters::Subscriber<PointCloud2> sub_pointcloud_;
    message_filters::Subscriber<Image> sub_image_;
    message_filters::Subscriber<opt_msgs::DetectionArray> sub_detections_;

    // Define the Synchronizer
    typedef ApproximateTime<PointCloud2, Image, opt_msgs::DetectionArray> ApproximatePolicy;
    typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
    boost::shared_ptr<ApproximateSync> approximate_sync_;

    // Messages to Publish
    ros::Publisher pub_rois_;
    ros::Publisher pub_Color_Image_;
    ros::Publisher pub_detections_;

    Rois output_rois_;

    enum {ACCUMULATE=0, TRAIN, DETECT, EVALUATE, LOAD};

    //Define the Classifier Object
    open_ptrack::detection::HaarDispAdaClassifier HDAC_;

    int num_class1;
    int num_class0;
    int num_TP_class1;
    int num_FP_class1;
    int num_TP_class0;
    int num_FP_class0;

    //
    bool vertical;

    // Flag stating if classifiers based on disparity image should be used or not:
    bool use_disparity;

    // Minimum classifier confidence for people detection:
    double min_confidence;

    // Object of class Conversions:
    open_ptrack::opt_utils::Conversions converter;

    // Output detections message:
    DetectionArray::Ptr output_detection_msg_;

    // Dynamic reconfigure
    boost::recursive_mutex config_mutex_;
    boost::shared_ptr<ReconfigureServer> reconfigure_server_;

  public:

    explicit HaarDispAdaNode(const ros::NodeHandle& nh):
    node_(nh)
    {
      num_class1 = 0;
      num_class0 = 0;
      num_TP_class1 = 0;
      num_FP_class1 = 0;
      num_TP_class0 = 0;
      num_FP_class0 = 0;

      nh.param("vertical", vertical, false);
      HDAC_.setOrientation(vertical);

      string nn = ros::this_node::getName();
      int qs;
      if(!node_.getParam("Q_Size",qs)){
        qs=3;
      }

      if(!node_.getParam("use_disparity", use_disparity)){
        use_disparity = true;
      }

      if(!node_.getParam("haar_disp_ada_min_confidence", min_confidence)){
        min_confidence = 3.0;
      }
      HDAC_.setMinConfidence(min_confidence);

      int NS;
      if(!node_.getParam("num_Training_Samples",NS)){
        NS = 350; // default sets asside very little for training
        node_.setParam("num_Training_Samples",NS);
      }
      HDAC_.setMaxSamples(NS);

      output_detection_msg_ = DetectionArray::Ptr(new DetectionArray);

      // Published Messages
      pub_rois_           = node_.advertise<Rois>("/HaarDispAdaOutputRois",qs);
      pub_Color_Image_    = node_.advertise<Image>("/HaarDispAdaColorImage",qs);
      pub_detections_ = node_.advertise<DetectionArray>("/detector/detections",3);

      // Subscribe to Messages
      sub_image_.subscribe(node_,"/Color_Image",qs);
      sub_pointcloud_.subscribe(node_, "/PointCloud",qs);
      sub_detections_.subscribe(node_,"/input_detections",qs);

      // Sync the Synchronizer
      approximate_sync_.reset(new ApproximateSync(ApproximatePolicy(qs),
          sub_pointcloud_,
    	  sub_image_,
          sub_detections_));

      approximate_sync_->registerCallback(boost::bind(&HaarDispAdaNode::imageCb,
          this,
          _1,
          _2,
          _3));

      // Set up dynamic reconfiguration
      ReconfigureServer::CallbackType f = boost::bind(&HaarDispAdaNode::configCb, this, _1, _2);
      reconfigure_server_.reset(new ReconfigureServer(config_mutex_, node_));
      reconfigure_server_->setCallback(f);
   }

    int
    get_mode()
    {

      int callback_mode;
      std::string mode="";
      node_.param("mode", mode, std::string("none"));
      if(mode.compare("detect") == 0)
      {
        callback_mode = DETECT;
      }
      else if(mode.compare("train")==0)
      {
        callback_mode = TRAIN;
      }
      else if(mode.compare("load")==0)
      {
        callback_mode = LOAD;
      }
      else if(mode.compare("evaluate")==0)
      {
        callback_mode = EVALUATE;
      }
      else if(mode.compare("accumulate")==0)
      {
        callback_mode = ACCUMULATE;
      }
      else // default mode is accumulate
      {
        callback_mode = ACCUMULATE;
      }
      return(callback_mode);
    }

    void
    createOutputDetectionsMessage(const DetectionArray::ConstPtr& input_msg, vector<float> confidences, DetectionArray::Ptr& output_msg)
    {
      // Set camera-specific fields:
      output_msg->detections.clear();
      output_msg->header = input_msg->header;
      output_msg->intrinsic_matrix = input_msg->intrinsic_matrix;
      output_msg->confidence_type = std::string("haar+ada");
      output_msg->image_type = std::string("disparity");


      // Add all valid detections:
      int k = 0;
      for(unsigned int i = 0; i < input_msg->detections.size(); i++)
      {
        if((!use_disparity) | (confidences[i] > min_confidence))            // keep only people with confidence above a threshold if use_disparity == true or all detections if use_disparity == false
        {
          output_msg->detections.push_back(input_msg->detections[i]);
          if (use_disparity)
            output_msg->detections[k].confidence = confidences[i];

          k++;
        }
      }
    }


    void
	convertPointCloud2Depth(pcl::PointCloud<pcl::PointXYZRGB>& depth_cloud, cv::Mat& dmatrix)
    {
        float f = 575.8157348632812; 	// focal length
        float T = 0.075;	// baseline distance

        dmatrix.create(depth_cloud.height, depth_cloud.width, CV_32FC1);
    	uchar* depth_image_ptr = (uchar*)dmatrix.data;
		 for (int v = 0; v < (int)depth_cloud.height; v++)
		 {
			 int depth_base_index = dmatrix.step * v;
			 for (int u = 0; u < (int)depth_cloud.width; u++)
			 {
				 int depth_index = depth_base_index + 1 * u * sizeof(float);
				 float* depth_data_ptr = (float*)(depth_image_ptr + depth_index);
				 pcl::PointXYZRGB point_xyz = depth_cloud(u, v);
				 depth_data_ptr[0] =(isnan(point_xyz.z)) ? 0.f : (f / (point_xyz.z * (1/T)));	// get disparity from depth
			 }
		 }
    }

    void
    imageCb(const PointCloud2ConstPtr& cloud_msg,
    		const ImageConstPtr& image_msg,
			const opt_msgs::DetectionArray::ConstPtr& detection_msg)
    {
      // Callback for people detection:
      bool label_all;
      vector<int> L_in;
      vector<int> L_out;
      vector<float> C_out;
      vector<Rect> R_in;
      vector<Rect> R_out;
      string param_name;
      string nn = ros::this_node::getName();
      string cfnm;
      int numSamples;
      double temp=0.0;

      pcl::PointCloud <pcl::PointXYZRGB> depth_cloud;
      pcl::fromROSMsg(*cloud_msg, depth_cloud);

	  cv::Mat dmatrix;
	  convertPointCloud2Depth(depth_cloud, dmatrix);

      sensor_msgs::Image::Ptr color_image_msg;
      cv::Mat color_image;

      cv_bridge::CvImageConstPtr color_image_ptr;
      color_image_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);
      color_image = color_image_ptr->image;

      // if vertical rotate Image and Disparity
	  if (vertical)
	  {
		 cv::Mat color_image_turned;
		 color_image_turned.create(color_image.cols, color_image.rows, color_image.type());

		 for (int v = 0; v < color_image_turned.rows; v++)
			 for (int u = 0; u < color_image_turned.cols; u++)
				 color_image_turned.at<cv::Vec3b>(v,u) = color_image.at<cv::Vec3b>(color_image.rows-1-u,v);
		// color_image.create(color_image_turned.rows, color_image_turned.cols, color_image_turned.type());

		 color_image = color_image_turned;

		 cv_bridge::CvImage cv_ptr;
		 cv_ptr.image = color_image_turned;
		 cv_ptr.encoding = sensor_msgs::image_encodings::BGR8;
		 color_image_msg = cv_ptr.toImageMsg();
		 color_image_msg->header = image_msg->header;


		 cv::Mat dmatrix_turned;
		 dmatrix_turned.create(dmatrix.cols, dmatrix.rows, dmatrix.type());

		 for (int v = 0; v < dmatrix_turned.rows; v++)
			 for (int u = 0; u < dmatrix_turned.cols; u++)
				 dmatrix_turned.at<float>(v,u) = dmatrix.at<float>(dmatrix.rows-1-u,v);

		 dmatrix = dmatrix_turned;
	  }

      if(!node_.getParam("UseMissingDataMask",HDAC_.useMissingDataMask_)){
        HDAC_.useMissingDataMask_ = false;
      }


      // **********************************************************************//
      // between these comments lies a hack that accounts for the difference   //
      // between the focal lengths of the kinect's color camera and ir cameras //
      // TODO, parameterize to disable this hack for the stereo data           //
      bool kinect_disparity_fix;
      if(!node_.getParam("Kinect_Disparity_Fix",kinect_disparity_fix)){
        kinect_disparity_fix = false;
      }

      if(kinect_disparity_fix){
    	  int nrows;
    	  int ncols;

    	if (vertical)
    	{
            nrows = 434;
            ncols = 579;
    	}
    	else
    	{
            ncols = 434;
            nrows = 579;
    	}
        int row_offset = (dmatrix.rows - nrows)/2;
        int col_offset = (dmatrix.cols - ncols)/2;
        cv::Mat Scaled_Disparity(nrows,ncols,CV_32FC1);
        resize(dmatrix,Scaled_Disparity,cv::Size(ncols,nrows),0,0, CV_INTER_NN );
        for(int i=0;i<dmatrix.rows;i++){
          for(int j=0;j<dmatrix.cols;j++){
            dmatrix.at<float>(i,j) = 0.0;
          }
        }
        for(int i=0;i<nrows;i++){
          for(int j=0;j<ncols;j++){
            dmatrix.at<float>(i+row_offset,j+col_offset) = Scaled_Disparity.at<float>(i,j);
          }
        }
      }
      // **********************************************************************//

      // take the detection message and create vectors of ROIs and labels
      R_in.clear();
      L_in.clear();

      // Read camera intrinsic parameters:
      Eigen::Matrix3f intrinsic_matrix;
      for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
          intrinsic_matrix(i, j) = detection_msg->intrinsic_matrix[i * 3 + j];

      // Read detections:
      for(unsigned int i=0;i<detection_msg->detections.size();i++)
      {
        // theoretical person centroid:
        Eigen::Vector3f centroid3d(detection_msg->detections[i].centroid.x, detection_msg->detections[i].centroid.y, detection_msg->detections[i].centroid.z);
        Eigen::Vector3f centroid2d = converter.world2cam(centroid3d, intrinsic_matrix);

        // theoretical person top point:
        Eigen::Vector3f top3d(detection_msg->detections[i].top.x, detection_msg->detections[i].top.y, detection_msg->detections[i].top.z);
        Eigen::Vector3f top2d = converter.world2cam(top3d, intrinsic_matrix);

        if (vertical) // rotate to the right pixelcoordinates
        {
        	int helper = centroid2d(0);
        	centroid2d(0) = color_image.cols-1-centroid2d(1);
        	centroid2d(1) = helper;

        	helper = top2d(0);
        	top2d(0) = color_image.cols-1-top2d(1);
        	top2d(1) = helper;
        }

        // Define Rect and make sure it is not out of the image:
        int h = centroid2d(1) - top2d(1);
        int w = h * 2 / 3.0;
        int x = std::max(0, int(centroid2d(0) - w / 2.0));
        int y = std::max(0, int(top2d(1)));
        h = std::min(int(dmatrix.rows - y), int(h));
        w = std::min(int(dmatrix.cols - x), int(w));

        Rect R(x,y,w,h);
        R_in.push_back(R);
        L_in.push_back(1);
      }
      // do the work of the node
      switch(get_mode())
	  {
      	case DETECT:
          // Perform people detection within the input rois:
          label_all = true;
          HDAC_.detect(R_in,L_in,dmatrix,R_out,L_out,C_out,label_all);

          // Build output detections message:
          createOutputDetectionsMessage(detection_msg, C_out, output_detection_msg_);

          output_rois_.rois.clear();
          output_rois_.header.stamp = image_msg->header.stamp;
          output_rois_.header.frame_id = image_msg->header.frame_id;
//		ROS_INFO("HaarDispAda found %d objects",(int)L_out.size());

          for(unsigned int i=0; i<R_out.size();i++){
            RoiRect R;
            R.x      = R_out[i].x;
            R.y      = R_out[i].y;
            R.width  = R_out[i].width;
            R.height = R_out[i].height;

/*
			if (C_out[i] > min_confidence)
            {
            	cv::Point ptUpperLeft = cv::Point(R.x,R.y);
            	cv::Point ptLowerRight = cv::Point(R.x+R.width,R.y+R.height);
            	rectangle(color_image,ptUpperLeft,ptLowerRight,CV_RGB(0, 0, 225), 2);
            }

*/
            if (use_disparity)
              R.label  = L_out[i];
            else
              R.label = 1;
            R.confidence = C_out[i];
            output_rois_.rois.push_back(R);
          }

          pub_rois_.publish(output_rois_);
          (vertical) ? pub_Color_Image_.publish(color_image_msg) : pub_Color_Image_.publish(image_msg);
          pub_detections_.publish(output_detection_msg_);
          break;

        case ACCUMULATE:
          numSamples = HDAC_.addToTraining(R_in,L_in,dmatrix);
          param_name = "num_Training_Samples";
          int NS;
          if(node_.getParam(param_name,NS)){
            float percent = (float)HDAC_.numSamples_ * 100.0/NS;
            ROS_INFO("ACCUMULATING: %6.1f%c",percent,'%');
            if(numSamples >= NS){
              param_name = "mode";
              node_.setParam(param_name, std::string("train"));
              ROS_ERROR("DONE Accumulating, switching to train mode");
            }
          }
          break;

        case TRAIN:
          param_name = "classifier_file";
          node_.param(param_name,cfnm,std::string("/home/clewis/classifiers/test.xml"));
          param_name = "HaarDispAdaPrior";
          node_.getParam(param_name,temp);
          HDAC_.HaarDispAdaPrior_ = temp;
          ROS_ERROR("Submitting %d Samples to Train ouput= %s",HDAC_.numSamples_,cfnm.c_str());
          HDAC_.train(cfnm);
          param_name = "mode";
          node_.setParam("mode", std::string("evaluate"));
          ROS_ERROR("DONE TRAINING, switching to evaluate mode");
          break;

        case EVALUATE:
        {
          if(!HDAC_.loaded)
		  {
            param_name = "classifier_file";
            node_.param(param_name,cfnm,std::string("test.xml"));
            std::cout << "HaarDispAda LOADING " << cfnm.c_str() << std::endl;
            HDAC_.load(cfnm);
          }
          label_all = false;
          HDAC_.detect(R_in,L_in,dmatrix,R_out,L_out,label_all);

          int total0_in_list=0;
          int total1_in_list=0;
          int fp_in_list=0;
          int tp_in_list=0;

          // count the input labels
          for(unsigned int i=0; i<R_in.size();i++)
		  {
            if(L_in[i] == 0 || L_in[i] == -1) total0_in_list++;
            if(L_in[i] == 1) total1_in_list++;
          }
          num_class0 += total0_in_list;
          num_class1 += total1_in_list;

          // count the output labels which have the correct and incorrect label
          for(unsigned int i=0; i<R_out.size();i++)
		  {
            if((L_out[i] == 1) | (use_disparity))
				tp_in_list++;
            
            else 
				fp_in_list++;
          }
          num_TP_class1 += tp_in_list;
          num_FP_class1 += fp_in_list;
          num_TP_class0 += total0_in_list - fp_in_list;
          num_FP_class0 += total1_in_list - tp_in_list;
          float tp1 = (float)num_TP_class1 / (float) num_class1 * 100.0;
          float tp0 = (float)num_TP_class0 / (float) num_class0 * 100.0;
          float fp1 = (float)num_FP_class1 / (float) num_class1 * 100.0;
          float fp0 = (float)num_FP_class0 / (float) num_class0 * 100.0;
          ROS_ERROR("TP0 = %6.2f FP0 =  %6.2f TP1 = %6.2f FP1 =  %6.2f",tp0,fp0,tp1,fp1);
        }

        // TODO
        break;

        case LOAD:
          param_name = "classifier_file";
          node_.param(param_name,cfnm,std::string("test.xml"));
          std::cout << "HaarDispAda LOADING " << cfnm.c_str() << std::endl;
          HDAC_.load(cfnm);
          param_name = "mode";
          node_.setParam(param_name, std::string("detect"));
          break;
      }// end switch
      
/*
      cv::imshow("Haar Detections", color_image);
      cv::waitKey(10);
*/
    }

    void
    configCb (Config &config, uint32_t level)
    {
      use_disparity = config.use_disparity;
      min_confidence = config.haar_disp_ada_min_confidence;
      HDAC_.setMinConfidence (min_confidence);
    }

    ~HaarDispAdaNode()
    {
    }
};

int
main(int argc, char **argv)
{
  ros::init(argc, argv, "HaarDispAda");
  ros::NodeHandle n("~");
  HaarDispAdaNode HN(n);
  ros::spin();
  return 0;
}

