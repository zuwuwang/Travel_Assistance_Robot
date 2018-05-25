/***
 * sub img,socket transfer
 * **********/
//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
//Use image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>
//Use cv_bridge to convert between ROS and OpenCV Image formats
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>
//Include headers for OpenCV Image processing
#include <opencv2/imgproc/imgproc.hpp>
//Include headers for OpenCV GUI handling
#include <opencv2/highgui/highgui.hpp>
#include<string>
#include <sstream>
#include <std_msgs/UInt8.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <netdb.h>
#include <netinet/in.h>
#include  <arpa/inet.h>
using namespace cv;
using namespace std;

//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;

void image_socket(Mat inImg);
Mat image1;
static int imgWidth, imgHeight;

static int image_num = 1;

#define PORT 30000
#define BUFFER_SIZE 20000
const char * servInetAddr = "192.168.1.103"; //service addr
static int sockfd;

//static ros::Publisher capture;
//This function is called everytime a new image_info message is published
void camInfoCallback(const sensor_msgs::CameraInfo & camInfoMsg)
{
  //Store the image width for calculation of angle
  imgWidth = camInfoMsg.width;
  imgHeight = camInfoMsg.height;
}

//This function is called everytime a new image is published
void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
    //Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        //Always copy, returning a mutable CvImage
        //OpenCV expects color images to use BGR channel order.
        cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        //if there is an error during conversion, display it
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }
    image_socket(cv_ptr->image);

}

void image_socket(Mat inImg)
{
   imshow("image_socket", inImg);//显示图片
    if( inImg.empty() )
    {
      ROS_INFO("Camera image empty");
      return;//break;
    }
    stringstream sss;
    string strs;

    char c = (char)waitKey(1);

    if( c == 27 )
      ROS_INFO("Exit boss");//break;
    switch(c)
    {
      case 'p':

      resize(inImg,image1,Size(imgWidth/6,imgHeight/6),0,0,CV_INTER_LINEAR);
      image1=image1(Rect(image1.cols/2-32,image1.rows/2-32, 64, 64));

      strs="/home/hsn/catkin_ws/src/rosopencv/";
      sss.clear();
      sss<<strs;
      sss<<image_num;
      sss<<".jpg";
      sss>>strs;
      imwrite(strs,image1);//保存图片
      cout<<image_num<<endl;

      int n;
      n = write(sockfd,image1.data,image1.total()*image1.channels());
      if (n < 0)
         perror("ERROR writing to socket");
      image_num++;
      break;

  default:
      break;
  }

}

/**
* This is ROS node to track the destination image
*/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_socket");
    ROS_INFO("-----test------");

    struct sockaddr_in serv_addr;
    /*创建socket*/
    if ((sockfd = socket(AF_INET,SOCK_STREAM,0)) == -1)
    {
      perror("Socket failed!\n");
      exit(1);
    }
    printf("Socket id = %d\n",sockfd);
    /*设置sockaddr_in 结构体中相关参数*/
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);
    inet_pton(AF_INET, servInetAddr, &serv_addr.sin_addr);
    bzero(&(serv_addr.sin_zero), 8);
    /*调用connect 函数主动发起对服务器端的连接*/
    if(connect(sockfd,(struct sockaddr *)&serv_addr, sizeof(serv_addr))== -1)
    {
        perror("Connect failed!\n");
        exit(1);
    }
    printf("welcome\n");

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    image_transport::Subscriber sub = it.subscribe("camera/rgb/image_raw", 1, imageCallback);
    ros::Subscriber camInfo         = nh.subscribe("camera/rgb/camera_info", 1, camInfoCallback);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
       ros::spinOnce();
       loop_rate.sleep();
    }
    close(sockfd);
    //ROS_INFO is the replacement for printf/cout.
    ROS_INFO("No error.");
return 0;

}
