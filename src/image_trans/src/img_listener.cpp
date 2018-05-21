/*****
 * sub img from img topic:/camera/image_raw
 * *****/
//qt head

/***
 * ros img headfile
 * ****/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <sys/time.h>

using namespace std;
/****
 * socket headfile
 * ******/
#include <netinet/in.h>    // for sockaddr_in
#include <sys/types.h>    // for socket
#include <sys/socket.h>    // for socket
#include <stdio.h>        // for qDebug
#include <stdlib.h>        // for exit
#include <string.h>        // for bzero
#include <time.h>                //for time_t and time
#include <arpa/inet.h>
#include <unistd.h>    //close(client_socket);

// opencv headfile
#include <vector>
#include <opencv2/imgproc/imgproc.hpp>


#define SERVER_PORT   7754
#define BUFFER_SIZE 1024
#define serverIpAddr "192.168.1.103"

unsigned int fileNum = 1;
bool getImgFlag = true;
void imageTransCallback(const sensor_msgs::ImageConstPtr& msg);
void captureTimerCallback(const ros::TimerEvent& e);
void socketInit();


int main(int argc, char **argv)
{
  ros::init(argc, argv, "img_listener");
  ros::NodeHandle nhImage;
  cv::namedWindow("show source image");
  cv::startWindowThread();
  //socket
  socketInit();
  //ros timer
  ros::Timer captureTimer=nhImage.createTimer(ros::Duration(2),captureTimerCallback);
  image_transport::ImageTransport transport(nhImage);

  //ROS_INFO("get image fro topic ");
  image_transport::Subscriber sub = transport.subscribe("/usb_cam/image_raw",1,imageTransCallback);

  ros::spin();
  cv::destroyWindow("view");

  return 0;
}
void socketInit()
{
  //socket image trans
     /**************
      * socket struct
      * ***********/
     //设置一个socket地址结构client_addr,代表客户机internet地址, 端口
     struct sockaddr_in client_addr;
     bzero(&client_addr,sizeof(client_addr)); //把一段内存区的内容全部设置为0
    // memset(&client_addr,sizeof(client_addr));
     client_addr.sin_family = AF_INET;    //internet协议族
     client_addr.sin_addr.s_addr = htons(INADDR_ANY);//INADDR_ANY表示自动获取本机地址
     client_addr.sin_port = htons(0);    //0表示让系统自动分配一个空闲端口

     /*****
      * socket descriptor
      * 创建用于internet的流协议(TCP)socket,用client_socket代表客户机socket
      * ****/
     int client_socket = socket(AF_INET,SOCK_STREAM,0);
     if( client_socket < 0)
         {
             cout<<"Create Socket Failed!"<<endl;
             exit(1);
         }

     /******
      * bind port
      * 把客户机的socket和客户机的socket地址结构联系起来,zhiding client_socket ip,point to addr
      * *******/
     if( bind(client_socket,(struct sockaddr*)&client_addr,sizeof(client_addr)))
         {
             cout<<"Client Bind Port Failed!"<<endl;
             exit(1);
         }

     /*****
      * get server addr from argv[1],
      * set server params
      * 设置一个socket地址结构server_addr,代表服务器的internet地址, 端口
      * server_addr.sin_addr.s_addr=inet_addr(argv[1]);//same as up inet_aton
      * ****/
     //
     struct sockaddr_in server_addr;
     bzero(&server_addr,sizeof(server_addr));
     server_addr.sin_family = AF_INET;
     server_addr.sin_addr.s_addr = inet_addr(serverIpAddr);
     server_addr.sin_port = htons(SERVER_PORT);
     socklen_t server_addr_length = sizeof(server_addr);
     /***
      * connect request,to server ip.success return 0;fail is 1
      * 向服务器发起连接,连接成功后client_socket代表了客户机和服务器的一个socket连接
      * **/
      if(connect(client_socket,(struct sockaddr*)&server_addr, server_addr_length) < 0)
         {
             cout<<"Can Not Connect To "<<serverIpAddr<<endl;
             exit(1);
         }
     cout<<"Success Connect To Server,Server Addr is "<<serverIpAddr<<"!"<<endl;
}
void captureTimerCallback(const ros::TimerEvent& e)
{
  getImgFlag=true;
  ROS_INFO(" T ");
}

void imageTransCallback(const sensor_msgs::ImageConstPtr& msg)
{
  if(getImgFlag)
  {
    try
    {
      //get img & show,transfer ros img to cv img
      cv::Mat s_img = cv_bridge::toCvShare(msg, "bgr8")->image;
      cv::imshow("show source image", s_img);
      //imgsave
      char key;
      key=cv::waitKey(33);

      struct tm* fileTime;
      char filePath[100];
      char fileName[100];
      time_t t;
      t=time(NULL);
      fileTime=localtime(&t);
      strftime(filePath,100,"/home/nvidia/Travel_Assistance_Robot/image/%Y%m%d_%H%M%S.jpg",fileTime);
      strftime(fileName,100,"%Y%m%d_%H%M%S.jpg",fileTime);

      cv::imwrite(filePath,s_img);
      // set flag
      getImgFlag=false;
      ROS_INFO("F");


        /*********************  data transfer test  ****************************/
        /*****
         * data prepare,set buffer
         * bzero == memset
         * ****/
        char buffer[BUFFER_SIZE];
        bzero(buffer,BUFFER_SIZE);
        //从服务器接收数据到buffer中
        int length = recv(client_socket,buffer,BUFFER_SIZE,0);
        if(length < 0)
            {
                cout<<"Recieve Data From Server %s Failed!"<<serverIpAddr<<endl;
                exit(1);
            }
        cout<<"buffer size is "<<buffer<<endl;
        bzero(buffer,BUFFER_SIZE);
        // 向服务器发buffer中的数据
        bzero(buffer,BUFFER_SIZE);
        strcpy(buffer,"Hello, World! From Client\n");
        int send_flag=send(client_socket,buffer,BUFFER_SIZE,0);
        if(!send_flag)
            cout<<"send error"<<endl;
        cout<<"send success"<<endl;

  //      /********************* 向服务器发送image  ****************************/
  //      //1.load image,get imagesize
  //      cv::Mat s_img=cv::imread("1.jpg");
  //      imshow("s_img",s_img);
  //      vector<uchar> encode_img;
  //      imencode(".jpg", s_img, encode_img);
  //      //get send_buffer
  //      int encode_img_size=encode_img.size();
  //      int s_img_size=s_img.rows*s_img.cols*3;
  //      cout<<"filesize is"<<encode_img_size<< "width*hight*3 is"<<s_img_size<<endl;

  //      uchar* send_buffer=new uchar[encode_img.size()];
  //      copy(encode_img.begin(),encode_img.end(),send_buffer);
  //      //2.send file_name
  //      int toSend=encode_img_size, receive  = 0, finished = 0;
  //      const char* file_name;
  //      char char_len[10];
  //      //file_name= fn.c_str();
  //      file_name=fileName;
  //      // file_name,qDebug file_name be empty
  //      //qDebug("file name is %s\n",file_name);
  //      bzero(buffer,BUFFER_SIZE);
  //      send_flag=send(client_socket, file_name, 100, 0);
  //      if(!send_flag)
  //        {
  //          cout<<" send file_name failed\n "<<endl;
  //          exit(1);
  //        }
  //      cout<<"success send file_name"<<endl;
  //      //3.send image length
  //      sprintf(char_len, "%d", toSend);
  //      send(client_socket, char_len, 10, 0);//hello world!!hei hei(xiao)!!  strlen(char_len)这里要写一个固定长度，然后让服务器端读出一个固定长度，否则会出错
  //      cout<<"char_len is "<<char_len<<endl;

  //      // send test

  //      //4.send image data
  //      while(toSend  >  0)
  //      {
  //          int size = (toSend>1000?1000:toSend);
  //          if((receive = send(client_socket, send_buffer + finished, size, 0)))  //send wenzi
  //          {
  //              if(receive==-1)
  //              {
  //                  cout<<"receive error"<<endl;
  //                  break;
  //              }
  //              else
  //              {
  //                  toSend -= receive;// shengxia de unsend
  //                  finished += receive; //sended
  //              }
  //          }
  //      }

  //      //5.close socket
  //      close(client_socket);
  //      cout<<"close socket"<<endl;
  //    }

    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
  }
}

