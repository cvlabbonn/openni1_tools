#include "viewer.h"

int Viewer::loop(){
    char key;
    // loop until esc is pressed
    while(!exitFlag){
        rc = context.WaitAndUpdateAll();
        if(rc == XN_STATUS_OK){
            // read the next frame
            depth_gen.GetMetaData(depth_data);
            image_gen.GetMetaData(image_data);

//            depth.readFrame(&depthFrame);
//            color.readFrame(&colorFrame);
            // make sure the data is valid
                cv::Mat normDepth, rgbd, depth_thresh;
                //process both images

                // depth image
                const XnDepthPixel* depthBuffer = depth_data.Data();
                frameDepth.create(depth_data.YRes(), depth_data.XRes(), CV_16UC1);
                memcpy( frameDepth.data, depthBuffer, depth_data.YRes()*depth_data.XRes()*sizeof(uint16_t) );
                cv::flip(frameDepth,frameDepth,1);


                // color image
                const XnUInt8* imageBuffer = image_data.Data();;
                frame.create(image_data.YRes(), image_data.XRes(), CV_8UC3);
                memcpy( frame.data, imageBuffer, 3*image_data.YRes()*image_data.XRes()*sizeof(uint8_t) );
                cv::flip(frame,frame,1);
                cv::cvtColor(frame,bgrMat,CV_RGB2BGR);
                cv::imshow("Color", bgrMat);

                // create the rgbd image
                createRGBD(frameDepth, bgrMat, rgbd, depth_thresh);
                cv::imshow("RGBD", rgbd);

                //normalize and show depth data
                cv::normalize(depth_thresh, normDepth, 0, 255, CV_MINMAX, CV_8UC1);
                cv::imshow("Depth", normDepth);

                if (saveMemory){
                    //store all data
                    rgb_images.push_back( bgrMat.clone() );
                    rgbd_images.push_back( rgbd.clone());
                    raw_depth.push_back( depth_thresh.clone() );
                    depth_show.push_back( normDepth.clone() );
                    FRAME_COUNTER += 1;

                }

                // get key interaction
                key = cv::waitKey(1);
                key_parse(key);
        }
}
    return 0;
}

void Viewer::createRGBD(cv::Mat& depth_mat, cv::Mat& color_mat, cv::Mat& dst_rgbd, cv::Mat& dst_depth){

    dst_rgbd = cv::Mat::zeros(depth_mat.rows, depth_mat.cols, CV_8UC3);
    dst_depth = cv::Mat::zeros(depth_mat.rows, depth_mat.cols, CV_16UC1);

    for (int j = 0; j< depth_mat.rows; j ++){
        for(int i = 0; i < depth_mat.cols; i++){
            int depth_value = (int) depth_mat.at<unsigned short>(j,i);
            if (depth_value != 0 && depth_value <= limitz_max && depth_value >= limitz_min)
                if ( limitx_min <= i && limitx_max >=i && limity_min <= j && limity_max >= j ){
                    dst_rgbd.at<cv::Vec3b>(j,i)  = color_mat.at<cv::Vec3b>(j,i);
                    dst_depth.at<unsigned short>(j,i)  = depth_mat.at<unsigned short>(j,i);
                }
        }
    }
}
