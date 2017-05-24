/*
 * cloud_viewer.cpp
 *
 *  Created on: Apr 10, 2017
 *      Author: kevin
 */

#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

int user_data;

// Not used
void viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor (1.0, 0.5, 1.0);
    pcl::PointXYZ o;
    o.x = 1.0;
    o.y = 0;
    o.z = 0;
    viewer.addSphere (o, 0.25, "sphere", 0);
    std::cout << "i only run once" << std::endl;

}

// Not used
void viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
{
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape ("text", 0);
    viewer.addText (ss.str(), 200, 300, "text", 0);

    //FIXME: possible race condition here:
    user_data++;
}

int main ()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

	std::string input_path = "/home/kevin/research/gazebo/evaluation_platform/only_snapshot/pcd/pointcloud_1.pcd";

	std::cout << "Loading from: " << input_path << std::endl;

    pcl::io::loadPCDFile (input_path, *cloud);
    //pcl::io::loadPCDFile ("Buffer_1.pcd", *cloud);



    //std::cout << "Visualization" << std::endl;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("Point Cloud Viewer"));

    viewer->setBackgroundColor (0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ> (cloud, "the cloud");
	//viewer->addPointCloud<pcl::PointXYZ> (target, "target cloud");
	//viewer->addPointCloud<pcl::PointXYZ> (result, "result cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0,1,0, "the cloud");
	//viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0,1,0, "target cloud");

    //pcl::visualization::CloudViewer viewer("Cloud Viewer");

    //blocks until the cloud is actually rendered
    //viewer.showCloud(cloud);

    //use the following functions to get access to the underlying more advanced/powerful
    //PCLVisualizer

    //This will only get called once
    //viewer.runOnVisualizationThreadOnce (viewerOneOff);

    //This will get called once per visualization iteration
    //viewer.runOnVisualizationThread (viewerPsycho);
//    while (!viewer.wasStopped ())
//    {
//		//you can also do cool processing here
//		//FIXME: Note that this is running in a separate thread from viewerPsycho
//		//and you should guard against race conditions yourself...
//		user_data++;
//    }

    while (!viewer->wasStopped ())
    {
    	viewer->spinOnce (100);
    }



    return 0;
}



