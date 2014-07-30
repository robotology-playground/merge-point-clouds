#include <iostream>
#include <string>

#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Module.h>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>

#include <dirent.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

using namespace std;
using namespace yarp::os;



class MergeModule:public RFModule
{
    RpcServer handlerPort; //a port to handle messages
    string path; //path to folder with .ply files

void Visualize(PointCloudT::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals)
	{
	
	pcl::visualization::PCLVisualizer viewer ("Merged Point Clouds");
	
	int v1(0); 
	viewer.createViewPort (0.0, 0.0, 1.0, 1.0, v1);
	
	// The color we will be using
	float bckgr_gray_level = 0.0; // Black
	float txt_gray_lvl = 1.0-bckgr_gray_level; 

	// ICP aligned point cloud 
	pcl::visualization::PointCloudColorHandlerRGBField<PointT> cloud_in_color_i (cloud);
	viewer.addPointCloud (cloud, cloud_in_color_i, "cloud_icp_v1", v1);


	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_icp_v1");
	
	//Add normals
	viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud,normals,10,0.01,"normals");


	// Set background color
	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
	
	// Set camera position and orientation
	viewer.setCameraPosition(-0.0611749, -0.040113, 0.00667606, -0.105521, 0.0891437, 0.990413);
	viewer.setSize(1280, 1024); // Visualiser window size


	// Display the visualiser
	while (!viewer.wasStopped ()) {
		viewer.spinOnce ();
	}

}


int MergePointclouds()
	{
    Network yarp;

	PointCloudT::Ptr cloud_in 	(new PointCloudT); // Original point cloud
	PointCloudT::Ptr cloud_icp	(new PointCloudT); // ICP output point cloud
	PointCloudT::Ptr cloud_filtered	(new PointCloudT); // downsampled point cloud
    
    int iterations = 20;
	// The Iterative Closest Point algorithm
	pcl::IterativeClosestPoint<PointT, PointT> icp;
	icp.setMaximumIterations(iterations);
	
	
	DIR *dir;
	struct dirent *ent;
	bool flag=false;
	int count = 0;

	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor; //filter to remove outliers
	sor.setStddevMulThresh (1.0);

	//loop through all ply-files in the data folder and apply ICP
	if ((dir = opendir (path.c_str())) != NULL) {
  		/* get all the files within directory */
  		while ((ent = readdir (dir)) != NULL) {
			// check if file name is correct
			string fname = ent->d_name;
			if (fname.find(".ply") != std::string::npos) {		
				printf ("Loading file: %s\n", ent->d_name);
				
								
				if (!flag) { //load first .ply file to cloud_icp
					if (pcl::io::loadPLYFile (path+fname, *cloud_icp) < 0)	{
						PCL_ERROR("Error loading cloud %s.\n", fname.c_str());
						return -1;
					}

					//filtering					
  					sor.setInputCloud (cloud_icp);
  					sor.setMeanK(cloud_icp->size()/2);
  					sor.filter (*cloud_icp);

					flag=true;
					count++;
				} else { //load all other files to cloud_in and apply ICP
					if (pcl::io::loadPLYFile (path+fname, *cloud_in) < 0)	{
						PCL_ERROR("Error loading cloud %s.\n", fname.c_str());
						return -1;
					}

					//filtering
					sor.setInputCloud (cloud_in);
  					sor.setMeanK(cloud_in->size()/2);
  					sor.filter (*cloud_in);			

					//ICP algorythm
					printf("Applying ICP...");
					icp.setInputSource(cloud_in);
					icp.setInputTarget(cloud_icp);
					icp.align(*cloud_in);
					
					if (icp.hasConverged()) {
						printf("\nICP has converged, score is %+.0e\n", icp.getFitnessScore());		
					} else {
						PCL_ERROR ("\nICP has not converged.\n");
						return -1;
					}

					*cloud_icp+=*cloud_in;
					count++;
				}
			}
  		}
  		closedir (dir);
	} else {
  		/* could not open directory */
  		perror ("can't load data files"); 	
		return EXIT_FAILURE;
	}

	//all files are processed, ICP is done	
	printf("ICP finished. %d files processed.\n", count);
	printf("Merged point cloud size is %d\n",cloud_icp->size());

	// save data
	/*printf("Saving data to file...\n");		
	pcl::PLYWriter writer;
	writer.write ("../cloud_merged.ply", *cloud_icp);
	*/

	// downsampling with voxel grid
	printf("Downsampling point clouds...\n");	
  	pcl::VoxelGrid<pcl::PointXYZRGB> grid;
  	grid.setInputCloud (cloud_icp);
  	grid.setLeafSize (0.005f, 0.005f, 0.005f);
  	grid.filter (*cloud_filtered);

	printf("Downsampled cloud size is %d\n",cloud_filtered->size());


	//compute surface normals
	printf("Computing surface normals...\n");

	// Create the normal estimation class, and pass the input dataset to it
  	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  	ne.setInputCloud (cloud_filtered);

  	// Create an empty kdtree representation, and pass it to the normal estimation object.
  	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  	ne.setSearchMethod (tree);

  	// Output datasets
  	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  	// Use all neighbors in a sphere of radius 1cm
  	ne.setRadiusSearch (0.01);

  	// Compute the features
  	ne.compute (*cloud_normals);

	// cloud_normals->points.size () should have the same size as the input cloud->points.size ()*
	printf("cloud size: %d; normals size: %d\n", cloud_filtered->points.size(), cloud_normals->points.size());

		
    printf("Visualizing point clouds...\n");				
	Visualize(cloud_filtered, cloud_normals);
    
    return 0;
}

public:

    double getPeriod()
    {
        return 1; //module periodicity (seconds)
    }

    bool updateModule()
    {
        return true;
    }

    bool respond(const Bottle& command, Bottle& reply) 
    {        
        if (command.get(0).asString()=="quit")
            return false;     
        else if (command.get(0).asString()=="merge")
            MergePointclouds();
        return true;
    }

    bool configure(yarp::os::ResourceFinder &rf)
    {
		path = rf.find("object-reconstruction").asString();        
		printf("Path: %s",path.c_str());		
		handlerPort.open("/mergeModule");
        attach(handlerPort);
        return true;
    }

    
    bool interruptModule()
    {
        cout<<"Interrupting your module, for port cleanup"<<endl;
        return true;
    }

    
    bool close()
    {
        cout<<"Calling close function\n";
        handlerPort.close();
        return true;
    }
};


int main(int argc, char * argv[])
{
    Network yarp;

    MergeModule module;
    ResourceFinder rf;
	rf.setDefaultContext("merge-point-clouds");
    rf.setDefaultConfigFile("config.ini");
    rf.setVerbose(true);
    rf.configure(argc, argv);


    cout<<"Configure module..."<<endl;
    module.configure(rf);
    cout<<"Start module..."<<endl;
    module.runModule();

    cout<<"Main returning..."<<endl;
    return 0;
}



