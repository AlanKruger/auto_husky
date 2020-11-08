// This program preprocesses the LiDAR data to avoid excessive or corrupt data as well as localizes highly reflective clusters 
// and packages the avg coordinates into a custom msg. Output = /object_parameters rostopic

#include <ros/ros.h>
#include <cstdlib>
#include <math.h>
#include <stdio.h>
#include <iostream>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <auto_husky/avoidance_input.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
 
using namespace std;
using namespace ros;
using namespace pcl;

// Const Global Variables ////////////////////////

//overall
int const num_objs = 2;				// number of objects to track
float const freq = 10;				// program frequency (Hz) 
float const distance_limit = 10;		// limit the latitudinal distance of the lidar (m)
float const side_roi = 0.5;			// limit the longitudinal distance of the lidar (m)
float const movement_threshold = 0.3;		// maximum displacement between where an object can move between frames (m)
float const static_threshold = 0.1;		// velocities below this value are essentially static (m/s)
float const fast_threshold = 3;			// velocity of fast object (m/s)

//filtering
float const req_density_dist = 0.04;		// spacing between candidate points (m)
int const req_density_neighbors = 5;		// minimum number of neighbors a point needs to have 

//object sorting
float const object_separation = 0.2;		// smaller = more object clusters | bigger = less object clusters
int const min_cloud_density = 100;		// minimum number of points a cluster needs to be an object
int const max_cloud_density = 1000;		// maximum number of points a cluster needs to be an object

// Global Variables /////////////////////////////

PointCloud<PointXYZI>::Ptr cloud (new PointCloud<PointXYZI>);				// 3D point cloud visualization only
PointCloud<pcl::PointXYZI>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZI>);	// filtered 3D point cloud visualization

class Clouds
{
	private:
		int good_points;									// how many eligible points passed thresholding
		int nan_values;										// how many Not-A-Number points were caught
		int inf_values;										// how many infinite valued points were caught
		int out_of_bounds_int;									// how many intensity values are outside 8-bits

	public:
		vector <float> x,y,z,intensity;								// temporary coordinate vectors
		vector <float> x_cluster_set, y_cluster_set, z_cluster_set;				// 3D coordinates to be published
		vector <float> d_cluster_set;								// euclidean distance vector for sorting
		int num_clusters;									// number of clusters in current frame

		// Constructor

		Clouds(void)
		{
			// resets all private values upon construction

			good_points = 0;
			nan_values = 0;
			inf_values = 0;
			out_of_bounds_int = 0;
		}

		// Filter raw points into usable vector data within a region of interest

		void get_ROI(PointCloud<PointXYZI>::Ptr temp_cloud)
		{
			for(uint raw_idx = 0; raw_idx <= temp_cloud->points.size(); raw_idx++)
			{
				// No Not-A-Number values please

				if(isnan(temp_cloud->points[raw_idx].x) || isnan(temp_cloud->points[raw_idx].y) || isnan(temp_cloud->points[raw_idx].z) || isnan(temp_cloud->points[raw_idx].intensity))
				{
			    		nan_values++;
				}

				// No Infinite values please

				else if(isinf(temp_cloud->points[raw_idx].x) || isinf(temp_cloud->points[raw_idx].y) || isinf(temp_cloud->points[raw_idx].z) || isinf(temp_cloud->points[raw_idx].intensity || fabs(temp_cloud->points[raw_idx].x) < 0.01 || fabs(temp_cloud->points[raw_idx].y) < 0.0001 || fabs(temp_cloud->points[raw_idx].z) < 0.0001))
				{
			    		inf_values++;
				}

				// No intensity values outside 8-bits please

				else if(temp_cloud->points[raw_idx].intensity < 0 || temp_cloud->points[raw_idx].intensity > 255)
				{
			    		out_of_bounds_int++;
				}

				// No points outside region of interest please

				else if(temp_cloud->points[raw_idx].x > 0 && temp_cloud->points[raw_idx].x < distance_limit && temp_cloud->points[raw_idx].y > -side_roi && temp_cloud->points[raw_idx].y < side_roi)
				{
					// create vector with filtered point cloud coordinates
		
					x.push_back(temp_cloud->points[raw_idx].x);
					y.push_back(temp_cloud->points[raw_idx].y);
					z.push_back(temp_cloud->points[raw_idx].z); 
					intensity.push_back(temp_cloud->points[raw_idx].intensity);
					good_points++;
				}
			}
		}

		// Filter out stray points

		void get_filtered_cloud(void)
		{
			if(good_points > 0)
			{
				// fill in new point cloud with valid data 

				cloud->points.resize(good_points);
				for(uint good_point_idx = 0; good_point_idx < good_points; good_point_idx++)
				{
					cloud->points[good_point_idx].x = x[good_point_idx];
					cloud->points[good_point_idx].y = y[good_point_idx];
					cloud->points[good_point_idx].z = z[good_point_idx];
					cloud->points[good_point_idx].intensity = intensity[good_point_idx];
				}

				// Create the filtering object: downsample the dataset using a leaf size of 1cm

				pcl::VoxelGrid<pcl::PointXYZI> vg;
				vg.setInputCloud (cloud);
				vg.setLeafSize (0.015f, 0.01f, 0.015f);
				vg.filter (*filtered_cloud);

				// filter out strays

				pcl::RadiusOutlierRemoval<pcl::PointXYZI> outrem;
				outrem.setInputCloud(filtered_cloud);
				outrem.setRadiusSearch(req_density_dist);
				outrem.setMinNeighborsInRadius (req_density_neighbors);
				outrem.filter (*filtered_cloud);
			}
		}

		// Group dense points and sort them in order of proximity

		void sort_clusters(void)
		{
			if(good_points > 0)
			{
				// grouping points

				pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
				tree->setInputCloud (filtered_cloud);
				std::vector<pcl::PointIndices> cluster_indices;
				pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
				ec.setClusterTolerance (object_separation); 
				ec.setMinClusterSize (min_cloud_density);
				ec.setMaxClusterSize (max_cloud_density);
				ec.setSearchMethod (tree);
				ec.setInputCloud (filtered_cloud);
				ec.extract (cluster_indices);

				// find center (or average) of clusters

				num_clusters = 0;
				for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
				{
					pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
					float x_cluster = 0;
					float y_cluster = 0;
					float z_cluster = 0;
					float distance = 0;
					for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
					{
						cloud_cluster->push_back ((*filtered_cloud)[*pit]);
						x_cluster += filtered_cloud->points[*pit].x;
						y_cluster += filtered_cloud->points[*pit].y;
						z_cluster += filtered_cloud->points[*pit].z;
					}
					x_cluster /= cloud_cluster->size ();
					y_cluster /= cloud_cluster->size ();
					z_cluster /= cloud_cluster->size ();

					// find euclidean distance 

					distance = pow(pow(x_cluster,2) + pow(y_cluster,2),0.5);

					// sort from nearest to furthest euclidean distance

					if(d_cluster_set.size() == 0)
					{
						d_cluster_set.push_back(distance);
						x_cluster_set.push_back(x_cluster);
						y_cluster_set.push_back(y_cluster);
						z_cluster_set.push_back(z_cluster);
					}
					else
					{
						bool inserted = false;
						for(int vec_idx = 0; vec_idx < d_cluster_set.size(); vec_idx++)
						{
							if(distance < d_cluster_set[vec_idx])
							{
								d_cluster_set.insert (d_cluster_set.begin() + vec_idx, distance);
								x_cluster_set.insert (x_cluster_set.begin() + vec_idx, x_cluster);
								y_cluster_set.insert (y_cluster_set.begin() + vec_idx, y_cluster);
								z_cluster_set.insert (z_cluster_set.begin() + vec_idx, z_cluster);
								inserted = true;
								break;
							}
						}
						if(inserted == false)
						{
							d_cluster_set.push_back(distance);
							x_cluster_set.push_back(x_cluster);
							y_cluster_set.push_back(y_cluster);
							z_cluster_set.push_back(z_cluster);
						}
					}
					cloud_cluster->width = cloud_cluster->size ();
					cloud_cluster->height = 1;
					cloud_cluster->is_dense = true;
					num_clusters++;
				}
			}
		}

		// clear data

		void clear_data(void)
		{
			good_points = 0;
			nan_values = 0;
			inf_values = 0;
			out_of_bounds_int = 0;
			filtered_cloud->clear();	
			cloud->clear();
			x.clear();
			y.clear();
			z.clear();
			intensity.clear();
			x_cluster_set.clear();
			y_cluster_set.clear();
			z_cluster_set.clear();
			d_cluster_set.clear();
		}
} CLOUD;

class Cluster
{
	public:
		visualization_msgs::MarkerArray cluster_avg;				// 3D Object Cluster visualization (white squares)
		vector<float> priority_distance;					// sorted euclidean distance vector 
		vector<float> priority_velocity;					// sorted velocity vector
		vector<float> last_distance;						// hold past localization values to see which objects are related to previous frame

		// update visualization and tracking

		void update(vector<float> x_vec, vector<float> y_vec, vector<float> z_vec, int num_clusters, vector<float> d)
		{
			// visualize big square as center of cluster

			geometry_msgs::Point cluster_coor;
			cluster_avg.markers.resize(num_clusters);
			for(uint candidate_count = 0; candidate_count < num_clusters; candidate_count++)
			{
				cluster_avg.markers[candidate_count].points.clear();
				cluster_avg.markers[candidate_count].header.frame_id = "velodyne";
				cluster_avg.markers[candidate_count].header.stamp = ros::Time();
				cluster_avg.markers[candidate_count].frame_locked = true;
				cluster_avg.markers[candidate_count].ns = "clusters";
				cluster_avg.markers[candidate_count].id = candidate_count;
				cluster_avg.markers[candidate_count].type = visualization_msgs::Marker::POINTS;
				cluster_avg.markers[candidate_count].action = 0;
				cluster_avg.markers[candidate_count].color.a = 1.0;
				cluster_avg.markers[candidate_count].lifetime = ros::Duration(1/freq);
				cluster_coor.x = x_vec[candidate_count];
				cluster_coor.y = y_vec[candidate_count];
				cluster_coor.z = z_vec[candidate_count];
				
				// track object by comparing previous distances to see what a potential object could have done between frames

				if(candidate_count <= num_objs - 1)
				{
					cluster_avg.markers[candidate_count].points.push_back(cluster_coor);
					priority_distance.push_back(d[candidate_count]);
					float min_diff = distance_limit;
					int min_idx = -1;
					for(uint old_idx = 0; old_idx < last_distance.size(); old_idx++)
					{
						if(fabs(d[candidate_count] - last_distance[old_idx]) <= movement_threshold)
						{
							float diff = fabs(d[candidate_count] - last_distance[old_idx]);
							if(min_diff > diff)
							{
								min_diff = diff;
								min_idx = old_idx;
							}
						}
					}

					// color code green as static object 

					if(min_idx == -1)
					{
						priority_velocity.push_back(0);
						cluster_avg.markers[candidate_count].scale.x = 0.25;
						cluster_avg.markers[candidate_count].scale.y = 0.25;
						cluster_avg.markers[candidate_count].scale.z = 0.25;
						cluster_avg.markers[candidate_count].color.r = 0.0;
						cluster_avg.markers[candidate_count].color.g = 1.0;
						cluster_avg.markers[candidate_count].color.b = 0.0;
					}

					// color code as yellow or red depending on velocity

					else
					{
						// same object from last frame and has non-negligible velocity

						if(freq*min_diff > static_threshold)
						{
							priority_velocity.push_back(freq*min_diff);
							cluster_avg.markers[candidate_count].scale.x = 0.25;
							cluster_avg.markers[candidate_count].scale.y = 0.25;
							cluster_avg.markers[candidate_count].scale.z = 0.25;
							cluster_avg.markers[candidate_count].color.b = 0.0;
							if(freq*min_diff > fast_threshold)
							{
								cluster_avg.markers[candidate_count].color.r = 1.0;
								cluster_avg.markers[candidate_count].color.g = 0.0;
							}
							else
							{
								if(freq*min_diff < fast_threshold/2)
								{
									cluster_avg.markers[candidate_count].color.r = (freq*min_diff)/(fast_threshold/2);
									cluster_avg.markers[candidate_count].color.g = 1.0;
								}
								else
								{
									cluster_avg.markers[candidate_count].color.g = (freq*min_diff)/(fast_threshold/2);
									cluster_avg.markers[candidate_count].color.r = 1.0;
								}
							}
						}

						// new object is given a velocity of 0 m/s

						else
						{
							priority_velocity.push_back(0);
							cluster_avg.markers[candidate_count].scale.x = 0.25;
							cluster_avg.markers[candidate_count].scale.y = 0.25;
							cluster_avg.markers[candidate_count].scale.z = 0.25;
							cluster_avg.markers[candidate_count].color.r = 0.0;
							cluster_avg.markers[candidate_count].color.g = 1.0;
							cluster_avg.markers[candidate_count].color.b = 0.0;
						}
						last_distance.erase(last_distance.begin() + min_idx);
					}
				}
					
				// cluster is outside the number of trackable objects desired currently

				else
				{
					cluster_avg.markers[candidate_count].points.push_back(cluster_coor);
					cluster_avg.markers[candidate_count].scale.x = 0.08;
					cluster_avg.markers[candidate_count].scale.y = 0.08;
					cluster_avg.markers[candidate_count].scale.z = 0.08;
					cluster_avg.markers[candidate_count].color.r = 1.0;
					cluster_avg.markers[candidate_count].color.g = 1.0;
					cluster_avg.markers[candidate_count].color.b = 1.0;
				}
			}

			// update past vector with current vector

			last_distance = priority_distance;
		}
} cluster_visualization;

// Agent Candidate Localization //////////////////

void cloud_callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& raw_cloud)
{
	// Clear and Initialize Vectors

	cluster_visualization.cluster_avg.markers.clear();
	cluster_visualization.priority_distance.clear();
	cluster_visualization.priority_velocity.clear();
	CLOUD.clear_data();

	// Convert PCL to PointCloud2

	PCLPointCloud2 pcl_pc2_raw;
	pcl_conversions::toPCL(*raw_cloud,pcl_pc2_raw);
	PointCloud<PointXYZI>::Ptr temp_cloud(new PointCloud<PointXYZI>);
	fromPCLPointCloud2(pcl_pc2_raw,*temp_cloud);

        // Filter out malicious values first

	CLOUD.get_ROI(temp_cloud);

	// Filter for dense objects

	CLOUD.get_filtered_cloud();

	// Create separate clusters
	
	CLOUD.sort_clusters();

	// Visualize white square where object clusters are located

	cluster_visualization.update(CLOUD.x_cluster_set, CLOUD.y_cluster_set, CLOUD.z_cluster_set, CLOUD.num_clusters, CLOUD.d_cluster_set); 
}
 
int main(int argc, char **argv)
{
	// Begin node //////////////////////////////////////////////
	
	init(argc, argv, "particle_tracking_node");
	NodeHandle nh;

	// Subscriber Declarations /////////////////////////////////
	
	Subscriber cloud_subscriber = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, cloud_callback);
	
	// Publisher Declarations //////////////////////////////////
	
	Publisher vis_pub = nh.advertise<visualization_msgs::MarkerArray>("/objects", 1);
	Publisher raw_publisher = nh.advertise<sensor_msgs::PointCloud2>("/raw_points", 1);
	Publisher filter_publisher = nh.advertise<sensor_msgs::PointCloud2>("/filter_points", 1);
	Publisher avoidance_pub = nh.advertise<auto_husky::avoidance_input>("/object_parameters", 1);
	sensor_msgs::PointCloud2 output_cloud, output_filter;
	auto_husky::avoidance_input input_msg;

	// ROS Time Initializations //////////////////////
	
	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();
	ros::Rate r(freq);

	while (ok())
	{
		// Update values

		spinOnce();
		toROSMsg(*cloud, output_cloud);
		toROSMsg(*filtered_cloud, output_filter);
		current_time = ros::Time::now();
				
		// Publish Point Cloud for Visualization

		output_cloud.header.stamp = current_time;
		output_cloud.header.frame_id = "velodyne";
		raw_publisher.publish(output_cloud);
		
		output_filter.header.stamp = current_time;
		output_filter.header.frame_id = "velodyne";
		filter_publisher.publish(output_filter);
	 
		vis_pub.publish(cluster_visualization.cluster_avg);

		input_msg.distance = cluster_visualization.priority_distance;
		input_msg.velocity = cluster_visualization.priority_velocity;
		avoidance_pub.publish(input_msg);
 
		// Time lapse
	 
		last_time = current_time;
		r.sleep();
	}
	return 0;
}
