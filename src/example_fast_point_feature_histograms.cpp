#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <stdio.h>
#include <dirent.h>
#include <pcl/features/vfh.h>
#include <sys/stat.h>
#include <fstream>

#define NORMAL_SEARCH_RADIUS (0.05f)
#define FPFH_SEARCH_RADIUS (0.25f)

void List_pcd(char *path,  std::list<std::string> & Pcd_List)
{
  //  printf("路径为[%s]\n", path);

    struct dirent* ent = NULL;
    DIR *pDir;
    pDir = opendir(path);
    //d_reclen：16表示子目录或以.开头的隐藏文件，24表示普通文本文件,28为二进制文件，还有其他……
    while (NULL != (ent=readdir(pDir)))
    {
        if (ent->d_reclen == 32)
        {
  //      std::cout << std::string(path)+"/"+ent->d_name << std::endl;
        Pcd_List.push_back(std::string(path)+"/"+ent->d_name);
        }
    }
}

char *itoa(int value,char *string,int radix)
{
   int rt=0;
   if(string==NULL)
      return NULL;
   if(radix<=0||radix>30)
      return NULL;
   rt=snprintf(string,radix,"%d",value);
   if(rt>radix)
      return NULL;
   string[rt]='\0';
   return string;
}

void Get_ToTal_FPFH(const char *path_pcd,float normal_search_radius, float fpfh_search_radius, float a[33])
{
	  std::string fileName = path_pcd;
	//  std::cout << "Reading " << fileName << std::endl;

	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	  if (pcl::io::loadPCDFile<pcl::PointXYZ> (fileName, *cloud) == -1) // load the file
	  {
	    PCL_ERROR ("Couldn't read file");
	    exit(0);
	  }

	//  std::cout << "Loaded " << cloud->points.size () << " points." << std::endl;
	  // Compute the normals
	  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
	  normal_estimation.setInputCloud (cloud);

	  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	  normal_estimation.setSearchMethod (tree);

	  pcl::PointCloud<pcl::Normal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::Normal>);

	  normal_estimation.setRadiusSearch (0.05);

	  normal_estimation.compute (*cloud_with_normals);

	  // Setup the feature computation
	  pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_estimation;
	  // Provide the original point cloud (without normals)
	  fpfh_estimation.setInputCloud (cloud);
	  // Provide the point cloud with normals
	  fpfh_estimation.setInputNormals (cloud_with_normals);
	  // fpfhEstimation.setInputWithNormals(cloud, cloudWithNormals); PFHEstimation does not have this function
	  // Use the same KdTree from the normal estimation
	  fpfh_estimation.setSearchMethod (tree);

	  pcl::PointCloud<pcl::FPFHSignature33>::Ptr pfh_features (new pcl::PointCloud<pcl::FPFHSignature33>);

	  fpfh_estimation.setRadiusSearch (0.25);

	  // Actually compute the spin images
	  fpfh_estimation.compute (*pfh_features);

	//  std::cout << "output points.size (): " << pfh_features->points.size () << std::endl;
	// float a[33] = {0.0f};
	  // Display and retrieve the shape context descriptor vector for the 0th point.
	  for (std::size_t i = 0; i < pfh_features->points.size(); i++)
	    {
	      pcl::FPFHSignature33 descriptor = pfh_features->points[i];
	      for (int j = 0; j < 33; j++)
	    	  a[j] = descriptor.histogram[j];
	    }
}

std::vector<float> result[33];
int main()
{
	//char *pcd_path = "/home/x86isnice/key_point/test";
	std::ofstream out("../FPFH_log.txt");
	if (!out.is_open())
	{
		std::cout << "FPFH_log file is happen for a error ,please check" << std::endl;
		exit(0);///log_out用来备份数据对应的点云，从而从这个个集合中将错误点剔除
	}
	else
   {
		std::list<std::string> Pcd_List ;
	#if 1
		char *pcd_path = "/home/x86isnice/0627_keypoints_with_filter/dataset2";
		char *bump_path = "/home/x86isnice/0627_keypoints_with_filter/dataset1";
	#else
		char *pcd_path = "/home/x86isnice/0627_keypoints_with_filter/rest2test/dataset2";
		char *bump_path = "/home/x86isnice/0627_keypoints_with_filter/rest2test/dataset1";
	#endif
		List_pcd(pcd_path ,Pcd_List);

		std::cout << "@relation FPFH-result"  <<std::endl;
		for (int i = 0; i < 33; i++)
		{
			char str[20]={'\0'};
			std::cout << "@attribute FPFH" + std::string(itoa(i,str,10))<< " numeric"<<std::endl;
		}

		std::cout << "@attribute class {bump,flat}"  <<std::endl;
		std::cout << "@data" <<std::endl;

		for (std::list<std::string>::iterator it = Pcd_List.begin(); it != Pcd_List.end(); it++)
		{
			float a[33] = {0.0f};
			Get_ToTal_FPFH(it->c_str(), NORMAL_SEARCH_RADIUS, FPFH_SEARCH_RADIUS, a);
			out << *it << ":";
			for (int i = 0; i < 33; i++)
			{
				std::cout << *(a+i)  <<"," ;
                out << *(a+i)  <<"," ;
			}
                out << "flat"<< std::endl;
			std::cout << "flat"<< std::endl;
		}

		Pcd_List.clear();
		List_pcd(bump_path ,Pcd_List);

		for (std::list<std::string>::iterator it = Pcd_List.begin(); it != Pcd_List.end(); it++)
		{
			float a[33] = {0.0f};
			Get_ToTal_FPFH(it->c_str(), NORMAL_SEARCH_RADIUS, FPFH_SEARCH_RADIUS, a);
			out << *it << ":";
			for (int i = 0; i < 33; i++)
			{
			  std::cout << *(a+i)  <<"," ;
			  out << *(a+i)  <<"," ;
			}
			 out <<  "bump"<< std::endl;
			std::cout << "bump"<< std::endl;
		}
		out.close();
    }
    return 0;
}
