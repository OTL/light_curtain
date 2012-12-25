#ifndef LIGHT_CURTAIN_TYPES_H
#define LIGHT_CURTAIN_TYPES_H

typedef boost::function<bool (const pcl::PointCloud<pcl::PointXYZ>::Ptr&)> JudgeDanger;
typedef boost::function<void (bool)> InformFunction;
typedef boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::Ptr&)>
        PointCloudCallback;

#endif  // LIGHT_CURTAIN_TYPES_H

