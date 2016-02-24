//
// Created by will on 2/15/16.
//

#include <pluginlib/class_list_macros.h>
#include <surface_manager/SurfaceManager.h>


#define INLIERS_TOPIC "new_surface_inliers"
#define CONCAVE_HULL_TOPIC "new_surface_concave_hull"
#define PLANE_TOPIC "new_surface_plane"

void surface_manager::SurfaceManager::onInit() {
    ros::NodeHandle& private_nh = getPrivateNodeHandle();

    int max_queue_size = 3;
    if (!private_nh.getParam("max_queue_size", max_queue_size)) {
        max_queue_size = 3;
    }

    // Advertise a topic called "surfaces" with a queue size of max_queue_size and latched=true
    surfaces_pub_ = private_nh.advertise<surfaces::Surfaces<PointIn> >("surfaces", (uint32_t) max_queue_size, true);

    // Subscribe to new_surface_* inputs (for adding surfaces)
    new_surface_inliers_sub_.subscribe(private_nh, INLIERS_TOPIC, (uint32_t) max_queue_size);
    new_surface_convex_hull_sub_.subscribe(private_nh, CONCAVE_HULL_TOPIC, (uint32_t) max_queue_size);
    new_surface_plane_sub_.subscribe(private_nh, PLANE_TOPIC, (uint32_t) max_queue_size);

    new_surface_synchronizer_ = boost::make_shared<NewSurfaceSynchronizer>(max_queue_size);
    new_surface_synchronizer_->connectInput(new_surface_inliers_sub_, new_surface_convex_hull_sub_, new_surface_plane_sub_);
    new_surface_synchronizer_->registerCallback(bind(&SurfaceManager::add_surface_synchronized, this, _1, _2, _3));

    NODELET_DEBUG("[%s::onInit] SurfaceManager started with max_queue_size = %d\n", getName().c_str(), max_queue_size);
    
    NODELET_DEBUG("[%s::onInit] Subscribed to topics %s, %s, %s", getName().c_str(),
                  private_nh.resolveName(INLIERS_TOPIC).c_str(),
                  private_nh.resolveName(CONCAVE_HULL_TOPIC).c_str(),
                  private_nh.resolveName(PLANE_TOPIC).c_str());
}

void surface_manager::SurfaceManager::add_surface_synchronized(PointCloudIn::ConstPtr inliers,
                                                               surfaces::Polygons::ConstPtr convex_hull,
                                                               pcl::ModelCoefficients::ConstPtr plane) {
    NODELET_DEBUG("New Surface received with %lu points, %lu vertices", inliers->size(), convex_hull->size());

    unsigned long id = surfaces.size();
    surfaces.resize(id+1);

    surfaces[id].id = (unsigned int) id;
    surfaces[id].color = std_msgs::ColorRGBA();
    surfaces[id].color.r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    surfaces[id].color.g = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    surfaces[id].color.b = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    surfaces[id].color.a = 1;
    surfaces[id].inliers = *inliers;
    surfaces[id].convex_hull = *convex_hull;
    surfaces[id].plane = *plane;

    std::sort(surfaces.begin(), surfaces.end(), [](surfaces::Surface<PointIn> a, surfaces::Surface<PointIn> b) {
        return b.convex_hull.size() > a.convex_hull.size();
    });

    surfaces::Surfaces<PointIn>::Ptr surfaces_msg = boost::make_shared<surfaces::Surfaces<PointIn> >();
    surfaces_msg->header = inliers->header;
    surfaces_msg->surfaces = surfaces;

    surfaces_pub_.publish(surfaces_msg);
}

PLUGINLIB_EXPORT_CLASS(surface_manager::SurfaceManager, nodelet::Nodelet)
