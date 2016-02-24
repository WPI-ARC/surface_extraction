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
    surfaces_pub_ = private_nh.advertise<Surfaces>("surfaces", (uint32_t) max_queue_size, true);

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

void surface_manager::SurfaceManager::add_surface_synchronized(const PointCloudIn::ConstPtr inliers,
                                                               const PolygonMesh::ConstPtr concave_hull,
                                                               const ModelCoefficients::ConstPtr model) {
    auto cloud_size = concave_hull->cloud.width * concave_hull->cloud.height;
    NODELET_DEBUG("New Surface received with %lu points, %u vertices", inliers->size(), cloud_size);

    unsigned long id = surfaces.size();
    surfaces.resize(id+1);

    surfaces[id].id = (unsigned int) id;
    surfaces[id].color = std_msgs::ColorRGBA();
    surfaces[id].color.r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    surfaces[id].color.g = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    surfaces[id].color.b = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    surfaces[id].color.a = 1;
    surfaces[id].model = *model;
    surfaces[id].concave_hull = *concave_hull;

//    surfaces[id].inliers = *inliers;

    std::sort(surfaces.begin(), surfaces.end(), [](const Surface& a, const Surface& b) {
        auto a_size = a.concave_hull.cloud.width * a.concave_hull.cloud.height;
        auto b_size = b.concave_hull.cloud.width * b.concave_hull.cloud.height;
        // A should come first (compare lower) if its size is greater than B's
        return a_size > b_size;
    });

    Surfaces::Ptr surfaces_msg = boost::make_shared<Surfaces>();
    surfaces_msg->header = concave_hull->header;
    surfaces_msg->surfaces = surfaces;

    surfaces_pub_.publish(surfaces_msg);
}

PLUGINLIB_EXPORT_CLASS(surface_manager::SurfaceManager, nodelet::Nodelet)
