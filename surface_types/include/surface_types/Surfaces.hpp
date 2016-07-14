#ifndef surface_types_HPP
#define surface_types_HPP

#include <pcl/PCLHeader.h>
#include "SurfaceData.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <surface_msgs2/Surfaces.h>
#include <surface_msgs2/SurfaceDetectionResponse.h>

namespace surface_types {

// Defined similarly to pcl::PointIndices
struct Surfaces {
    Surfaces() : header(), surfaces() {}

    pcl::PCLHeader header;

    std::vector<SurfaceData> surfaces;

public:
    void add_surface(const SurfaceData &s) {
        s.validate();

        surfaces.push_back(s);
    }

    void update_surface(const SurfaceData &s) {
        s.validate();

        auto posn = find_id(s.id);
        assert(posn != surfaces.end() && "Tried to update surface that isn't in this Surfaces object");
        (*posn) = s;
    }

    std::vector<SurfaceData>::iterator find_id(uint32_t surface_id) {
        return std::find_if(surfaces.begin(), surfaces.end(),
                            [&surface_id](const SurfaceData &s) { return s.id == surface_id; });
    }

    operator surface_msgs2::Surfaces() const {
        surface_msgs2::Surfaces ros_surfaces;

        pcl_conversions::fromPCL(header, ros_surfaces.header);
        ros_surfaces.surfaces.reserve(surfaces.size());
        std::copy(surfaces.begin(), surfaces.end(), std::back_inserter(ros_surfaces.surfaces));

        return ros_surfaces;
    }
    operator surface_msgs2::SurfaceDetectionResponse() const {
        surface_msgs2::SurfaceDetectionResponse ros_surfaces;

        ros_surfaces.surfaces.reserve(surfaces.size());
        std::copy(surfaces.begin(), surfaces.end(), std::back_inserter(ros_surfaces.surfaces));

        return ros_surfaces;
    }
    typedef boost::shared_ptr<::surface_types::Surfaces> Ptr;
    typedef boost::shared_ptr<::surface_types::Surfaces const> ConstPtr;
}; // struct surface_types

inline std::ostream &operator<<(std::ostream &s, const ::surface_types::Surfaces &v) {
    s << "header: " << std::endl;
    s << "  " << v.header;
    s << "surfaces[]" << std::endl;
    for (size_t i = 0; i < v.surfaces.size(); ++i) {
        s << "  surfaces[" << i << "]: ";
        s << "  " << v.surfaces[i] << std::endl;
    }
    return (s);
}
}

namespace ros {
namespace message_traits {
template <>
struct IsFixedSize<surface_types::Surfaces> : public FalseType {};
// TODO: Determine if this actually is a simple message
template <>
struct IsSimple<surface_types::Surfaces> : public FalseType {};
template <>
struct HasHeader<surface_types::Surfaces> : public FalseType {};

template <>
struct MD5Sum<surface_types::Surfaces> {
    static const char *value() { return MD5Sum<surface_msgs2::Surfaces>::value(); }

    static const char *value(const surface_types::Surfaces &m) { return MD5Sum<surface_msgs2::Surfaces>::value(); }
};

template <>
struct DataType<surface_types::Surfaces> {
    static const char *value() { return DataType<surface_msgs2::Surfaces>::value(); }

    static const char *value(const surface_types::Surfaces &m) { return DataType<surface_msgs2::Surfaces>::value(); }
};

template <>
struct Definition<surface_types::Surfaces> {
    static const char *value() { return Definition<surface_msgs2::Surfaces>::value(); }

    static const char *value(const surface_types::Surfaces &m) { return Definition<surface_msgs2::Surfaces>::value(); }
};

template <>
struct TimeStamp<surface_types::Surfaces> {
    static ros::Time *pointer(surface_types::Surfaces &m) {
        header_.reset(new std_msgs::Header());
        pcl_conversions::fromPCL(m.header, *(header_));
        return &(header_->stamp);
    }
    static ros::Time const *pointer(const surface_types::Surfaces &m) {
        header_const_.reset(new std_msgs::Header());
        pcl_conversions::fromPCL(m.header, *(header_const_));
        return &(header_const_->stamp);
    }
    static ros::Time value(const surface_types::Surfaces &m) { return pcl_conversions::fromPCL(m.header).stamp; }

private:
    static boost::shared_ptr<std_msgs::Header> header_;
    static boost::shared_ptr<std_msgs::Header> header_const_;
};
} // namespace message_traits

namespace serialization {
template <>
struct Serializer<surface_types::Surfaces> {
    template <typename Stream>
    inline static void write(Stream &stream, const surface_types::Surfaces &item) {
        stream.next(item.header);
        stream.next(item.surfaces);
    }

    template <typename Stream>
    inline static void read(Stream &stream, surface_types::Surfaces &item) {
        // Convert ROS header to PCL header
        std_msgs::Header ros_header;
        stream.next(ros_header);
        pcl_conversions::toPCL(ros_header, item.header);
        stream.next(item.surfaces);
    }

    inline static int32_t serializedLength(const surface_types::Surfaces &item) {
        uint32_t size = 0;
        size += serializationLength(item.header);
        size += serializationLength(item.surfaces);
        return size;
    }
};
} // namespace serialization
} // namespace ros

#endif // surface_types_HPP
