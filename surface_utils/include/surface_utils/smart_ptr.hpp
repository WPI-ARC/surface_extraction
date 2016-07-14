//
// Created by will on 6/21/16.
//

#ifndef PROJECT_SMART_PTR_HPP
#define PROJECT_SMART_PTR_HPP

#include <memory>
#include <boost/shared_ptr.hpp>

struct null_deleter {
    void operator()(void const *) const {}
};

template<typename T, typename ...Args>
std::unique_ptr<T> std_make_unique( Args&& ...args )  {
    return std::unique_ptr<T>( new T( std::forward<Args>(args)... ) );
}

template <typename T>
std::shared_ptr<T> std_fake_shared(T &obj) {
    // Ailaising constructor to an empty shared pointer
    return std::shared_ptr<T>(std::shared_ptr<T>(), &obj);
}

template <typename T>
boost::shared_ptr<T> boost_fake_shared(T &obj) {
    // Ailaising constructor to an empty shared pointer
    return boost::shared_ptr<T>(boost::shared_ptr<T>(), &obj);
}

#endif //PROJECT_SMART_PTR_HPP
