//
// Created by will on 6/21/16.
//

#ifndef PROJECT_SMART_PTR_HPP
#define PROJECT_SMART_PTR_HPP

#include <memory>

struct null_deleter {
    void operator()(void const *) const {}
};

template<typename T, typename ...Args>
std::unique_ptr<T> std_make_unique( Args&& ...args )  {
    return std::unique_ptr<T>( new T( std::forward<Args>(args)... ) );
}

#endif //PROJECT_SMART_PTR_HPP
