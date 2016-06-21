//
// Created by will on 6/20/16.
//

#ifndef PROJECT_MULTI_FUTURE_HPP
#define PROJECT_MULTI_FUTURE_HPP

#include <queue>
#include <memory>
#include <type_traits>

template<typename T, typename ...Args>
std::unique_ptr<T> make_unique( Args&& ...args )
{
    return std::unique_ptr<T>( new T( std::forward<Args>(args)... ) );
}

//template <typename T>
//class multi_future {
//public:
//    class resolver_interface {
//        template<typename I>
//        virtual void operator()(I val) = 0;
//
//        virtual ~resolver_interface() {}
//    };
//
//    template<typename ResolveT, typename CallbackT>
//    class resolver : resolver_interface {
//        const std::shared_ptr<multi_future<ResolveT>> future_;
//        const CallbackT func_;
//
//    public:
//        resolver(std::shared_ptr<multi_future<ResolveT>> future, CallbackT func) : future_(future), func_(func) {}
//
//        void operator() (ResolveT value) {
//            func_(value, std::bind(&multi_future::add_item, future_));
//        }
//    };
//
//    template<typename NewT, typename FuncT>
//    std::shared_ptr<multi_future<NewT>> then(FuncT func) {
//        auto new_future = std::make_shared<multi_future<NewT>>();
//
//        std::unique_ptr<resolver_interface> res(new resolver<NewT, FuncT>(new_future, func));
//
//        for (auto &item : items_) {
//            res->(item);
//        }
//
//        resolvers_.push_back(std::move(res));
//        return new_future;
//    }
//
//    std::vector<T> get() {
//        return items_;
//    }
//
//    T get_one() {
//        assert(items_.size() == 1);
//        return items_[0];
//    }
//
//protected:
//    std::vector<std::unique_ptr<resolver_interface>> resolvers_;
//    std::vector<T> items_;
//
//    void add_item(T item) {
//        items_.push_back(item);
//    }
//
//    // Makes other specializations of the class friends (required for .then)
//    template <typename U>
//    friend class multi_future;
//
//    template<class TT>
//    friend std::shared_ptr<multi_future<TT>> make_multi_future(TT val);
//};
//
//template<typename T>
//std::shared_ptr<multi_future<T>> make_multi_future(T val) {
//    std::shared_ptr<multi_future<T>> future;
//    future->add_item(val);
//    return future;
//}

#endif //PROJECT_MULTI_FUTURE_HPP
