//
// Created by will on 7/13/16.
//

#ifndef PROJECT_CGAL_UTILS_HPP
#define PROJECT_CGAL_UTILS_HPP

#include <build_surface/cgal_types.hpp>

void mark_domains(CT &ct, CT::Face_handle start, int index, std::list<CT::Edge> &border) {
    if (start->info().nesting_level != -1) return;
    std::list<CT::Face_handle> queue;
    queue.push_back(start);
    while (!queue.empty()) {
        CT::Face_handle fh = queue.front();
        if (fh->info().nesting_level == -1) {
            fh->info().nesting_level = index;
            for (int i = 0; i < 3; i++) {
                CT::Edge e(fh, i);
                CT::Face_handle n = fh->neighbor(i);
                if (n->info().nesting_level == -1) {
                    if (ct.is_constrained(e)) {
                        border.push_back(e);
                    } else {
                        queue.push_back(n);
                    }
                }
            }
        }
        queue.pop_front();
    }
}
// explore set of facets connected with non constrained edges,
// and attribute to each such set a nesting level.
// We start from facets incident to the infinite vertex, with a nesting
// level of 0. Then we recursively consider the non-explored facets incident
// to constrained edges bounding the former set and increase the nesting level by 1.
// Facets in the domain are those with an odd nesting level.
void mark_domains(CT &cdt) {
    for (auto it = cdt.all_faces_begin(); it != cdt.all_faces_end(); ++it) {
        it->info().nesting_level = -1;
    }
    std::list<CT::Edge> border;
    mark_domains(cdt, cdt.infinite_face(), 0, border);
    while (!border.empty()) {
        CT::Edge e = border.front();
        border.pop_front();
        CT::Face_handle n = e.first->neighbor(e.second);
        if (n->info().nesting_level == -1) {
            mark_domains(cdt, n, e.first->info().nesting_level + 1, border);
        }
    }
}

// Get the source vertex of an edge (templated so it works on both types of iterators)
template <typename T>
inline typename T::value_type::first_type::value_type::Vertex_handle edge_source(T &it) {
    return it->first->vertex(it->first->cw(it->second));
};

// Get target vertex of an edge (templated so it works on both types of iterators)
template <typename T>
inline typename T::value_type::first_type::value_type::Vertex_handle edge_target(T &it) {
    return it->first->vertex(it->first->ccw(it->second));
};

template <typename T>
inline bool is_in(const std::set<std::pair<uint32_t, uint32_t>> &set, T &it) {
    // Needs initializer-list syntax so it will return a non-const pair
    return set.find(std::minmax({edge_source(it)->info().pcl_index, edge_target(it)->info().pcl_index})) != set.end();
};

template <typename T>
inline bool add_to(std::set<std::pair<uint32_t, uint32_t>> &set, T &it) {
    // Needs initializer-list syntax so it will return a non-const pair
    return set.insert(std::minmax({edge_source(it)->info().pcl_index, edge_target(it)->info().pcl_index})).second;
};


#endif //PROJECT_CGAL_UTILS_HPP
