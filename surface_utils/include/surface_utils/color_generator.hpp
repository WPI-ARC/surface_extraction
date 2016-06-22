//
// Created by will on 6/22/16.
//

#ifndef PROJECT_RANDOM_COLORS_HPP
#define PROJECT_RANDOM_COLORS_HPP

#include <random>
#include <tuple>

namespace random_colors {
// Generate random colors with method described here:
// http://martankerl.com/2009/12/09/how-to-create-random-colors-programmatically/
class color_generator {
    static constexpr double golden_ratio_conjugate = 0.618033988749895;

    double h;
    const double s;
    const double v;

public:
    color_generator(double s, double v) : s(s), v(v) {
        std::uniform_real_distribution<double> unif(0., 1.);
        std::random_device rand_dev;
        std::mt19937 rand_engine(rand_dev());

        h = unif(rand_engine);
    }

    std::tuple<double, double, double> hsv() {
        h += golden_ratio_conjugate;
        if (h > 1) h -= 1; // Equivalent to h %= 1

        return std::make_tuple(h, s, v);
    }

    std::tuple<double, double, double> rgb() { return hsv_to_rgb(hsv()); }

    std::tuple<double, double, double> hsv_to_rgb(std::tuple<double, double, double> hsv) {
        double h, s, v, r, g, b;
        std::tie(h, s, v) = hsv;
        h *= 360; // Convert (0. < h < 1.) to (0. < h < 360.)

        double hh, p, q, t, ff;
        long i;

        if (s <= 0.0) { // Greyscale
            r = v;
            g = v;
            b = v;
            return std::make_tuple(r, g, b);
        }

        hh = h;
        if (hh >= 360.0) hh = 0.0;
        hh /= 60.0;
        i = (long)hh;
        ff = hh - i;
        p = v * (1.0 - s);
        q = v * (1.0 - (s * ff));
        t = v * (1.0 - (s * (1.0 - ff)));

        switch (i) {
        case 0:
            r = v;
            g = t;
            b = p;
            break;
        case 1:
            r = q;
            g = v;
            b = p;
            break;
        case 2:
            r = p;
            g = v;
            b = t;
            break;

        case 3:
            r = p;
            g = q;
            b = v;
            break;
        case 4:
            r = t;
            g = p;
            b = v;
            break;
        case 5:
        default:
            r = v;
            g = p;
            b = q;
            break;
        }

        return std::make_tuple(r, g, b);
    }
};
}

#endif // PROJECT_RANDOM_COLORS_HPP
