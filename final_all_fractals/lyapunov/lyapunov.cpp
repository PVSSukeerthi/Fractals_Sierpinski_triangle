#include <iostream>
#include <fstream>
#include <vector>
#include <cstdint>
#include <cmath>
#include <complex>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include <universal/number/posit/posit.hpp>
#include <universal/number/cfloat/cfloat.hpp>
#include <universal/number/bfloat/bfloat.hpp>

#include <omp.h>

using namespace std;

// Pixel type
using Pixel = std::array<uint8_t, 3>;

static inline void putPixel(std::vector<Pixel>& img, int W, int H,
                            int x, int y, uint8_t r, uint8_t g, uint8_t b) {
    if (x>=0 && x<W && y>=0 && y<H) img[y*W + x] = {r,g,b};
}

// Compute Lyapunov exponent for logistic map with sequence "AB"
template<typename T>
T lyapunov_exponent(T a, T b, int maxIter, int discard=20) {
    static const string seq = "AB";
    size_t L = seq.size();
    T x = T(0.5);
    T le = T(0);
    for (int i = 0; i < maxIter + discard; i++) {
        T r = (seq[i % L] == 'A') ? a : b;
        x = r * x * (T(1) - x);
        if (x <= T(0) || x >= T(1)) return T(1); // escape
        if (i >= discard) {
            T d = abs(r * (T(1) - T(2) * x));
            
            // if (d > T(0)) le += log(d);

            if (d > T(0)) {
                if constexpr (std::is_same_v<T, sw::universal::bfloat16>) {
                    // for bfloat, cast to double
                    le += T(std::log((double)d));
                } else {
                    // for posit/cfloat etc., use native log
                    le += log(d);
                }
            }
        }
    }
    return le / T(maxIter);
}

// Render Lyapunov fractal for type T
template<typename T>
void render_lyapunov(const std::string& filename, int W, int H,
                     double amin, double amax, double bmin, double bmax,
                     int maxIter) {
    vector<Pixel> img(W*H, {0,0,0});

    #pragma omp parallel for schedule(dynamic)
    for (int y=0; y<H; y++) {
        double bv = bmin + (y / double(H-1)) * (bmax - bmin);
        for (int x=0; x<W; x++) {
            double av = amin + (x / double(W-1)) * (amax - amin);

            T a = T(av);
            T b = T(bv);

            T le = lyapunov_exponent<T>(a, b, maxIter);

            // Coloring: blue for negative (stable), red for positive (chaotic)
            uint8_t r=0,g=0,bb=0;
            if (le < T(0)) {
                uint8_t shade = uint8_t(std::min(255.0, 255.0 * (double)(-le)));
                r = 0; g = 0; bb = shade;
            } else {
                uint8_t shade = uint8_t(std::min(255.0, 255.0 * (double)(le)));
                r = shade; g = 0; bb = 0;
            }

            putPixel(img, W, H, x, y, r, g, bb);
        }
    }

    stbi_write_png(filename.c_str(), W, H, 3, img.data(), W * 3);
}

int main() {
    const int W = 800, H = 800;
    const double amin = 3.6, amax = 3.9;
    const double bmin = 2.7, bmax = 3.0;
    
    const int maxIter = 200;
    using posit16_1  = sw::universal::posit<16,1>;
    using posit16_2  = sw::universal::posit<16,2>;
    using posit16_3  = sw::universal::posit<16,3>;
    using posit32_2 = sw::universal::posit<32,2>;
    using cfloat16_5 = sw::universal::cfloat<16,5>;
    using bfloat16_8 = sw::universal::bfloat16;


    render_lyapunov<posit16_1>("lyapunov_posit16_1.png",  W, H, amin, amax, bmin, bmax, maxIter);
    render_lyapunov<posit16_2>("lyapunov_posit16_2.png",  W, H, amin, amax, bmin, bmax, maxIter);
    render_lyapunov<posit16_3>("lyapunov_posit16_3.png",  W, H, amin, amax, bmin, bmax, maxIter);
    render_lyapunov<posit32_2>("lyapunov_posit32_2.png",  W, H, amin, amax, bmin, bmax, maxIter);

    render_lyapunov<cfloat16_5>("lyapunov_cfloat16_5.png", W, H, amin, amax, bmin, bmax, maxIter);
    render_lyapunov<float>("lyapunov_cfloat32_8.png", W, H, amin, amax, bmin, bmax, maxIter);
    render_lyapunov<double>("lyapunov_cfloat64_11.png", W, H, amin, amax, bmin, bmax, maxIter);

    render_lyapunov<bfloat16_8>("lyapunov_bfloat16_8.png", W, H, amin, amax, bmin, bmax, maxIter);

    return 0;
}
