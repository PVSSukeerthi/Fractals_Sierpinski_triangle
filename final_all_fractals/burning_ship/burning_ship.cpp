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

using namespace std;

using Pixel = std::array<uint8_t, 3>;

static inline void putPixel(std::vector<Pixel>& img, int W, int H, int x, int y, uint8_t r, uint8_t g, uint8_t b){
    if (x>=0 && x<W && y>=0 && y<H) img[y*W + x] = {r,g,b};
}
template<typename T>
static inline T abs2(const std::complex<T>& z){
    return z.real()*z.real() + z.imag()*z.imag();
}

template<typename T>
int solve(complex<T> c, int maxIter) {
    complex<T> z{ T(0), T(0) };
    for (int i = 0; i < maxIter; ++i) {
        T zr = abs(z.real());
        T zi = abs(z.imag());

        // Burning Ship formula: (|Re(z)| + i|Im(z)|)^2 + c
        T new_re = zr*zr - zi*zi + c.real();
        T new_im = 2*zr*zi + c.imag();
        z = {new_re, new_im};

        if (abs2(z) > T(4)) {
            return i+1;
        }
    }
    return -1;  // inside set
}


template<typename T>
void burning_ship(const std::string& filename,int W, int H,double xmin, double xmax,double ymin, double ymax,int maxIter)
{
    vector<Pixel> img(W*H, {0,0,0});
    #pragma omp parallel for schedule(dynamic)
    for (int y=0; y<H; ++y){
        const double v = ymin + (y / double(H-1)) * (ymax - ymin);
        for (int x=0; x<W; ++x){
            const double u = xmin + (x / double(W-1)) * (xmax - xmin);

            complex<T> c{ T(u), T(v) };
            int iters = solve<T>(c, maxIter);
            uint8_t r=0,g=0,b=0;
            // r = static_cast<uint8_t> ((iters * 9) % 256);
            // g = static_cast<uint8_t> ((iters * 2) % 256);
            // b = static_cast<uint8_t> ((iters * 15) % 256);
            // if (iters == -1) r = g = b = 0; // black
            // uint8_t color = (iters == -1) ? 0 : uint8_t(255.0 * iters / maxIter);
            // r = g = b = color;
            if (iters == -1) {
                // Inside set → black
                r = g = b = 0;
            } else {
                // Simple cyclic color scheme like escRamCanvas
                int colorIndex = (iters * 10) % 768;

                if (colorIndex < 256) {
                    r = colorIndex; g = 0; b = 255 - colorIndex;
                } else if (colorIndex < 512) {
                    r = 255 - (colorIndex - 256); g = colorIndex - 256; b = 0;
                } else {
                    r = 0; g = 255 - (colorIndex - 512); b = colorIndex - 512;
                }
            }

            putPixel(img, W, H, x, y, r, g, b);

        }
    }

    stbi_write_png(filename.c_str(), W, H, 3, img.data(), W * 3);

}

int main() {
    const int    W = 800, H = 800;
    // const double xmin=-2, xmax=-1.7;
    // const double ymin=-0.1, ymax=0.1;
    const double xmin = -1.8;
    const double xmax = -1.7;
    const double ymin = -0.08;
    const double ymax = 0.08;

    const int maxIter = 200;
    using posit16_1  = sw::universal::posit<16,1>;
    using posit16_2  = sw::universal::posit<16,2>;
    using posit16_3  = sw::universal::posit<16,3>;
    using posit32_2 = sw::universal::posit<32,2>;
    using cfloat16_5 = sw::universal::cfloat<16,5>;
    using bfloat16_8 = sw::universal::bfloat16;

    burning_ship<posit16_1>("burning_ship_posit16_1.png",  W, H, xmin, xmax, ymin, ymax, maxIter);
    burning_ship<posit16_2>("burning_ship_posit16_2.png",  W, H, xmin, xmax, ymin, ymax, maxIter);
    burning_ship<posit16_3>("burning_ship_posit16_3.png",  W, H, xmin, xmax, ymin, ymax, maxIter);
    burning_ship<posit32_2>("burning_ship_posit32_2.png",  W, H, xmin, xmax, ymin, ymax, maxIter);

    burning_ship<cfloat16_5>("burning_ship_cfloat16_5.png", W, H, xmin, xmax, ymin, ymax, maxIter);
    burning_ship<float>("burning_ship_cfloat32_8.png", W, H, xmin, xmax, ymin, ymax, maxIter);
    burning_ship<double>("burning_ship_cfloat64_11.png", W, H, xmin, xmax, ymin, ymax, maxIter);

    burning_ship<bfloat16_8>("burning_ship_bfloat16_8.png", W, H, xmin, xmax, ymin, ymax, maxIter);

    return 0;
}
