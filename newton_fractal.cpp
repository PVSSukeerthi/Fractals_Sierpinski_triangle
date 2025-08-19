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

using namespace std;

using Pixel = std::array<uint8_t, 3>;

static inline void putPixel(std::vector<Pixel>& img, int W, int H, int x, int y, uint8_t r, uint8_t g, uint8_t b){
    if (x>=0 && x<W && y>=0 && y<H) img[y*W + x] = {r,g,b};
}
// squared magnitude of a complex number without sqrt
template<typename T>
static inline T abs2(const std::complex<T>& z){
    return z.real()*z.real() + z.imag()*z.imag();
}

// Single Newton solve; returns (rootIndex, iterations). rootIndex = -1 if not converged.
template<typename T>
pair<int,int> newton_solve(complex<T> z0, int maxIter, T tol, T eps) {
    const T tol2 = tol*tol;
    for (int i = 0; i < maxIter; ++i) {
        // f(z) = z^3 - 1, f'(z) = 3z^2
        const complex<T> z2 = z0*z0;
        const complex<T> z3 = z2*z0;
        const complex<T> f  = z3 - complex<T>(T(1), T(0));
        const complex<T> fp = complex<T>(T(3), T(0)) * z2;

        if (abs2(fp) < eps) break;                 // derivative too small -> unstable to divide
        const complex<T> step = f / fp;
        const complex<T> z1   = z0 - step;

        if (abs2(step) < tol2) {
            // Decide which root we converged to (nearest among the 3 cube roots of unity)
            // Roots in double for robust identification
            static const complex<double> rts[3] = {
                { 1.0,  0.0},
                {-0.5,  sqrt(3.0)/2.0},
                {-0.5, -sqrt(3.0)/2.0}
            };
            const double xr = static_cast<double>(z1.real());
            const double yi = static_cast<double>(z1.imag());
            double bestd = 1e300; int best = -1;

            for (int k=0;k<3;++k){
                const double dx = xr - rts[k].real();
                const double dy = yi - rts[k].imag();
                const double d2 = dx*dx + dy*dy;
                if (d2 < bestd){ bestd = d2; best = k; }
            }
            return {best, i+1};
        }
        z0 = z1;
    }
    return {-1, maxIter};
}

// Render a Newton fractal image for type T
template<typename T>
void render_newton(const std::string& filename,int W, int H,double xmin, double xmax,double ymin, double ymax,int maxIter,double tol_d = 1e-3,double eps_d = 1e-20)
{
    vector<Pixel> img(W*H, {0,0,0});

    const T tol = T(tol_d);
    const T eps = T(eps_d);
    
    for (int y=0; y<H; ++y){
        const double v = ymin + (y / double(H-1)) * (ymax - ymin);
        for (int x=0; x<W; ++x){
            const double u = xmin + (x / double(W-1)) * (xmax - xmin);

            complex<T> z0{ T(u), T(v) };
            auto [root, iters] = newton_solve<T>(z0, maxIter, tol, eps);
            uint8_t shade = (root >= 0) ? static_cast<uint8_t>( 255u * iters / std::max(1, maxIter) ) : 0;

            uint8_t r=0,g=0,b=0;
            switch (root){
                case 0: r = 200; g = shade; b = shade/2; break; // red-ish
                case 1: r = shade/2; g = 200; b = shade; break; // green-ish
                case 2: r = shade; g = shade/2; b = 200; break; // blue-ish
                default: r = g = b = 0; break;                   // non-converged/unstable
            }
            putPixel(img, W, H, x, y, r, g, b);
        }
    }

    stbi_write_png(filename.c_str(), W, H, 3, img.data(), W * 3);

}

int main() {
    const int    W = 512, H = 512;
    const double xmin=-2.0, xmax=2.0, ymin=-2.0, ymax=2.0;
    const int    maxIter = 20;

    using posit16  = sw::universal::posit<16,2>;
    using cfloat16 = sw::universal::cfloat<16,5>;

    render_newton<posit16>("newton3_posit16.png",  W, H, xmin, xmax, ymin, ymax, maxIter);
    render_newton<cfloat16>("newton3_cfloat16.png", W, H, xmin, xmax, ymin, ymax, maxIter);
    render_newton<float>("newton3_float.png", W, H, xmin, xmax, ymin, ymax, maxIter);
    render_newton<double>("newton3_double.png", W, H, xmin, xmax, ymin, ymax, maxIter);


    return 0;
}
