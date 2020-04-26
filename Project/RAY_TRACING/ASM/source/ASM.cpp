#include <iostream>
#include <fstream>
#include <cstdlib>
#include <chrono>

#include "maths/Vector3.h"
#include "maths/Vector4.h"
#include "maths/Ray.h"
#include "maths/Sphere.h"

//#define SSE_OFF

float rand01()
{
    return (float)rand() / (RAND_MAX + 1.0f);
}

int main(int argc, const char* argv[])
{
    // Format for output image is PPM, can be opened with GIMP
    std::ofstream output("out.ppm", std::ios_base::out);

    if (!output.is_open())
        return -1;

    // Output image dimensions
    const int width = 200, height = 100;
    // Number of ray pack sent per pixel in the image
    const int rayPackPerPixel = 16;

    // PPM header
    output << "P3\n" << width << ' ' << height << "\n255\n";

    // Near plane configuration, ratio must match width & height
    const float horizontal = 4.f, vertical = 2.f;
    const float widthIncrement = horizontal / width;
    const float heightIncrement = vertical / height;

#ifdef SSE_OFF
    const float lowerLeftX = -horizontal * 0.5f, lowerLeftY = -vertical * 0.5f;
#else
    // Near plane lower left corner world coordinates
    const maths::Vector4 lowerLeftX(-horizontal * 0.5f);
    const maths::Vector4 lowerLeftY(-vertical * 0.5f);
#endif // SEE_ON

    // The primitive we will test rays against
    const maths::Sphere sphere(maths::Vector3(0.f, 0.f, 2.f), 1.f);

    // Colors for the sphere and the background
    const maths::Vector3 red(1.f, 0.f, 0.f), skyBlue(0.529f, 0.808f, 0.922f);

    // Intermediate integer values for the generated color
    // that will be output to the image file
    int32_t ir, ig, ib;

    auto start = std::chrono::system_clock::now();

    // Iterate over all pixels in the order expected by the PPM format
    for (int y = height - 1; y >= 0; y--)
    {
        for (int x = 0; x < width; x++)
        {
            // Base color is black
            maths::Vector3 color(0.f);

#ifdef SSE_OFF
            for (int r = 0; r < rayPackPerPixel * 4; r++)
            {
                float u = ((float)x + rand01()) * (float)widthIncrement;
                float v = ((float)y + rand01()) * (float)heightIncrement;

                maths::Vector3 origin(0.f);
                maths::Vector3 direction(lowerLeftX + u, lowerLeftY + v, 1.f);
                maths::Ray ray(origin, direction);

                bool hit = sphere.hit(ray);

                color += hit ? red : skyBlue;
            }

            color /= rayPackPerPixel * 4.f;
#else
            for (int r = 0; r < rayPackPerPixel; r++)
            {
                // Initialize the rays originating at the camera's position (0, 0, 0)
                // and with a random direction that makes them go through the current pixel
                maths::Vector3Packed directions(
                    lowerLeftX + maths::Vector4(x + rand01(), x + rand01(), x + rand01(), x + rand01()) * widthIncrement,
                    lowerLeftY + maths::Vector4(y + rand01(), y + rand01(), y + rand01(), y + rand01()) * heightIncrement,
                    maths::Vector4(1.f)
                );
                directions.normalize();

                maths::RayPacked rays(
                    maths::Vector3Packed(0.f, 0.f, 0.f),
                    directions
                );

                // Get the number of rays in the pack that have hit the sphere and
                // blend between the sphere and background colors with this ratio
                int hitCount = sphere.hit(rays);
                color += lerp(skyBlue, red, hitCount / 4.0f);
            }

            // Normalize the pixel color by the number or ray pack sent
            color /= (float)rayPackPerPixel;
#endif // SSE_OFF

            // Output the color to the file as expected by the PPM fromat
            ir = (int32_t)(255.99f * color.r);
            ig = (int32_t)(255.99f * color.g);
            ib = (int32_t)(255.99f * color.b);

            output << ir << ' ' << ig << ' ' << ib << '\n';
        }
    }

    auto end = std::chrono::system_clock::now();
    long long duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << (duration / 1000.f) << " seconds" << std::endl;

    // Close the output file
    output.close();

    return 0;
}