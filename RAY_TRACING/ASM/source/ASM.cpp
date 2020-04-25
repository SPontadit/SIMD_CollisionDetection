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
    std::ofstream output("out.ppm", std::ios_base::out);

    if (!output.is_open())
        return -1;

    const int width = 200, height = 100;
    const int rayPackPerPixel = 16;

    output << "P3\n" << width << ' ' << height << "\n255\n";

    const float horizontal = 4.f, vertical = 2.f;
    const float widthIncrement = horizontal / width;
    const float heightIncrement = vertical / height;

#ifdef SSE_OFF
    const float lowerLeftX = -horizontal * 0.5f, lowerLeftY = -vertical * 0.5f;
#else
    const maths::Vector4 lowerLeftX(-horizontal * 0.5f);
    const maths::Vector4 lowerLeftY(-vertical * 0.5f);
#endif // SEE_ON
    const maths::Sphere sphere(maths::Vector3(0.f, 0.f, 2.f), 1.f);

    const maths::Vector3 red(1.f, 0.f, 0.f), skyBlue(0.529f, 0.808f, 0.922f);

    uint8_t ir, ig, ib;

    auto start = std::chrono::system_clock::now();

    for (int y = height - 1; y >= 0; y--)
    {
        for (int x = 0; x < width; x++)
        {
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

                int hitCount = sphere.hit(rays);
                color += lerp(skyBlue, red, hitCount / 4.0f);
            }

            color /= (float)rayPackPerPixel;
#endif // SSE_OFF

            ir = (uint8_t)(255.99f * color.r);
            ig = (uint8_t)(255.99f * color.g);
            ib = (uint8_t)(255.99f * color.b);

            output << (int32_t)ir << ' ' << (int32_t)ig << ' ' << (int32_t)ib << '\n';
        }
    }

    auto end = std::chrono::system_clock::now();
    long long duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << (duration / 1000.f) << " seconds" << std::endl;

    output.close();

    return 0;
}