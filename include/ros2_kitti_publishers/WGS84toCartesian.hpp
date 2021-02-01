/*
 * MIT License
 *
 * Copyright (c) 2018  Christian Berger
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef WGS84TOCARTESIAN_HPP
#define WGS84TOCARTESIAN_HPP

#include <cmath>
#include <array>
#include <limits>

namespace wgs84 {

/**
 * @param WGS84Reference WGS84 position to be used as reference.
 * @param WGS84Position WGS84 position to be transformed.
 * @return std::array<double, 2> Cartesian position after transforming WGS84Position using the given WGS84Reference using Mercator projection.
 */
inline std::array<double, 2> toCartesian(const std::array<double, 2> &WGS84Reference, const std::array<double, 2> &WGS84Position) {
#ifndef M_PI
    const double M_PI = 3.141592653589793;
#endif
    const double DEG_TO_RAD{M_PI / 180.0};
    const double HALF_PI{M_PI / 2.0};
    const double EPSILON10{1.0e-10};
    const double EPSILON12{1.0e-12};

    const double EQUATOR_RADIUS{6378137.0};
    const double FLATTENING{1.0 / 298.257223563};
    const double SQUARED_ECCENTRICITY{2.0 * FLATTENING - FLATTENING * FLATTENING};
    //constexpr double SQUARE_ROOT_ONE_MINUS_ECCENTRICITY{0.996647189335};
    //constexpr double POLE_RADIUS{EQUATOR_RADIUS * SQUARE_ROOT_ONE_MINUS_ECCENTRICITY};

    const double C00{1.0};
    const double C02{0.25};
    const double C04{0.046875};
    const double C06{0.01953125};
    const double C08{0.01068115234375};
    const double C22{0.75};
    const double C44{0.46875};
    const double C46{0.01302083333333333333};
    const double C48{0.00712076822916666666};
    const double C66{0.36458333333333333333};
    const double C68{0.00569661458333333333};
    const double C88{0.3076171875};

    const double R0{C00 - SQUARED_ECCENTRICITY * (C02 + SQUARED_ECCENTRICITY * (C04 + SQUARED_ECCENTRICITY * (C06 + SQUARED_ECCENTRICITY * C08)))};
    const double R1{SQUARED_ECCENTRICITY * (C22 - SQUARED_ECCENTRICITY * (C04 + SQUARED_ECCENTRICITY * (C06 + SQUARED_ECCENTRICITY * C08)))};
    const double R2T{SQUARED_ECCENTRICITY * SQUARED_ECCENTRICITY};
    const double R2{R2T * (C44 - SQUARED_ECCENTRICITY * (C46 + SQUARED_ECCENTRICITY * C48))};
    const double R3T{R2T * SQUARED_ECCENTRICITY};
    const double R3{R3T * (C66 - SQUARED_ECCENTRICITY * C68)};
    const double R4{R3T * SQUARED_ECCENTRICITY * C88};

    auto mlfn = [&](const double &lat) {
        const double sin_phi{std::sin(lat)};
        const double cos_phi{std::cos(lat) * sin_phi};
        const double squared_sin_phi = sin_phi * sin_phi;
        return (R0 * lat - cos_phi * (R1 + squared_sin_phi * (R2 + squared_sin_phi * (R3 + squared_sin_phi * R4))));
    };

    const double ML0{mlfn(WGS84Reference[0] * DEG_TO_RAD)};

    auto msfn = [&](const double &sinPhi, const double &cosPhi, const double &es) { return (cosPhi / std::sqrt(1.0 - es * sinPhi * sinPhi)); };

    auto project = [&](double lat, double lon) {
        std::array<double, 2> retVal{lon, -1.0 * ML0};
        if (!(std::abs(lat) < EPSILON10)) {
            const double ms{(std::abs(std::sin(lat)) > EPSILON10) ? msfn(std::sin(lat), std::cos(lat), SQUARED_ECCENTRICITY) / std::sin(lat) : 0.0};
            retVal[0] = ms * std::sin(lon *= std::sin(lat));
            retVal[1] = (mlfn(lat) - ML0) + ms * (1.0 - std::cos(lon));
        }
        return retVal;
    };

    auto fwd = [&](double lat, double lon) {
        const double D = std::abs(lat) - HALF_PI;
        if ((D > EPSILON12) || (std::abs(lon) > 10.0)) {
            return std::array<double, 2>{0.0, 0.0};
        }
        if (std::abs(D) < EPSILON12) {
            lat = (lat < 0.0) ? -1.0 * HALF_PI : HALF_PI;
        }
        lon -= WGS84Reference[1] * DEG_TO_RAD;
        const auto projectedRetVal{project(lat, lon)};
        return std::array<double, 2>{EQUATOR_RADIUS * projectedRetVal[0], EQUATOR_RADIUS * projectedRetVal[1]};
    };

    return fwd(WGS84Position[0] * DEG_TO_RAD, WGS84Position[1] * DEG_TO_RAD);
}

/**
 * @param WGS84Reference WGS84 position to be used as reference.
 * @param CartesianPosition Cartesian position to be transformed.
 * @return std::array<double, 2> Approximating a WGS84 position from a given CartesianPosition based on a given WGS84Reference using Mercator projection.
 */
inline std::array<double, 2> fromCartesian(const std::array<double, 2> &WGS84Reference, const std::array<double, 2> &CartesianPosition) {
    const double EPSILON10{1.0e-2};
    const double incLon{1e-5};
    const int32_t signLon{(CartesianPosition[0] < 0) ? -1 : 1};
    const double incLat{incLon};
    const int32_t signLat{(CartesianPosition[1] < 0) ? -1 : 1};

    std::array<double, 2> approximateWGS84Position{WGS84Reference};
    std::array<double, 2> cartesianResult{toCartesian(WGS84Reference, approximateWGS84Position)};

    double dPrev{(std::numeric_limits<double>::max)()};
    double d{std::abs(CartesianPosition[1] - cartesianResult[1])};
    while ((d < dPrev) && (d > EPSILON10)) {
        approximateWGS84Position[0] = approximateWGS84Position[0] + signLat * incLat;
        cartesianResult             = toCartesian(WGS84Reference, approximateWGS84Position);
        dPrev                       = d;
        d                           = std::abs(CartesianPosition[1] - cartesianResult[1]);
    }

    dPrev = (std::numeric_limits<double>::max)();
    d     = std::abs(CartesianPosition[0] - cartesianResult[0]);
    while ((d < dPrev) && (d > EPSILON10)) {
        approximateWGS84Position[1] = approximateWGS84Position[1] + signLon * incLon;
        cartesianResult             = toCartesian(WGS84Reference, approximateWGS84Position);
        dPrev                       = d;
        d                           = std::abs(CartesianPosition[0] - cartesianResult[0]);
    }

    return approximateWGS84Position;
}

struct Xy
{
    double x;
    double y;
};

inline Xy latlon2xy_helper(double lat, double lngd)
{
    // WGS 84 datum
    double eqRad = 6378137.0;
    double flat = 298.2572236;

    // constants used in calculations:
    double a = eqRad;           // equatorial radius in meters
    double f = 1.0 / flat;        // polar flattening
    double b = a * (1.0 - f);     // polar radius
    double e = sqrt(1.0 - (pow(b, 2) / pow(a, 2))); // eccentricity
    double k0 = 0.9996;
    double drad = M_PI / 180.0;

    double phi = lat * drad;   // convert latitude to radians
    double utmz = 1.0 + floor((lngd + 180.0) / 6.0); // longitude to utm zone
    double zcm = 3.0 + 6.0 * (utmz - 1.0) - 180.0;     // central meridian of a zone
    double esq = (1.0 - (b / a) * (b / a));
    double e0sq = e * e / (1.0 - e * e);
    double M = 0.0;
    double M0 = 0.0;
    double N = a / sqrt(1.0 - pow(e * sin(phi), 2));
    double T = pow(tan(phi), 2);
    double C = e0sq * pow(cos(phi), 2);
    double A = (lngd - zcm) * drad * cos(phi);

    // calculate M (USGS style)
    M = phi * (1.0 - esq * (1.0 / 4.0 + esq * (3.0 / 64.0 + 5.0 * esq / 256.0)));
    M = M - sin(2.0 * phi) * (esq * (3.0 / 8.0 + esq * (3.0 / 32.0 + 45.0 * esq / 1024.0)));
    M = M + sin(4.0 * phi) * (esq * esq * (15.0 / 256.0 + esq * 45.0 / 1024.0));
    M = M - sin(6.0 * phi) * (esq * esq * esq * (35.0 / 3072.0));
    M = M * a; // Arc length along standard meridian

    // now we are ready to calculate the UTM values...
    // first the easting (relative to CM)
    double x = k0 * N * A * (1.0 + A * A * ((1.0 - T + C) / 6.0 + A * A * (5.0 - 18.0 * T + T * T + 72.0 * C - 58.0 * e0sq) / 120.0));
    x = x + 500000.0; // standard easting

    // now the northing (from the equator)
    double y = k0 * (M - M0 + N * tan(phi) * (A * A * (1.0 / 2.0 + A * A * ((5.0 - T + 9.0 * C + 4.0 * C * C) / 24.0 + A * A * (61.0 - 58.0 * T + T * T + 600.0 * C - 330.0 * e0sq) / 720.0))));
    if (y < 0)
    {
        y = 10000000.0 + y; // add in false northing if south of the equator
    }
    double easting  = x;
    double northing = y;

    Xy coords;
    coords.x = easting;
    coords.y = northing;

    return coords;
}


}
#endif