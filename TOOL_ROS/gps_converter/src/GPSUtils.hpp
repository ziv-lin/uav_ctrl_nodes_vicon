#include <cmath>

class GPSUtils
{
  public:
    // WGS-84 geodetic constants
    static constexpr double a = 6378137;          // WGS-84 Earth semimajor axis (m)
    static constexpr double b = 6356752.3142;     // WGS-84 Earth semiminor axis (m)
    static constexpr double f = ( a - b ) / a;    // Ellipsoid Flatness
    static constexpr double e_sq = f * ( 2 - f ); // Square of Eccentricity

    static double DegreesToRadians( double d )
    {
        return d / 180.0 * M_PI;
    };

    // Converts WGS-84 Geodetic point (lat, lon, h) to the
    // Earth-Centered Earth-Fixed (ECEF) coordinates (x, y, z).
    static void GeodeticToEcef( double lat, double lon, double h, double &x, double &y, double &z )
    {
        // Convert to radians in notation consistent with the paper:
        double lambda = DegreesToRadians( lat );
        double phi = DegreesToRadians( lon );
        double s = std::sin( lambda );
        double N = a / std::sqrt( 1 - e_sq * s * s );

        double sin_lambda = std::sin( lambda );
        double cos_lambda = std::cos( lambda );
        double cos_phi = std::cos( phi );
        double sin_phi = std::sin( phi );

        x = ( h + N ) * cos_lambda * cos_phi;
        y = ( h + N ) * cos_lambda * sin_phi;
        z = ( h + ( 1 - e_sq ) * N ) * sin_lambda;
    };

    // Converts the Earth-Centered Earth-Fixed (ECEF) coordinates (x, y, z) to
    // East-North-Up coordinates in a Local Tangent Plane that is centered at the
    // (WGS-84) Geodetic point (lat0, lon0, h0).
    static void EcefToEnu( double  x,
                           double  y,
                           double  z,
                           double  lat0,
                           double  lon0,
                           double  h0,
                           double &xEast,
                           double &yNorth,
                           double &zUp )
    {
        // Convert to radians in notation consistent with the paper:
        double lambda = DegreesToRadians( lat0 );
        double phi = DegreesToRadians( lon0 );
        double s = std::sin( lambda );
        double N = a / std::sqrt( 1 - e_sq * s * s );

        double sin_lambda = std::sin( lambda );
        double cos_lambda = std::cos( lambda );
        double cos_phi = std::cos( phi );
        double sin_phi = std::sin( phi );

        double x0 = ( h0 + N ) * cos_lambda * cos_phi;
        double y0 = ( h0 + N ) * cos_lambda * sin_phi;
        double z0 = ( h0 + ( 1 - e_sq ) * N ) * sin_lambda;

        double xd, yd, zd;
        xd = x - x0;
        yd = y - y0;
        zd = z - z0;

        // This is the matrix multiplication
        xEast = -sin_phi * xd + cos_phi * yd;
        yNorth = -cos_phi * sin_lambda * xd - sin_lambda * sin_phi * yd + cos_lambda * zd;
        zUp = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd;
    };

    // Converts the geodetic WGS-84 coordinated (lat, lon, h) to
    // East-North-Up coordinates in a Local Tangent Plane that is centered at the
    // (WGS-84) Geodetic point (lat0, lon0, h0).
    static void GeodeticToEnu( double  lat,
                               double  lon,
                               double  h,
                               double  lat0,
                               double  lon0,
                               double  h0,
                               double &xEast,
                               double &yNorth,
                               double &zUp )
    {
        double x, y, z;
        GeodeticToEcef( lat, lon, h, x, y, z );
        EcefToEnu( x, y, z, lat0, lon0, h0, xEast, yNorth, zUp );
    };
};