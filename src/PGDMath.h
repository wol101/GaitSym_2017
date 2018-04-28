// Handy geometry classes from Physics for Game Developers
// Altered to produce a pgd_floating_point_type precision version

#ifndef _MYMATH
#define _MYMATH

#include <cmath>
#include <cfloat>

#if defined(dSINGLE)
typedef float pgd_floating_point_type;
#else
typedef double pgd_floating_point_type;
#endif

#define MYMATH_ABS(a) ((a) >= 0 ? (a) : -(a))
#define MYMATH_CLAMP(value, low, high) (((value)<(low))?(low):(((value)>(high))?(high):(value)))

#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif

// wis  - namespace to avoid naming problems
namespace pgd
{

    //------------------------------------------------------------------------//
    // Misc. Constants
    //------------------------------------------------------------------------//

    pgd_floating_point_type   const   pi  = M_PI;

#ifdef dSINGLE
    pgd_floating_point_type const epsilon = FLT_EPSILON; // 1 + this value is detectable
    pgd_floating_point_type const minPositive = FLT_MIN; // the minimum positive number
    pgd_floating_point_type const maxPositive = FLT_MAX; // the maximum positive
#else
    pgd_floating_point_type const epsilon = DBL_EPSILON; // 1 + this value is detectable
    pgd_floating_point_type const minPositive = DBL_MIN; // the minimum positive number
    pgd_floating_point_type const maxPositive = DBL_MAX; // the maximum positive
#endif

    //------------------------------------------------------------------------//
    // Misc. Functions
    //------------------------------------------------------------------------//
    inline  pgd_floating_point_type   DegreesToRadians(pgd_floating_point_type deg);
    inline  pgd_floating_point_type   RadiansToDegrees(pgd_floating_point_type rad);

    inline  pgd_floating_point_type   DegreesToRadians(pgd_floating_point_type deg)
    {
        return deg * pi / 180.0;
    }

    inline  pgd_floating_point_type   RadiansToDegrees(pgd_floating_point_type rad)
    {
        return rad * 180.0 / pi;
    }

    //------------------------------------------------------------------------//
    // Vector Class and vector functions
    //------------------------------------------------------------------------//
    class Vector {
public:
        pgd_floating_point_type x;
        pgd_floating_point_type y;
        pgd_floating_point_type z;

        Vector(void);
        Vector(pgd_floating_point_type xi, pgd_floating_point_type yi, pgd_floating_point_type zi);

        void Set(pgd_floating_point_type xi, pgd_floating_point_type yi, pgd_floating_point_type zi);

        pgd_floating_point_type Magnitude(void);
        pgd_floating_point_type Magnitude2(void);
        void  Normalize(void);
        void  Reverse(void);

        Vector& operator+=(Vector u);   // vector addition
        Vector& operator-=(Vector u);   // vector subtraction
        Vector& operator*=(pgd_floating_point_type s);    // scalar multiply
        Vector& operator/=(pgd_floating_point_type s);    // scalar divide
        Vector& operator=(pgd_floating_point_type *s);    // assign from POD array

        Vector operator-(void);

    };

    inline  Vector operator+(Vector u, Vector v);
    inline  Vector operator-(Vector u, Vector v);
    inline  Vector operator^(Vector u, Vector v);
    inline  pgd_floating_point_type operator*(Vector u, Vector v);
    inline  Vector operator*(pgd_floating_point_type s, Vector u);
    inline  Vector operator*(Vector u, pgd_floating_point_type s);
    inline  Vector operator/(Vector u, pgd_floating_point_type s);
    inline  pgd_floating_point_type TripleScalarProduct(Vector u, Vector v, Vector w);

    inline Vector::Vector(void)
    {
        x = 0;
        y = 0;
        z = 0;
    }

    inline Vector::Vector(pgd_floating_point_type xi, pgd_floating_point_type yi, pgd_floating_point_type zi)
    {
        x = xi;
        y = yi;
        z = zi;
    }

    inline void Vector::Set(pgd_floating_point_type xi, pgd_floating_point_type yi, pgd_floating_point_type zi)
    {
        x = xi;
        y = yi;
        z = zi;
    }

    inline  pgd_floating_point_type Vector::Magnitude(void)
    {
        return (pgd_floating_point_type) sqrt(x*x + y*y + z*z);
    }

    inline  pgd_floating_point_type Vector::Magnitude2(void)
    {
        return (pgd_floating_point_type) (x*x + y*y + z*z);
    }

    inline  void  Vector::Normalize(void)
    {
        // wis - to cope with very small vectors (quite common) we need to divide by the largest magnitude element
        // to minimise rounding errors. This will make it less good with larger vectors but that's
        // much less common in this application

        pgd_floating_point_type xx = MYMATH_ABS(x);
        pgd_floating_point_type yy = MYMATH_ABS(y);
        pgd_floating_point_type zz = MYMATH_ABS(z);
        pgd_floating_point_type m;
        if (yy >= xx)
        {
            if (zz >= yy) m = zz;
            else m = yy;
        }
        else
        {
            if (zz >= xx) m = zz;
            else m = xx;
        }
        if (m <= minPositive) // too small, need to fix up
        {
            x = 1;
            y = 0;
            z = 0;
            return;
        }

        // divide by maximum element to get all numbers to a sensible size for squaring
        x /= m;
        y /= m;
        z /= m;

        // now do the standard normalisation calculation
        m = (pgd_floating_point_type) sqrt(x*x + y*y + z*z);
        x /= m;
        y /= m;
        z /= m;
    }

    inline  void  Vector::Reverse(void)
    {
        x = -x;
        y = -y;
        z = -z;
    }

    inline Vector& Vector::operator+=(Vector u)
    {
        x += u.x;
        y += u.y;
        z += u.z;
        return *this;
    }

    inline  Vector& Vector::operator-=(Vector u)
    {
        x -= u.x;
        y -= u.y;
        z -= u.z;
        return *this;
    }

    inline  Vector& Vector::operator*=(pgd_floating_point_type s)
    {
        x *= s;
        y *= s;
        z *= s;
        return *this;
    }

    inline  Vector& Vector::operator/=(pgd_floating_point_type s)
    {
        x /= s;
        y /= s;
        z /= s;
        return *this;
    }

    inline  Vector& Vector::operator=(pgd_floating_point_type *s)
    {
        x = s[0];
        y = s[1];
        z = s[2];
        return *this;
    }

    inline  Vector Vector::operator-(void)
    {
        return Vector(-x, -y, -z);
    }


    inline  Vector operator+(Vector u, Vector v)
    {
        return Vector(u.x + v.x, u.y + v.y, u.z + v.z);
    }

    inline  Vector operator-(Vector u, Vector v)
    {
        return Vector(u.x - v.x, u.y - v.y, u.z - v.z);
    }

    // Vector cross product (u cross v)
    inline  Vector operator^(Vector u, Vector v)
    {
        return Vector(  u.y*v.z - u.z*v.y,
                        -u.x*v.z + u.z*v.x,
                        u.x*v.y - u.y*v.x );
    }

    // Vector dot product
    inline  pgd_floating_point_type operator*(Vector u, Vector v)
    {
        return (u.x*v.x + u.y*v.y + u.z*v.z);
    }

    inline  Vector operator*(pgd_floating_point_type s, Vector u)
    {
        return Vector(u.x*s, u.y*s, u.z*s);
    }

    inline  Vector operator*(Vector u, pgd_floating_point_type s)
    {
        return Vector(u.x*s, u.y*s, u.z*s);
    }

    inline  Vector operator/(Vector u, pgd_floating_point_type s)
    {
        return Vector(u.x/s, u.y/s, u.z/s);
    }

    // triple scalar product (u dot (v cross w))
    inline  pgd_floating_point_type TripleScalarProduct(Vector u, Vector v, Vector w)
    {
        return pgd_floating_point_type(   (u.x * (v.y*w.z - v.z*w.y)) +
                        (u.y * (-v.x*w.z + v.z*w.x)) +
                        (u.z * (v.x*w.y - v.y*w.x)) );
        //return u*(v^w);

    }

    //------------------------------------------------------------------------//
    // Quaternion Class and Quaternion functions
    //------------------------------------------------------------------------//

    class Quaternion {
public:
        pgd_floating_point_type   n;  // number (scalar) part
        Vector  v;  // vector part: v.x, v.y, v.z

        Quaternion(void);
        Quaternion(pgd_floating_point_type e0, pgd_floating_point_type e1, pgd_floating_point_type e2, pgd_floating_point_type e3);

        void Set(pgd_floating_point_type e0, pgd_floating_point_type e1, pgd_floating_point_type e2, pgd_floating_point_type e3);

        pgd_floating_point_type   Magnitude(void);
        Vector  GetVector(void);
        pgd_floating_point_type   GetScalar(void);
        void Normalize();
        Quaternion  operator+=(Quaternion q);
        Quaternion  operator-=(Quaternion q);
        Quaternion operator*=(pgd_floating_point_type s);
        Quaternion operator/=(pgd_floating_point_type s);
        Quaternion  operator~(void) const { return Quaternion(n, -v.x, -v.y, -v.z);}
        Quaternion  operator-(void) const { return Quaternion(-n, -v.x, -v.y, -v.z);}
    };

    inline  Quaternion operator+(Quaternion q1, Quaternion q2);
    inline  Quaternion operator-(Quaternion q1, Quaternion q2);
    inline  Quaternion operator*(Quaternion q1, Quaternion q2);
    inline  Quaternion operator*(Quaternion q, pgd_floating_point_type s);
    inline  Quaternion operator*(pgd_floating_point_type s, Quaternion q);
    inline  Quaternion operator*(Quaternion q, Vector v);
    inline  Quaternion operator*(Vector v, Quaternion q);
    inline  Quaternion operator/(Quaternion q, pgd_floating_point_type s);
    inline  pgd_floating_point_type QGetAngle(Quaternion q);
    inline  Vector QGetAxis(Quaternion q);
    inline  Quaternion QRotate(Quaternion q1, Quaternion q2);
    inline  Vector  QVRotate(Quaternion q, Vector v);
    inline  Quaternion  MakeQFromEulerAngles(pgd_floating_point_type x, pgd_floating_point_type y, pgd_floating_point_type z);
    inline  Vector  MakeEulerAnglesFromQ(Quaternion q);
    inline  Quaternion  MakeQFromAxis(pgd_floating_point_type x, pgd_floating_point_type y, pgd_floating_point_type z, pgd_floating_point_type angle);
    inline Quaternion FindRotation(Quaternion qa, Quaternion qb);
    inline pgd_floating_point_type FindAngle(Quaternion qa, Quaternion qb);
    inline Vector FindAxis(Quaternion qa, Quaternion qb);
    inline Quaternion FindRotation(Vector v1, Vector v2);

    inline  Quaternion::Quaternion(void)
    {
        n = 0;
        v.x = 0;
        v.y = 0;
        v.z = 0;
    }


    inline  Quaternion::Quaternion(pgd_floating_point_type e0, pgd_floating_point_type e1, pgd_floating_point_type e2, pgd_floating_point_type e3)
    {
        n = e0;
        v.x = e1;
        v.y = e2;
        v.z = e3;
    }

    inline  void Quaternion::Set(pgd_floating_point_type e0, pgd_floating_point_type e1, pgd_floating_point_type e2, pgd_floating_point_type e3)
    {
        n = e0;
        v.x = e1;
        v.y = e2;
        v.z = e3;
    }

    inline  pgd_floating_point_type   Quaternion::Magnitude(void)
    {
        return (pgd_floating_point_type) sqrt(n*n + v.x*v.x + v.y*v.y + v.z*v.z);
    }

    inline  Vector  Quaternion::GetVector(void)
    {
        return Vector(v.x, v.y, v.z);
    }

    inline  pgd_floating_point_type   Quaternion::GetScalar(void)
    {
        return n;
    }

    inline  Quaternion  Quaternion::operator+=(Quaternion q)
    {
        n += q.n;
        v.x += q.v.x;
        v.y += q.v.y;
        v.z += q.v.z;
        return *this;
    }

    inline  Quaternion  Quaternion::operator-=(Quaternion q)
    {
        n -= q.n;
        v.x -= q.v.x;
        v.y -= q.v.y;
        v.z -= q.v.z;
        return *this;
    }

    inline  Quaternion Quaternion::operator*=(pgd_floating_point_type s)
    {
        n *= s;
        v.x *= s;
        v.y *= s;
        v.z *= s;
        return *this;
    }

    inline  Quaternion Quaternion::operator/=(pgd_floating_point_type s)
    {
        n /= s;
        v.x /= s;
        v.y /= s;
        v.z /= s;
        return *this;
    }

    inline  Quaternion operator+(Quaternion q1, Quaternion q2)
    {
        return  Quaternion( q1.n + q2.n,
                            q1.v.x + q2.v.x,
                            q1.v.y + q2.v.y,
                            q1.v.z + q2.v.z);
    }

    inline  Quaternion operator-(Quaternion q1, Quaternion q2)
    {
        return  Quaternion( q1.n - q2.n,
                            q1.v.x - q2.v.x,
                            q1.v.y - q2.v.y,
                            q1.v.z - q2.v.z);
    }

    inline  Quaternion operator*(Quaternion q1, Quaternion q2)
    {
        return  Quaternion( q1.n*q2.n - q1.v.x*q2.v.x - q1.v.y*q2.v.y - q1.v.z*q2.v.z,
                            q1.n*q2.v.x + q1.v.x*q2.n + q1.v.y*q2.v.z - q1.v.z*q2.v.y,
                            q1.n*q2.v.y + q1.v.y*q2.n + q1.v.z*q2.v.x - q1.v.x*q2.v.z,
                            q1.n*q2.v.z + q1.v.z*q2.n + q1.v.x*q2.v.y - q1.v.y*q2.v.x);
    }

    inline  Quaternion operator*(Quaternion q, pgd_floating_point_type s)
    {
        return  Quaternion(q.n*s, q.v.x*s, q.v.y*s, q.v.z*s);
    }

    inline  Quaternion operator*(pgd_floating_point_type s, Quaternion q)
    {
        return  Quaternion(q.n*s, q.v.x*s, q.v.y*s, q.v.z*s);
    }

    inline  Quaternion operator*(Quaternion q, Vector v)
    {
        return  Quaternion( -(q.v.x*v.x + q.v.y*v.y + q.v.z*v.z),
                            q.n*v.x + q.v.y*v.z - q.v.z*v.y,
                            q.n*v.y + q.v.z*v.x - q.v.x*v.z,
                            q.n*v.z + q.v.x*v.y - q.v.y*v.x);
    }

    inline  Quaternion operator*(Vector v, Quaternion q)
    {
        return  Quaternion( -(q.v.x*v.x + q.v.y*v.y + q.v.z*v.z),
                            q.n*v.x + q.v.z*v.y - q.v.y*v.z,
                            q.n*v.y + q.v.x*v.z - q.v.z*v.x,
                            q.n*v.z + q.v.y*v.x - q.v.x*v.y);
    }

    inline  Quaternion operator/(Quaternion q, pgd_floating_point_type s)
    {
        return  Quaternion(q.n/s, q.v.x/s, q.v.y/s, q.v.z/s);
    }

    inline  pgd_floating_point_type QGetAngle(Quaternion q)
    {
        if (q.n <= -1) return 0; // 2 * pi
        if (q.n >= 1) return 0; // 2 * 0
        return  (pgd_floating_point_type) (2*acos(q.n));
    }

    inline  Vector QGetAxis(Quaternion q)
    {
        Vector v = q.GetVector();
        v.Normalize();
        return v;
    }

    inline  Quaternion QRotate(Quaternion q1, Quaternion q2)
    {
        return  q1*q2*(~q1);
    }

    inline  Vector  QVRotate(Quaternion q, Vector v)
    {
#ifdef EASY_TO_READ
        Quaternion t;


        t = q*v*(~q);

        return  t.GetVector();
#else
    // optimisation based on OpenSG code
    pgd_floating_point_type rx,ry,rz;
    pgd_floating_point_type QwQx, QwQy, QwQz, QxQy, QxQz, QyQz;

    QwQx = q.n * q.v.x;
    QwQy = q.n * q.v.y;
    QwQz = q.n * q.v.z;
    QxQy = q.v.x * q.v.y;
    QxQz = q.v.x * q.v.z;
    QyQz = q.v.y * q.v.z;

    rx = 2* (v.y * (-QwQz + QxQy) + v.z *( QwQy + QxQz));
    ry = 2* (v.x * ( QwQz + QxQy) + v.z *(-QwQx + QyQz));
    rz = 2* (v.x * (-QwQy + QxQz) + v.y *( QwQx + QyQz));

    pgd_floating_point_type QwQw, QxQx, QyQy, QzQz;

    QwQw = q.n * q.n;
    QxQx = q.v.x * q.v.x;
    QyQy = q.v.y * q.v.y;
    QzQz = q.v.z * q.v.z;

    rx+= v.x * (QwQw + QxQx - QyQy - QzQz);
    ry+= v.y * (QwQw - QxQx + QyQy - QzQz);
    rz+= v.z * (QwQw - QxQx - QyQy + QzQz);

    return Vector(rx,ry,rz);

#endif
    }

    // these are intrinsic Euler XYZ angles (or fixed axis ZYX)
    inline  Quaternion  MakeQFromEulerAngles(pgd_floating_point_type x, pgd_floating_point_type y, pgd_floating_point_type z)
    {
        Quaternion  q;
        pgd_floating_point_type   roll = DegreesToRadians(x);
        pgd_floating_point_type   pitch = DegreesToRadians(y);
        pgd_floating_point_type   yaw = DegreesToRadians(z);

        pgd_floating_point_type   cyaw, cpitch, croll, syaw, spitch, sroll;
        pgd_floating_point_type   cyawcpitch, syawspitch, cyawspitch, syawcpitch;

        cyaw = cos(0.5 * yaw);
        cpitch = cos(0.5 * pitch);
        croll = cos(0.5 * roll);
        syaw = sin(0.5 * yaw);
        spitch = sin(0.5 * pitch);
        sroll = sin(0.5 * roll);

        cyawcpitch = cyaw*cpitch;
        syawspitch = syaw*spitch;
        cyawspitch = cyaw*spitch;
        syawcpitch = syaw*cpitch;

        q.n = (pgd_floating_point_type) (cyawcpitch * croll + syawspitch * sroll);
        q.v.x = (pgd_floating_point_type) (cyawcpitch * sroll - syawspitch * croll);
        q.v.y = (pgd_floating_point_type) (cyawspitch * croll + syawcpitch * sroll);
        q.v.z = (pgd_floating_point_type) (syawcpitch * croll - cyawspitch * sroll);

        return q;
    }

    // these are intrinsic Euler XYZ angles (or fixed axis ZYX)
    inline  Vector  MakeEulerAnglesFromQ(Quaternion q)
    {
        pgd_floating_point_type   r11, r21, r31, r32, r33, r12, r13;
        pgd_floating_point_type   q00, q11, q22, q33;
        pgd_floating_point_type   tmp;
        Vector  u;

        q00 = q.n * q.n;
        q11 = q.v.x * q.v.x;
        q22 = q.v.y * q.v.y;
        q33 = q.v.z * q.v.z;

        r11 = q00 + q11 - q22 - q33;
        r21 = 2 * (q.v.x*q.v.y + q.n*q.v.z);
        r31 = 2 * (q.v.x*q.v.z - q.n*q.v.y);
        r32 = 2 * (q.v.y*q.v.z + q.n*q.v.x);
        r33 = q00 - q11 - q22 + q33;

        tmp = MYMATH_ABS(r31);
        if(tmp > (1 - epsilon))
        {
            r12 = 2 * (q.v.x*q.v.y - q.n*q.v.z);
            r13 = 2 * (q.v.x*q.v.z + q.n*q.v.y);

            u.x = RadiansToDegrees(0.0); //roll
            u.y = RadiansToDegrees((pgd_floating_point_type) (-(pi/2) * r31/tmp)); // pitch
            u.z = RadiansToDegrees((pgd_floating_point_type) atan2(-r12, -r31*r13)); // yaw
            return u;
        }

        u.x = RadiansToDegrees((pgd_floating_point_type) atan2(r32, r33)); // roll
        u.y = RadiansToDegrees((pgd_floating_point_type) asin(-r31));      // pitch
        u.z = RadiansToDegrees((pgd_floating_point_type) atan2(r21, r11)); // yaw
        return u;


    }

    // wis  - new routine to make a Quaternion from an axis and a rotation angle in radians
    inline  Quaternion  MakeQFromAxis(pgd_floating_point_type x, pgd_floating_point_type y, pgd_floating_point_type z, pgd_floating_point_type angle)
    {
        Quaternion  q;

        Vector v(x, y, z);
        v.Normalize();

        while (angle > M_PI) angle -= (2 * M_PI);
        while (angle < -M_PI) angle += (2 * M_PI);

        pgd_floating_point_type sin_a = sin( angle / 2 );
        pgd_floating_point_type cos_a = cos( angle / 2 );

        q.v.x    = v.x * sin_a;
        q.v.y    = v.y * sin_a;
        q.v.z    = v.z * sin_a;
        q.n    = cos_a;

        return q;
    }

    // wis - new routines to calculate the quaternion which rotates qa to qb
    //
    // based on http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/slerp/index.htm
    // if q is the quaternion which rotates from qa to qb then:
    // qb = qa * q
    //
    // multiplying both sides by conj(qa) gives:
    // q = conj(qa) * qb
    //
    // The real part of a multiplication is:
    // real((qa.w + i qa.x + j qa.y + k qa.z)*(qb.w + i qb.x + j qb.y + k qb.z)) = qa.w * qb.w - qa.x*qb.x - qa.y*qb.y- qa.z*qb.z
    //
    // So using the conjugate of qa gives:
    // real((qa.w - i qa.x - j qa.y - k qa.z)*(qb.w + i qb.x + j qb.y + k qb.z)) = qa.w*qb.w + qa.x*qb.x + qa.y*qb.y+ qa.z*qb.z
    //
    // real(q) = cos(t/2)
    //
    // Therefore
    // cos(theta/2) = qa.w*qb.w + qa.x*qb.x + qa.y*qb.y+ qa.z*qb.z
    inline Quaternion FindRotation(Quaternion qa, Quaternion qb)
    {
        return ((~qa) * qb);
    }
    inline pgd_floating_point_type FindAngle(Quaternion qa, Quaternion qb)
    {
        pgd_floating_point_type v = qa.n*qb.n + qa.v.x*qb.v.x + qa.v.y*qb.v.y+ qa.v.z*qb.v.z;
        if (v <= -1) return 0; // 2 * pi
        if (v >= 1) return 0; // 2 * 0
        pgd_floating_point_type angle = 2 * acos(v);
        if (angle < -pi) angle += (2 * pi);
        else if (angle > pi) angle -= (2 * pi);
        return angle;
    }
    inline Vector FindAxis(Quaternion qa, Quaternion qb)
    {
        return QGetAxis((~qa) * qb);
    }

    // this is useful when reading in quaternions since there is inevitable precision loss
    // which means the quaternion will be slightly denormal
    inline void Quaternion::Normalize()
    {
        pgd_floating_point_type l = sqrt(n*n + v.x*v.x + v.y*v.y + v.z*v.z);
        n /= l;
        v.x /= l;
        v.y /= l;
        v.z /= l;
    }

    // this routine returns the quaternion that rotates v1 to v2 via the shortest path
    inline Quaternion FindRotation(Vector v1, Vector v2)
    {
        Quaternion q;
        q.n = sqrt((v1.Magnitude2()) * (v2.Magnitude2())) + (v1 * v2);
        if (q.n < epsilon) // this only occurs if a 180 degree rotation is needed
        {
            Vector perp; // this is a perpendicular vector (v1 dot perp = 0)
            if (fabs(v1.z) > epsilon) perp = Vector(0, -v1.z, v1.y);
            else perp = Vector(-v1.y, v1.x, 0);
            q = MakeQFromAxis(perp.x, perp.y, perp.z, M_PI);
        }
        else
        {
            q.v = v1 ^ v2;
            q.Normalize();
        }
        return q;
    }


    //------------------------------------------------------------------------//
    // Matrix Class and matrix functions
    //------------------------------------------------------------------------//

    class Matrix3x3 {
public:
        // elements eij: i -> row, j -> column
        pgd_floating_point_type   e11, e12, e13, e21, e22, e23, e31, e32, e33;

        Matrix3x3(void);
        Matrix3x3(  pgd_floating_point_type r1c1, pgd_floating_point_type r1c2, pgd_floating_point_type r1c3,
                    pgd_floating_point_type r2c1, pgd_floating_point_type r2c2, pgd_floating_point_type r2c3,
                    pgd_floating_point_type r3c1, pgd_floating_point_type r3c2, pgd_floating_point_type r3c3 );
        Matrix3x3(Quaternion q);

        void Set(   pgd_floating_point_type r1c1, pgd_floating_point_type r1c2, pgd_floating_point_type r1c3,
                    pgd_floating_point_type r2c1, pgd_floating_point_type r2c2, pgd_floating_point_type r2c3,
                    pgd_floating_point_type r3c1, pgd_floating_point_type r3c2, pgd_floating_point_type r3c3 );

        pgd_floating_point_type   det(void);
        Matrix3x3   Transpose(void);
        Matrix3x3   Inverse(void);

        Matrix3x3& operator+=(Matrix3x3 m);
        Matrix3x3& operator-=(Matrix3x3 m);
        Matrix3x3& operator*=(pgd_floating_point_type s);
        Matrix3x3& operator/=(pgd_floating_point_type s);
    };

    inline  Matrix3x3 operator+(Matrix3x3 m1, Matrix3x3 m2);
    inline  Matrix3x3 operator-(Matrix3x3 m1, Matrix3x3 m2);
    inline  Matrix3x3 operator/(Matrix3x3 m, pgd_floating_point_type s);
    inline  Matrix3x3 operator*(Matrix3x3 m1, Matrix3x3 m2);
    inline  Matrix3x3 operator*(Matrix3x3 m, pgd_floating_point_type s);
    inline  Matrix3x3 operator*(pgd_floating_point_type s, Matrix3x3 m);
    inline  Vector operator*(Matrix3x3 m, Vector u);
    inline  Vector operator*(Vector u, Matrix3x3 m);





    inline  Matrix3x3::Matrix3x3(void)
    {
        e11 = 0;
        e12 = 0;
        e13 = 0;
        e21 = 0;
        e22 = 0;
        e23 = 0;
        e31 = 0;
        e32 = 0;
        e33 = 0;
    }

    inline  Matrix3x3::Matrix3x3(   pgd_floating_point_type r1c1, pgd_floating_point_type r1c2, pgd_floating_point_type r1c3,
                                    pgd_floating_point_type r2c1, pgd_floating_point_type r2c2, pgd_floating_point_type r2c3,
                                    pgd_floating_point_type r3c1, pgd_floating_point_type r3c2, pgd_floating_point_type r3c3 )
    {
        e11 = r1c1;
        e12 = r1c2;
        e13 = r1c3;
        e21 = r2c1;
        e22 = r2c2;
        e23 = r2c3;
        e31 = r3c1;
        e32 = r3c2;
        e33 = r3c3;
    }

    inline  void Matrix3x3::Set( pgd_floating_point_type r1c1, pgd_floating_point_type r1c2, pgd_floating_point_type r1c3,
                                 pgd_floating_point_type r2c1, pgd_floating_point_type r2c2, pgd_floating_point_type r2c3,
                                 pgd_floating_point_type r3c1, pgd_floating_point_type r3c2, pgd_floating_point_type r3c3 )
    {
        e11 = r1c1;
        e12 = r1c2;
        e13 = r1c3;
        e21 = r2c1;
        e22 = r2c2;
        e23 = r2c3;
        e31 = r3c1;
        e32 = r3c2;
        e33 = r3c3;
    }

    // initialise a matrix from a quaternion
    // based on code from
    // http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm
    // untested!
    inline  Matrix3x3::Matrix3x3(Quaternion q)
    {
        pgd_floating_point_type sqw = q.n*q.n;
        pgd_floating_point_type sqx = q.v.x*q.v.x;
        pgd_floating_point_type sqy = q.v.y*q.v.y;
        pgd_floating_point_type sqz = q.v.z*q.v.z;

        // invs (inverse square length) is only required if quaternion is not already normalised
        pgd_floating_point_type invs = 1 / (sqx + sqy + sqz + sqw);
        e11 = ( sqx - sqy - sqz + sqw)*invs ; // since sqw + sqx + sqy + sqz =1/invs*invs
        e22 = (-sqx + sqy - sqz + sqw)*invs ;
        e33 = (-sqx - sqy + sqz + sqw)*invs ;

        pgd_floating_point_type tmp1 = q.v.x*q.v.y;
        pgd_floating_point_type tmp2 = q.v.z*q.n;
        e21 = 2.0 * (tmp1 + tmp2)*invs ;
        e12 = 2.0 * (tmp1 - tmp2)*invs ;

        tmp1 = q.v.x*q.v.z;
        tmp2 = q.v.y*q.n;
        e31 = 2.0 * (tmp1 - tmp2)*invs ;
        e13 = 2.0 * (tmp1 + tmp2)*invs ;
        tmp1 = q.v.y*q.v.z;
        tmp2 = q.v.x*q.n;
        e32 = 2.0 * (tmp1 + tmp2)*invs ;
        e23 = 2.0 * (tmp1 - tmp2)*invs ;
    }

    inline  pgd_floating_point_type   Matrix3x3::det(void)
    {
        return  e11*e22*e33 -
        e11*e32*e23 +
        e21*e32*e13 -
        e21*e12*e33 +
        e31*e12*e23 -
        e31*e22*e13;
    }

    inline  Matrix3x3   Matrix3x3::Transpose(void)
    {
        return Matrix3x3(e11,e21,e31,e12,e22,e32,e13,e23,e33);
    }

    inline  Matrix3x3   Matrix3x3::Inverse(void)
    {
        pgd_floating_point_type   d = e11*e22*e33 -
        e11*e32*e23 +
        e21*e32*e13 -
        e21*e12*e33 +
        e31*e12*e23 -
        e31*e22*e13;

        if (d == 0) d = 1;

        return  Matrix3x3(  (e22*e33-e23*e32)/d,
                            -(e12*e33-e13*e32)/d,
                            (e12*e23-e13*e22)/d,
                            -(e21*e33-e23*e31)/d,
                            (e11*e33-e13*e31)/d,
                            -(e11*e23-e13*e21)/d,
                            (e21*e32-e22*e31)/d,
                            -(e11*e32-e12*e31)/d,
                            (e11*e22-e12*e21)/d );
    }

    inline  Matrix3x3& Matrix3x3::operator+=(Matrix3x3 m)
    {
        e11 += m.e11;
        e12 += m.e12;
        e13 += m.e13;
        e21 += m.e21;
        e22 += m.e22;
        e23 += m.e23;
        e31 += m.e31;
        e32 += m.e32;
        e33 += m.e33;
        return *this;
    }

    inline  Matrix3x3& Matrix3x3::operator-=(Matrix3x3 m)
    {
        e11 -= m.e11;
        e12 -= m.e12;
        e13 -= m.e13;
        e21 -= m.e21;
        e22 -= m.e22;
        e23 -= m.e23;
        e31 -= m.e31;
        e32 -= m.e32;
        e33 -= m.e33;
        return *this;
    }

    inline  Matrix3x3& Matrix3x3::operator*=(pgd_floating_point_type s)
    {
        e11 *= s;
        e12 *= s;
        e13 *= s;
        e21 *= s;
        e22 *= s;
        e23 *= s;
        e31 *= s;
        e32 *= s;
        e33 *= s;
        return *this;
    }

    inline  Matrix3x3& Matrix3x3::operator/=(pgd_floating_point_type s)
    {
        e11 /= s;
        e12 /= s;
        e13 /= s;
        e21 /= s;
        e22 /= s;
        e23 /= s;
        e31 /= s;
        e32 /= s;
        e33 /= s;
        return *this;
    }

    inline  Matrix3x3 operator+(Matrix3x3 m1, Matrix3x3 m2)
    {
        return  Matrix3x3(  m1.e11+m2.e11,
                            m1.e12+m2.e12,
                            m1.e13+m2.e13,
                            m1.e21+m2.e21,
                            m1.e22+m2.e22,
                            m1.e23+m2.e23,
                            m1.e31+m2.e31,
                            m1.e32+m2.e32,
                            m1.e33+m2.e33);
    }

    inline  Matrix3x3 operator-(Matrix3x3 m1, Matrix3x3 m2)
    {
        return  Matrix3x3(  m1.e11-m2.e11,
                            m1.e12-m2.e12,
                            m1.e13-m2.e13,
                            m1.e21-m2.e21,
                            m1.e22-m2.e22,
                            m1.e23-m2.e23,
                            m1.e31-m2.e31,
                            m1.e32-m2.e32,
                            m1.e33-m2.e33);
    }

    inline  Matrix3x3 operator/(Matrix3x3 m, pgd_floating_point_type s)
    {
        return  Matrix3x3(  m.e11/s,
                            m.e12/s,
                            m.e13/s,
                            m.e21/s,
                            m.e22/s,
                            m.e23/s,
                            m.e31/s,
                            m.e32/s,
                            m.e33/s);
    }

    inline  Matrix3x3 operator*(Matrix3x3 m1, Matrix3x3 m2)
    {
        return Matrix3x3(   m1.e11*m2.e11 + m1.e12*m2.e21 + m1.e13*m2.e31,
                            m1.e11*m2.e12 + m1.e12*m2.e22 + m1.e13*m2.e32,
                            m1.e11*m2.e13 + m1.e12*m2.e23 + m1.e13*m2.e33,
                            m1.e21*m2.e11 + m1.e22*m2.e21 + m1.e23*m2.e31,
                            m1.e21*m2.e12 + m1.e22*m2.e22 + m1.e23*m2.e32,
                            m1.e21*m2.e13 + m1.e22*m2.e23 + m1.e23*m2.e33,
                            m1.e31*m2.e11 + m1.e32*m2.e21 + m1.e33*m2.e31,
                            m1.e31*m2.e12 + m1.e32*m2.e22 + m1.e33*m2.e32,
                            m1.e31*m2.e13 + m1.e32*m2.e23 + m1.e33*m2.e33 );
    }

    inline  Matrix3x3 operator*(Matrix3x3 m, pgd_floating_point_type s)
    {
        return  Matrix3x3(  m.e11*s,
                            m.e12*s,
                            m.e13*s,
                            m.e21*s,
                            m.e22*s,
                            m.e23*s,
                            m.e31*s,
                            m.e32*s,
                            m.e33*s);
    }

    inline  Matrix3x3 operator*(pgd_floating_point_type s, Matrix3x3 m)
    {
        return  Matrix3x3(  m.e11*s,
                            m.e12*s,
                            m.e13*s,
                            m.e21*s,
                            m.e22*s,
                            m.e23*s,
                            m.e31*s,
                            m.e32*s,
                            m.e33*s);
    }

    inline  Vector operator*(Matrix3x3 m, Vector u)
    {
        return Vector(  m.e11*u.x + m.e12*u.y + m.e13*u.z,
                        m.e21*u.x + m.e22*u.y + m.e23*u.z,
                        m.e31*u.x + m.e32*u.y + m.e33*u.z);
    }

    inline  Vector operator*(Vector u, Matrix3x3 m)
    {
        return Vector(  u.x*m.e11 + u.y*m.e21 + u.z*m.e31,
                        u.x*m.e12 + u.y*m.e22 + u.z*m.e32,
                        u.x*m.e13 + u.y*m.e23 + u.z*m.e33);
    }

    inline Matrix3x3 MakeMFromQ(Quaternion q)
    {
        Matrix3x3 m;

        double qq1 = 2*q.v.x*q.v.x;
        double qq2 = 2*q.v.y*q.v.y;
        double qq3 = 2*q.v.z*q.v.z;
        m.e11 = 1 - qq2 - qq3;
        m.e12 = 2*(q.v.x*q.v.y - q.n*q.v.z);
        m.e13 = 2*(q.v.x*q.v.z + q.n*q.v.y);
        m.e21 = 2*(q.v.x*q.v.y + q.n*q.v.z);
        m.e22 = 1 - qq1 - qq3;
        m.e23 = 2*(q.v.y*q.v.z - q.n*q.v.x);
        m.e31 = 2*(q.v.x*q.v.z - q.n*q.v.y);
        m.e32 = 2*(q.v.y*q.v.z + q.n*q.v.x);
        m.e33 = 1 - qq1 - qq2;
        return m;
    }

    inline Quaternion MakeQfromM (Matrix3x3 R)
    {
        Quaternion q;
        double tr,s;
        tr = R.e11 + R.e22 + R.e33;
        if (tr >= 0)
        {
            s = sqrt (tr + 1);
            q.n = 0.5 * s;
            s = 0.5 * (1.0/s);
            q.v.x = (R.e32 - R.e23) * s;
            q.v.y = (R.e13 - R.e31) * s;
            q.v.z = (R.e21 - R.e12) * s;
        }
        else
        {
            // find the largest diagonal element and jump to the appropriate case
            if (R.e22 > R.e11)
            {
                if (R.e33 > R.e22) goto case_2;
                goto case_1;
            }
            if (R.e33 > R.e11) goto case_2;
            goto case_0;

case_0:
            s = sqrt((R.e11 - (R.e22 + R.e33)) + 1);
            q.v.x = 0.5 * s;
            s = 0.5 * (1.0/s);
            q.v.y = (R.e12 + R.e21) * s;
            q.v.z = (R.e31 + R.e13) * s;
            q.n = (R.e32 - R.e23) * s;
            return q;

case_1:
            s = sqrt((R.e22 - (R.e33 + R.e11)) + 1);
            q.v.y = 0.5 * s;
            s = 0.5 * (1.0/s);
            q.v.z = (R.e23 + R.e32) * s;
            q.v.x = (R.e12 + R.e21) * s;
            q.n = (R.e13 - R.e31) * s;
            return q;

case_2:
            s = sqrt((R.e33 - (R.e11 + R.e22)) + 1);
            q.v.z = 0.5 * s;
            s = 0.5 * (1.0/s);
            q.v.x = (R.e31 + R.e13) * s;
            q.v.y = (R.e23 + R.e32) * s;
            q.n = (R.e21 - R.e12) * s;
            return q;
        }
        return q;
    }

    // find the closest point to point P on a line defined as origin B and direction M
    // using formulae from www.geometrictools.com
    inline Vector ClosestPoint(Vector P, Vector B, Vector M)
    {
        double t0 = (M * (P - B)) / (M * M);
        Vector Q = B + (t0 * M);
        return Q;
    }

    //  generates a quaternion between two given quaternions in proportion to the variable t
    // if t=0 then qm=qa, if t=1 then qm=qb, if t is between them then qm will interpolate between them
    inline Quaternion slerp(Quaternion v0, Quaternion v1, double t, bool normalise = true)
    {
        if (normalise)
        {
            // Only unit quaternions are valid rotations.
            // Normalize to avoid undefined behavior.
            v0.Normalize();
            v1.Normalize();
        }

        // Compute the cosine of the angle between the two vectors.
        double dot = v0.v * v1.v; // dot product

        const double DOT_THRESHOLD = 0.9995;
        if (dot > DOT_THRESHOLD)
        {
            // If the inputs are too close for comfort, linearly interpolate
            // and normalize the result.

            Quaternion result = v0 + t * (v1 - v0);
            result.Normalize();
            return result;
        }

        // If the dot product is negative, the quaternions
        // have opposite handed-ness and slerp won't take
        // the shorter path. Fix by reversing one quaternion.
        if (dot < 0.0)
        {
            v1 = -v1;
            dot = -dot;
        }

        MYMATH_CLAMP(dot, -1, 1);    // Robustness: Stay within domain of acos()
        double theta_0 = acos(dot);  // theta_0 = angle between input vectors
        double theta = theta_0*t;    // theta = angle between v0 and result

        Quaternion v2 = v1 - v0*dot;
        v2.Normalize();              // { v0, v2 } is now an orthonormal basis

        return v0*cos(theta) + v2*sin(theta);
    }

}



#endif
