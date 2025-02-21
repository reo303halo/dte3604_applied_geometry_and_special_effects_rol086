#ifndef LISSAJOUSCURVE_IMPLEMENT_POINT_THREE_H
#define LISSAJOUSCURVE_IMPLEMENT_POINT_THREE_H

#include "../../gmlib/modules/parametrics/gmpcurve.h"


namespace GMlib {


template <typename T>
class LissajousCurve : public PCurve<T,3> {
    GM_SCENEOBJECT(LissajousCurve)
public:
    LissajousCurve( T _a = 3, T _b = 3, T _kx = 3, T _ky = 2);
    LissajousCurve( const LissajousCurve<T>& copy );
    virtual ~LissajousCurve();

    bool                isClosed() const override;

protected:
    void                eval(T t, int d, bool l) const override;
    T                   getStartP() const override;
    T                   getEndP()   const override;

    // Protected data for the curve
    T _a, _b, _kx, _ky;

}; // END class LissajousCurve



template <typename T>
inline
    LissajousCurve<T>::LissajousCurve( T a, T b, T kx, T ky ) : PCurve<T,3>(20, 0, 7), _a(a), _b(b), _kx(kx), _ky(ky) {}



template <typename T>
inline
    LissajousCurve<T>::LissajousCurve( const LissajousCurve<T>& copy ) : PCurve<T,3>(copy) {
    _a = copy._a;
    _b = copy._b;
    _kx = copy._kx;
    _ky = copy._ky;
}



// Destructor
template <typename T>
LissajousCurve<T>::~LissajousCurve() {}



// Closed curve
template <typename T>
bool LissajousCurve<T>::isClosed() const {
    return true;
}



// Lissajous curve:
// x = a cos(kx t)
// y = b sin(ky t)
template <typename T>
void LissajousCurve<T>::eval( T t, int d, bool /*l*/ ) const {

    this->_p.setDim( d + 1 );

    this->_p[0][0] = _a * cos(_kx * t);
    this->_p[0][1] = _b * sin(_ky * t);
    this->_p[0][2] = T(0);
}



// Start = 0
template <typename T>
T LissajousCurve<T>::getStartP() const {
    return T(0);
}



//End = 2pi
template <typename T>
T LissajousCurve<T>::getEndP() const {
    return T(M_2PI);
}
} // END namepace GMlib

#endif // LISSAJOUSCURVE_IMPLEMENT_POINT_THREE_H
