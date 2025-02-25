#ifndef BSPLINE_IMPLEMENT_POINT_ONE_H
#define BSPLINE_IMPLEMENT_POINT_ONE_H


#include "../../gmlib/modules/parametrics/gmpcurve.h"

namespace GMlib {


template <typename T>
class BSpline : public PCurve<T,3> {
    GM_SCENEOBJECT(BSpline)
public:
    BSpline( const DVector<Vector<T,3>>& c);
    BSpline( const DVector<Vector<T,3>>& p, int n);
    BSpline( const BSpline<T>& copy );
    virtual ~BSpline();

    bool                isClosed() const override;

protected:
    void                eval(T t, int d, bool l) const override;
    T                   getStartP() const override;
    T                   getEndP()   const override;

    // Protected data for the curve
    int                  _d, _k; // Where _d is polynomial degree and _k is order of the B-spline (k = d + 1)
    std::vector<T>       _t;     // Knot vector
    DVector<Vector<T,3>> _c;     // Control points

    // Local functions
    void        makeKnots(int n, int k);
    int         findI(T t) const;
    T           getW(int d, int i, T t) const;
    Vector<T,3> getB(int i, T t) const;

}; // END class BSpline



// Constructor 1: Use c as control points and generate a knot vector.
template <typename T>
inline
    BSpline<T>::BSpline(const DVector<Vector<T,3>>& c):PCurve<T,3>(20, 0, 0), _d(2), _k(_d + 1), _c(c) {
    makeKnots(c.getDim(), _k);
}



// Constructor 2: Use least squares to make n control points and generate a knot vector.
template <typename T>
inline
    BSpline<T>::BSpline(const DVector<Vector<T,3>>& p, int n):PCurve<T,3>(20, 0, 0), _d(2), _k(_d + 1) {

    _c.setDim(n);
    makeKnots(n, _k);

    // Find control points

    int m = p.getDim();
    DMatrix<T>A(m, n, T(0));
    for (int j = 0; j < m; j++) {
        T t = getParStart() + j * getParDelta() / (m - 1);
        int i = findI(t);

        GMlib:: Vector<T,3> b = getB(i, t);
        A[j][i - 2] = b[0];
        A[j][i - 1] = b[1];
        A[j][i]     = b[2];
    }

    GMlib::DMatrix<T> AT = A;
    AT.transpose();
    GMlib::DMatrix<T> B = AT * A;
    GMlib::DVector<Vector<T,3>> y = AT * p;
    B.invert();

    _c = B * y;
}



// Copy
template <typename T>
inline
    BSpline<T>::BSpline( const BSpline<T>& copy ) : PCurve<T,3>(copy) {
    _d = copy._d;
    _k = copy._k;
    _t = copy._t;
    _c = copy._c;
}



// Destructor
template <typename T>
BSpline<T>::~BSpline() {}



// Open curve
template <typename T>
bool BSpline<T>::isClosed() const {
    return false;
}



template <typename T>
void BSpline<T>::eval( T t, int d, bool /*l*/ ) const {
    this->_p.setDim( d + 1 );

    // Finds the appropriate knot span i.
    int i = findI(t);

    // Computes the B-spline basis values b.
    Vector<T,3> b = getB(i, t);

    // Computes the weighted sum of control points.
    this->_p[0] = b[0]*_c[i-2] + b[1]*_c[i-1] + b[2]*_c[i];
}



template <typename T>
T BSpline<T>::getStartP() const {
    return _t[_d];
}



template <typename T>
T BSpline<T>::getEndP() const {
    return _t[_c.getDim()];
}


// Protected helper functions

// Creates a uniform knot vector with extra knots at the start and end
template <typename T>
void BSpline<T>::makeKnots(int n, int k) {
    for(int i = 0; i < _k; i++)
        _t.push_back(0);
    for(int i = _k; i < _c.getDim(); i++)
        _t.push_back(i-_d);
    for(int i = 0; i < _k; i++)
        _t.push_back(_t[_c.getDim()-1]+1);
}



// Find the knot span index where t belongs
template <typename T>
int BSpline<T>::findI(T t) const {
    for(int i = _d; i < _c.getDim(); i++)  // Loop through the knot vector
        if(t >= _t[i] && t < _t[i+1])      // Check if t is within the interval [t_i, t_(i+1))
            return i;

    return _c.getDim() - 1; // Return the index of the interval
}



// Computes the blending weight for recursive basis function evaluation
template <typename T>
inline
    T BSpline<T>::getW(int d, int i, T t) const {
    return (t - _t[i]) / (_t[i+d] - _t[i]);
}



// Computes the three basis function values for quadratic B-splines
template <typename T>
Vector<T,3> BSpline<T>::getB(int i, T t) const {
    T w1i =   getW(1, i,   t);
    T w2im1 = getW(2, i-1, t);
    T w2i =   getW(2, i,   t);

    Vector<T,3> b;
    b[0] = (1- w1i) * (1 - w2im1);
    b[1] = (1 - w1i) * w2im1 + w1i * (1 - w2i);
    b[2] = w1i * w2i;

    return b;
}
} // END namepace GMlib

#endif // BSPLINE_IMPLEMENT_POINT_ONE_H
