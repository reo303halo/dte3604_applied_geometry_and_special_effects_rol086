#ifndef BLENDING_SPLINE_CURVE_IMPLEMENT_POINT_FOUR_H
#define BLENDING_SPLINE_CURVE_IMPLEMENT_POINT_FOUR_H


#include "../../gmlib/modules/parametrics/gmpcurve.h"
#include "parametrics/curves/gmpsubcurve.h"


namespace GMlib {


template <typename T>
class BlendSplineCurve : public PCurve<T,3> {
    GM_SCENEOBJECT(BlendSplineCurve)
public:
    BlendSplineCurve(PCurve<T,3>* g, int n, bool animate = false);
    BlendSplineCurve( const BlendSplineCurve<T>& copy);
    virtual ~BlendSplineCurve();

    // From PCurve
    bool                isClosed() const override;

    // Public local functions
    int                 findI(T t) const;
    T                   blend(T w) const;
    T                   getW(int d, int i, T t) const;
    Vector<T, 1>        B(int i, T t) const;
    void                makeKnots(T min, T max);
    void                insertLocal(PCurve<T, 3>* c);

    // Implement point 5: Use affine transformations of local curves to make a dynamic visual special effect.
    void            rotateControlCurveAlongCenter(int i, float angle);
    void            translateControlCurve(int i, Vector<T, 3> direction);

protected:
    void                eval(T t, int d, bool l) const override;
    T                   getStartP() const override;
    T                   getEndP()   const override;

    // Protected data for the curve
    int                   _d, _k, _n; // Where n is number of subcurves
    std::vector<T>        _t; // Knot vector storing parameter values
    DVector<PCurve<T,3>*> _c; // Vector of subcurves
    bool                  _closed, _animate;
    Vector<T, 3>          _center; // Central position of the curve


    // Local function
    void                  localSimulate(double dt) override;

}; // END class BlendSplineCurve



// Constructor
template <typename T>
inline
    BlendSplineCurve<T>::BlendSplineCurve( PCurve<T, 3>* g, int n, bool animate)
    : PCurve<T,3>(0, 0, 0), _n(n), _k(_d + 1), _d(1), _animate(animate) {

    _closed = g->isClosed();

    if (isClosed())
        _n++;

    makeKnots(g->getParStart(), g->getParEnd());

    _c.setDim(_n);
    for (int i = 0; i < n; i++) {
        _c[i] = new PSubCurve<T>(g, _t[i], _t[i+2], _t[i+1]);
        _c[i]->setNumber(T(i));
        insertLocal(_c[i]); // Inserts the local (control) curves visually.
    }

    if (isClosed())
        _c[n] = _c[0];

    _center = this->getPos();
}



// Copy
template <typename T>
inline
    BlendSplineCurve<T>::BlendSplineCurve( const BlendSplineCurve<T>& copy ) : PCurve<T,3>(copy) {
}



// Destructor
template <typename T>
BlendSplineCurve<T>::~BlendSplineCurve() {}



template <typename T>
inline
    bool BlendSplineCurve<T>::isClosed() const {
    return _closed;
}



template <typename T>
void BlendSplineCurve<T>::eval( T t, int d, bool /*l*/ ) const {
    this->_p.setDim( d + 1 );

    if (t > _t[_n]) { // Handling speecial case -if t exceeds the last knot value _t[_n], it blends the last two subcurves using the blending function B(i, t).
        Vector<T, 1> B_t = B(_n, t);
        DVector<Vector<T, 3>> c0 = _c[_n - 1]->evaluateParent(t, d);

        if (std::abs(t - _t[_n]) < 1e-5) { this->_p = c0; return; }
        DVector<Vector<T, 3>> c1 = _c[0]->evaluateParent(t, d);

        this->_p = c1 + (c0 - c1) * B_t[0];

    } else { // Normal case: Finding i and Blending
        int i = findI(t);
        Vector<T, 1> B_t = B(i, t);
        DVector<Vector<T, 3>> c0 = _c[i - 1]->evaluateParent(t, d);

        if (std::abs(t - _t[i]) < 1e-5) { this->_p = c0; return; }
        DVector<Vector<T, 3>> c1 = _c[i]->evaluateParent(t, d);

        this->_p = c1 + (c0 - c1) * B_t[0];
    }
}



template <typename T>
T BlendSplineCurve<T>::getStartP() const {
    return _t[_d];
}



template <typename T>
T BlendSplineCurve<T>::getEndP() const {
    return _t[_n];
}



template <typename T>
inline
    int BlendSplineCurve<T>::findI(T t) const {
    if (t == _t[_n])
        return _n - 1;

    for (int i = _d; i < _n; i++) {
        if (t < _t[i + 1])
            return i;
    }
}



template <typename T>
inline
    T BlendSplineCurve<T>::blend(T w) const {
    // Polynomial function of first order
    return 3*w*w - 2*w*w*w;

    // Rational function of first order
    //return (w*w) / (w*w + (1-w)*(1-w));
}



template <typename T>
inline
    T BlendSplineCurve<T>::getW(int d, int i, T t) const {
    return (t - _t[i]) / (_t[i+d] - _t[i]);
}



template <typename T>
inline
    Vector<T, 1> BlendSplineCurve<T>::B(int i, T t) const {
    return {1 - blend(getW(1, i, t))};
}



// ti = (i - d) (max - min) / n - d
// If closed then loop back to start (first and last knots are the same).
template <typename T>
inline
    void BlendSplineCurve<T>::makeKnots(T min, T max) {
    for (int i = 0; i < _k; i++)
        _t.push_back(min); // Extend start

    for (int i = _k; i < _n; i++)
        _t.push_back((i - _d) * (max - min) / (_n - _d)); // Uniformly spaced knots

    for (int i = 0; i < _k; i++)
        _t.push_back(max); // Extend end

    if (isClosed()) {
        T step = (max - min) / (_n - _d);
        for (int i = 0; i < _d; i++) {
            _t[_d - 1 - i] = _t[_d - i] - step;
            _t[_n + 1 + i] = _t[_n + i] + step;
        }
    }
}



template <typename T>
inline
    void BlendSplineCurve<T>::insertLocal(PCurve<T, 3>* c) {
    c->setLocal(true);
    c->toggleDefaultVisualizer();
    c->sample(30, 0);
    c->setVisible(true);
    c->setCollapsed(true);
    this->insert(c);
}



//***********************************************************
// Implement point 5: Affine transformation                **
//***********************************************************

template <typename T>
void BlendSplineCurve<T>::rotateControlCurveAlongCenter(int i, float angle) {
    Vector<T, 3> vec_to_center = this->getPos() - _c[i]->getPos();
    _c[i]->rotateParent(angle, vec_to_center);
}



template <typename T>
void BlendSplineCurve<T>::translateControlCurve(int i, Vector<T, 3> direction) {
    double velocity = 0.05;
    _c[i]->translateGlobal(direction * -velocity);
}



template <typename T>
void BlendSplineCurve<T>::localSimulate(double dt) {
    if (_animate) {
        double speed = 0.2;

        double minimum_distance = 1.0;
        double maximum_distance = 5.0;

        for (int i = 1; i < _c.getDim(); i++) {
            auto* n = _c[i];
            Vector<T, 3> vec_to_center = this->getPos() - n->getPos();

            double dist = vec_to_center.getLength();
            vec_to_center = vec_to_center.normalize();

            // Calculate movement factor for smooth transition
            double movement_factor = (dist - minimum_distance) / (maximum_distance - minimum_distance);
            movement_factor = clamp(movement_factor, 0.0, 1.0);

            // Rotate & Translate smoothly
            rotateControlCurveAlongCenter(i, M_2PI * dt * speed);
            translateControlCurve(i, vec_to_center * movement_factor * dt * speed);
        }
        this->resample();
    }
    else {
        this->sample(100, 0);
    }

    this->setEditDone();
}



// Ensures that a value stays within a specified range. It prevents values from going below a minimum or above a maximum.
template <typename T>
constexpr const T& clamp(const T& value, const T& min, const T& max) {
    return (value < min) ? min : (value > max) ? max : value;
}
} // END namepace GMlib

#endif // BLENDING_SPLINE_CURVE_IMPLEMENT_POINT_FOUR_H
