#ifndef CLOSED_SUBDIVISION_CURVE_IMPLEMENT_POINT_TWO_H
#define CLOSED_SUBDIVISION_CURVE_IMPLEMENT_POINT_TWO_H

#include "../../gmlib/modules/parametrics/gmpcurve.h"

namespace GMlib {


template <typename T>
class LaneRiesenfeldCurve : public PCurve<T,3> {
    GM_SCENEOBJECT(LaneRiesenfeldCurve)
public:
    LaneRiesenfeldCurve( const DVector<Vector<T,3>>& p, int d, bool closed = true);
    LaneRiesenfeldCurve( const LaneRiesenfeldCurve<T>& copy );
    virtual ~LaneRiesenfeldCurve();

    bool                isClosed() const override;
    void                sample(int m, int d = 0) override;

protected:
    void                eval(T t, int d, bool l) const override;
    T                   getStartP() const override;
    T                   getEndP()   const override;

    // Protected data for the curve
    int                  _d, _k;  // Where _d is polynomial degree and _k is order of the B-spline (k = d + 1)
    DVector<Vector<T,3>> _c;      // Control points
    bool                 _closed; // Default to true

    // Local functions (found on page 112 in "Blending techniques in Cruve and Surface constructions - Arne Lakså")
    void    LaneRiesenfeldOpen(std::vector<DVector<Vector<T, 3>>>& ph, int k, int d)   const;
    void    LaneRiesenfeldClosed(std::vector<DVector<Vector<T, 3>>>& ph, int k, int d) const;
    int     doublePart(std::vector<DVector<Vector<T, 3>>>& ph, int n)                  const;
    void    smoothPartOpen(std::vector<DVector<Vector<T, 3>>>& ph, int& n, int d)      const;
    void    smoothPartClosed(std::vector<DVector<Vector<T, 3>>>& ph, int n, int d)     const;


}; // END class LaneReisenfeldCurve



// Constructor
template <typename T>
inline
    LaneRiesenfeldCurve<T>::LaneRiesenfeldCurve(const DVector<Vector<T,3>>& p, int d, bool closed):PCurve<T,3>(20, 0, 0), _d(d), _k(_d + 1), _c(p), _closed(closed) {

}



// Copy
template <typename T>
inline
    LaneRiesenfeldCurve<T>::LaneRiesenfeldCurve( const LaneRiesenfeldCurve<T>& copy ) : PCurve<T,3>(copy) {
    _d = copy._d;
    _k = copy._k;
    _c = copy._c;
    _closed = copy._closed;
}



// Destructor
template <typename T>
LaneRiesenfeldCurve<T>::~LaneRiesenfeldCurve() {}



// Not in use
template <typename T>
void LaneRiesenfeldCurve<T>::eval( T t, int d, bool /*l*/ ) const {}



// Not in use
template <typename T>
T LaneRiesenfeldCurve<T>::getStartP() const {
    return 0;
}



// Not in use
template <typename T>
T LaneRiesenfeldCurve<T>::getEndP() const {
    return 0;
}



template <typename T>
bool LaneRiesenfeldCurve<T>::isClosed() const {
    return _closed;
}



template <typename T>
void LaneRiesenfeldCurve<T>::sample(int m, int d) {
    _visu.resize(1);

    LaneRiesenfeldClosed(_visu[0].sample_val, m, d);

    computeSurroundingSphere(_visu[0].sample_val, _visu[0].sur_sphere);

    this->setEditDone();
}



template <typename T>
inline
    void LaneRiesenfeldCurve<T>::LaneRiesenfeldOpen(std::vector<DVector<Vector<T, 3>>>& ph, int k, int d) const {

}



template <typename T>
inline
    void LaneRiesenfeldCurve<T>::LaneRiesenfeldClosed(std::vector<DVector<Vector<T, 3>>>& ph, int k, int d) const {
    int n = _c.getDim();
    int m = pow(2, k) * n + 1;
    ph.resize(m);

    for(int i = 0; i < n; i++) {
        ph[i][0] = _c[i];
    }

    ph[n++][0] = _c[0];

    for(int i = 0; i < k; i++){
        n = doublePart(ph, n);
        smoothPartClosed(ph, n, d);
    }
}



template <typename T>
inline
    int LaneRiesenfeldCurve<T>::doublePart(std::vector<DVector<Vector<T, 3>>>& ph, int n) const {
    for(int i = n - 1; i > 0; i--) {
        ph[2 * i][0] = ph[i][0];
        ph[2 * i - 1][0] = 0.5 * (ph[i][0] + ph[i - 1][0]);
    }
    return 2 * n - 1;
}



template <typename T>
inline
    void LaneRiesenfeldCurve<T>::smoothPartOpen(std::vector<DVector<Vector<T, 3>>>& ph, int& n, int d) const {

}



template <typename T>
inline
    void LaneRiesenfeldCurve<T>::smoothPartClosed(std::vector<DVector<Vector<T, 3>>>& ph, int n, int d) const {
    for (int j = 1; j < d; j++) {
        for (int i = 0; i < n - 1; i++) {
            ph[i][0] = 0.5 * (ph[i][0] + ph[i + 1][0]);
        }
        ph[n - 1][0] = ph[0][0];
    }
}


} // END namepace GMlib

#endif // CLOSED_SUBDIVISION_CURVE_IMPLEMENT_POINT_TWO_H
