#ifndef BLENDING_SPLINE_SURFACE_IMPLEMENT_POINT_SIX_H
#define BLENDING_SPLINE_SURFACE_IMPLEMENT_POINT_SIX_H

#include "../simplesubsurf.h"
#include <vector>


namespace GMlib {

template <typename T>
class MyBlendingSurf : public PSurf<T,3> {
    GM_SCENEOBJECT(MyBlendingSurf)
public:
    MyBlendingSurf( PSurf<T, 3>* os, int n_u, int n_v );
    MyBlendingSurf( const MyBlendingSurf<T>& copy );
    virtual ~MyBlendingSurf();

    //***************************************
    //****** Virtual public functions  ******
    //***************************************

    // from PSurf
    bool            isClosedU()  const override;
    bool            isClosedV()  const override;
    T               B(T t, int i, const std::vector<T>& _t) const;
    T               W(T t, int i, int d, const std::vector<T>& _t) const;
    T               blend(T w) const;
    void            generateKnots(std::vector<T>& _t, T min, T max, int n, int d, int k, bool closed) const;
    int             findI(T t, std::vector<T> _t, int _n) const;

protected:
    // Virtual function from PSurf that has to be implemented locally
    void            eval(T u, T v, int d1 = 1, int d2 = 1, bool lu = true, bool lv = true ) const override;
    T               getStartPU() const override;
    T               getEndPU()   const override;
    T               getStartPV() const override;
    T               getEndPV()   const override;
    void            insertLocal(PSurf<T, 3>* c);
    void            localSimulate(double dt) override;

    // Help function to ensure consistent initialization
    virtual void  init();

    // Protected data for the surface
    PSurf<T, 3>*                    _os;
    DMatrix<GMlib::PSurf<T, 3>*>    _c;
    std::vector<T>                  _u;
    std::vector<T>                  _v;
    int                             _n_u;
    int                             _n_v;
    int                             _d;
    int                             _k;

}; // END class MyBlendingSurf

template <typename T>
inline
    MyBlendingSurf<T>::MyBlendingSurf( PSurf<T, 3>* os, int n_u, int n_v)
    :   _os(os), _n_u(n_u), _n_v(n_v), _d(1), _k(2)
{
    this->_dm = GM_DERIVATION_DD;

    if(isClosedU())
        _n_u++;
    if(isClosedV())
        _n_v++;

    generateKnots(_u, os->getParStartU(), os->getParEndU(), _n_u, _d, _k, os->isClosedU());
    generateKnots(_v, os->getParStartV(), os->getParEndV(), _n_v, _d, _k, os->isClosedV());


    _c.setDim(_n_u, _n_v);
    for(int i = 0; i < n_u; i++){
        for(int j = 0; j < n_v; j++) {
            _c[i][j] = new PSimpleSubSurf<T>(_os, _u[i], _u[i+2], _u[i+1], _v[j], _v[j+2], _v[j+1]);
            insertLocal(_c[i][j]);
        }
    }

    if(isClosedU()){
        for(int i = 0; i < n_u; i++)
            _c[i][n_v] = _c[i][0];
    }

    if(isClosedV()){
        for(int i = 0; i < n_v; i++)
            _c[n_u][i] = _c[0][i];
    }

    if(isClosedU() && isClosedV())
        _c[n_u][n_v] = _c[0][0];
}

template <typename T>
inline
    MyBlendingSurf<T>::MyBlendingSurf( const MyBlendingSurf<T>& copy ) : PSurf<T, 3> (copy){

}

template <typename T>
inline
    MyBlendingSurf<T>::~MyBlendingSurf() {}

template <typename T>
inline
    void MyBlendingSurf<T>::init() {

}

template <typename T>
inline
    void MyBlendingSurf<T>::insertLocal(PSurf<T, 3>* c) {
    //    c->setLocal(true);
    c->toggleDefaultVisualizer();
    c->sample(10, 10, 1, 1);
    c->setVisible(true);
    c->setCollapsed(true);
    this->insert(c);
}

template <typename T>
inline
    void MyBlendingSurf<T>::generateKnots(std::vector<T>& _t, T min, T max, int n, int d, int k, bool closed) const {
    for(int i = 0; i < k; i++)
        _t.push_back(min);

    for(int i = k; i < n; i++)
        _t.push_back((i - d)*(max - min)/(n - d));

    for(int i = 0; i < k; i++)
        _t.push_back(max);

    if(closed){
        T step = (max-min) / (n - d);
        for(int i = 0; i < d; i++){
            _t[d - 1 - i] = _t[d - i] - step;
            _t[n + 1 + i] = _t[n + i] + step;
        }
    }
}

template <typename T>
inline
    T MyBlendingSurf<T>::getStartPU() const {
    return _u[_d];
}

template <typename T>
inline
    T MyBlendingSurf<T>::getEndPU() const {
    return _u[_n_u];
}

template <typename T>
inline
    T MyBlendingSurf<T>::getStartPV() const {
    return _v[_d];
}

template <typename T>
inline
    T MyBlendingSurf<T>::getEndPV() const {
    return _v[_n_v];
}

template <typename T>
inline
    bool MyBlendingSurf<T>::isClosedU() const {
    return _os->isClosedU();
}

template <typename T>
inline
    bool MyBlendingSurf<T>::isClosedV() const {
    return _os->isClosedV();
}

template <typename T>
inline
    T MyBlendingSurf<T>::W(T t, int i, int d, const std::vector<T>& _t) const {
    return (t - _t[i]) / (_t[i+d] - _t[i]);
}

template <typename T>
inline
    T MyBlendingSurf<T>::B(T t, int i, const std::vector<T>& _t) const {
    T a = (1 - blend(W(t, i, 1, _t)));

    return {a};
}

template <typename T>
inline
    T MyBlendingSurf<T>::blend(T w) const {
    return 6*w*w*w*w*w - 15*w*w*w*w + 10*w*w*w;
}

template <typename T>
inline
    int MyBlendingSurf<T>::findI(T t, std::vector<T> _t, int _n) const {
    if(t == _t[_n])
        return _n - 1;

    for(int i = _d; i < _n; i++){
        if(t < _t[i + 1])
            return i;
    }
}


template <typename T>
inline
    void MyBlendingSurf<T>::eval(T u, T v, int d1, int d2, bool /*lu*/, bool /*lv*/) const {
    this->_p.setDim( d1 + 1, d2 + 1 );

    int i_u = findI(u, _u, _n_u);
    int i_v = findI(v, _v, _n_v);
    T B_u = B(u, i_u, _u);
    T B_v = B(v, i_v, _v);

    DVector<T> left(2);
    DVector<T> right(2);
    left[1] = B_u;
    left[0] = T(1) - left[1];
    right[1] = B_v;
    right[0] = T(1) - right[1];

    DMatrix<DMatrix<Vector<T,3>>> mat(2, 2);
    mat[1][1] = _c(i_u - 1)(i_v - 1)->evaluateParent(u, v, d1, d2);
    mat[1][0] = _c(i_u - 1)(i_v)->evaluateParent(u, v, d1, d2);
    mat[0][1] = _c(i_u)(i_v - 1)->evaluateParent(u, v, d1, d2);
    mat[0][0] = _c(i_u)(i_v)->evaluateParent(u, v, d1, d2);

    DVector<DMatrix<Vector<T,3>>> vec(2);
    vec[0] = (left[0] * mat[0][0] + left[1] * mat[1][0]) * right[0];
    vec[1] = (left[0] * mat[0][1] + left[1] * mat[1][1]) * right[1];

    this->_p = vec[0] + vec[1];


    //    if(std::abs(u - _u[_n]) < 1e-5) {this->_p = {0}; return;}

    //    DVector<Vector<T, 3>> c1 = _c[0]->evaluateParent(t, d);

    //    this->_p = c1 + (c0 - c1)*B_t[0];

}

template <typename T>
void MyBlendingSurf<T>::localSimulate(double dt) {
    this->sample(20, 30, 1, 1);
    this->setEditDone();
}

} // END namespace GMlib





#endif // BLENDING_SPLINE_SURFACE_IMPLEMENT_POINT_SIX_H
