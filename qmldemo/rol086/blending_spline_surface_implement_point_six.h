#ifndef BLENDING_SPLINE_SURFACE_IMPLEMENT_POINT_SIX_H
#define BLENDING_SPLINE_SURFACE_IMPLEMENT_POINT_SIX_H

#include "../simplesubsurf.h"


namespace GMlib {

template <typename T>
class BlendingSplineSurface : public PSurf<T,3> {
    GM_SCENEOBJECT(BlendingSplineSurface)
public:
    BlendingSplineSurface( PSurf<T, 3>* model_curve, int n_u, int n_v );
    BlendingSplineSurface( const BlendingSplineSurface<T>& copy );
    virtual ~BlendingSplineSurface();

    //***************************************
    //****** Virtual public functions  ******
    //***************************************

    // from PSurf
    bool            isClosedU() const override;
    bool            isClosedV() const override;

protected:
    // Virtual function from PSurf that has to be implemented locally
    void            eval(T u, T v, int d1, int d2, bool lu = true, bool lv = true ) const override;
    T               getStartPU() const override;
    T               getEndPU()   const override;
    T               getStartPV() const override;
    T               getEndPV()   const override;


    // Help function to ensure consistent initialization
    virtual void    init();

    // Protected data for the surface
    PSurf<T, 3>*            _model_curve;
    int                     _n_u, _n_v;




}; // END class BlendingSplineSurface






} // END namespace GMlib


#endif // BLENDING_SPLINE_SURFACE_IMPLEMENT_POINT_SIX_H
