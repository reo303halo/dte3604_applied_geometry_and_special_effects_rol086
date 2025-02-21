#include <iostream>

#include "scenario.h"
//#include "testtorus.h"
#include "rol086/lissajouscurve_implement_point_three.h"
#include "rol086/bspline_implement_point_one.h"
#include "rol086/closed_subdivision_curve_implement_point_two.h"
#include "rol086/blending_spline_curve_implement_point_four.h"


// hidmanager
//#include "hidmanager/defaulthidmanager.h"

// gmlib
#include <scene/light/gmpointlight.h>
#include <scene/sceneobjects/gmpathtrack.h>
#include <scene/sceneobjects/gmpathtrackarrows.h>
#include "parametrics/curves/gmpline.h"
#include "parametrics/surfaces/gmpsphere.h"
#include "parametrics/curves/gmpcircle.h"

// qt
#include <QQuickItem>


template <typename T>
inline
    std::ostream& operator<<(std::ostream& out, const std::vector<T>& v) {
    out << v.size() << std::endl;
    for(uint i=0; i<v.size(); i++) out << " " << v[i];
    out << std::endl;
    return out;
}



void Scenario::initializeScenario() {

    // Insert a light
    GMlib::Point<GLfloat,3> init_light_pos( 2.0, 4.0, 10 );
    GMlib::PointLight *light = new GMlib::PointLight(GMlib::GMcolor::white(), GMlib::GMcolor::white(),
                                                     GMlib::GMcolor::white(), init_light_pos );
    light->setAttenuation(0.8f, 0.002f, 0.0008f);
    this->scene()->insertLight( light, false );

    // Insert Sun
    this->scene()->insertSun();

    // Default camera parameters
    int init_viewport_size = 600;
    GMlib::Point<float,3>  init_cam_pos( 0.0f, 0.0f, 0.0f );
    GMlib::Vector<float,3> init_cam_dir( 0.0f, 0.0f, -1.0f);
    GMlib::Vector<float,3> init_cam_up(  0.0f, 1.0f, 0.0f );

    // Projection cam
    auto proj_rcpair = createRCPair("Projection");
    proj_rcpair.camera->set(init_cam_pos,init_cam_dir,init_cam_up);
    proj_rcpair.camera->setCuttingPlanes( 1.0f, 8000.0f );
    //proj_rcpair.camera->rotateGlobal( GMlib::Angle(-45), GMlib::Vector<float,3>( 1.0f, 0.0f, 0.0f ) );
    proj_rcpair.camera->translateGlobal( GMlib::Vector<float,3>( 0.0f, 0.0f, 30.0f ) );
    scene()->insertCamera( proj_rcpair.camera.get() );
    proj_rcpair.renderer->reshape( GMlib::Vector<int,2>(init_viewport_size, init_viewport_size) );


    /***************************************************************************
   *                                                                         *
   * Standar example, including path track and path track arrows             *
   *                                                                         *
   ***************************************************************************/

    /*
  GMlib::Material mm(GMlib::GMmaterial::polishedBronze());
  mm.set(45.0);

  auto ptom = new TestTorus(1.0f, 0.4f, 0.6f);
  ptom->toggleDefaultVisualizer();
  ptom->sample(60,60,1,1);
  this->scene()->insert(ptom);
  auto ptrack = new GMlib::PathTrack();
  ptrack->setLineWidth(2);
  ptom->insert(ptrack);
  auto ptrack2 = new GMlib::PathTrackArrows();
  ptrack2->setArrowLength(2);
  ptom->insert(ptrack2);
  */


    /**************************************************************************
    *                                                                         *
    * Class Tasks Week 1, implemention point 1, 2 and 3                       *
    * -Roy E Olsen                                                            *
    **************************************************************************/



    // c
    GMlib::DVector<GMlib::Vector<float,3>> c_points(10);
    c_points[0] = GMlib::Vector<float, 3>(0.7, -5.0, 0);
    c_points[1] = GMlib::Vector<float, 3>(0.3, -1.7, 0);
    c_points[2] = GMlib::Vector<float, 3>(5.0, -0.5, 0);
    c_points[3] = GMlib::Vector<float, 3>(8.7, -1.7, 0);
    c_points[4] = GMlib::Vector<float, 3>(8.9, -5.0, 0);
    c_points[5] = GMlib::Vector<float, 3>(6.0, -5.0, 0);
    c_points[6] = GMlib::Vector<float, 3>(5.0, -2.3, 0);
    c_points[7] = GMlib::Vector<float, 3>(1.4, -2.4, 0);
    c_points[8] = GMlib::Vector<float, 3>(1.6, -4.3, 0);
    c_points[9] = GMlib::Vector<float, 3>(4.4, -5.2, 0);



    // Implement point 3: Simple parametric model curve - Lissajous Curve
    auto* curve1 = new GMlib::LissajousCurve<float>(); // Use default values (3, 3, 3, 2)
    curve1->toggleDefaultVisualizer();
    curve1->sample(100);
    //this->scene()->insert(curve1);

    auto* curve2 = new GMlib::LissajousCurve<float>(8, 3, 3, 2);
    curve2->toggleDefaultVisualizer();
    curve2->setColor(GMlib::GMcolor::blue());
    curve2->sample(100);
    //this->scene()->insert(curve2);



    // Implement point 1: Own version of 2nd-degree B-spline curve
    auto bspline1 = new GMlib::BSpline<float>(c_points);
    bspline1-> toggleDefaultVisualizer();
    bspline1-> sample(100,0);
    //this->scene()-> insert(bspline1);
    bspline1->setLineWidth(5);

    for (int i = 0; i < 9; i++)
    {
        auto line = new GMlib::PLine<float>(GMlib::Point<float,3>(c_points[i]),GMlib::Point<float,3>(c_points[i+1]));
        line-> toggleDefaultVisualizer();
        line->setColor(GMlib::GMcolor::blue());
        line-> sample(3,0);
        //this->scene()-> insert(line);

        auto* sphere = new GMlib::PSphere<float>(0.05f);
        sphere->toggleDefaultVisualizer();
        sphere->sample(8, 8, 1, 1);
        sphere->translate(c_points[i]);
        //this->scene()->insert(sphere);
    }



    // Implement point 1: BSpline with Least Square
    GMlib::DVector<GMlib::Vector<float,3>> P(50);
    auto circle = new GMlib::PCircle<float>(10);
    for (int i = 0; i < 50; i++) {
        P[i] = circle->getPosition(circle->getParStart() + i * circle->getParDelta() / 49);
    }

    auto BSplineLeastSquare = new GMlib::BSpline<float> (P, 7);
    BSplineLeastSquare->toggleDefaultVisualizer();
    BSplineLeastSquare->setColor(GMlib::GMcolor::green());
    BSplineLeastSquare->sample(60);
    BSplineLeastSquare->setLineWidth(3);
    //this->scene()->insert(BSplineLeastSquare);



    // Implement point 2: Closed Subdivision Curve by using Lane-Riesenfeld algorithm
    auto* subdiv = new GMlib::LaneRiesenfeldCurve<float>(c_points, 2);
    subdiv->toggleDefaultVisualizer();
    subdiv->sample(10, 2);
    //this->scene()->insert(subdiv);
    subdiv->translateGlobal({0, 7.5, 0});



    /**************************************************************************
    *                                                                         *
    * Class Tasks Week 2, implemention point 4, 5 and 6                       *
    * -Roy E Olsen                                                            *
    **************************************************************************/



    // Implement point 4: Implement your own version of a Blending Spline Curve
    auto* blendingSpline = new GMlib::BlendSplineCurve<float>(curve1, 8);
    blendingSpline->translateGlobal({-7.5, 7.5, 0});
    blendingSpline->toggleDefaultVisualizer();
    blendingSpline->setLineWidth(2);
    blendingSpline->sample(200, 0);
    this->scene()->insert(blendingSpline);


    // Implement point 5:
    auto* blendingSplineWithAnimation = new GMlib::BlendSplineCurve<float>(curve1, 8, true);
    blendingSplineWithAnimation->toggleDefaultVisualizer();
    blendingSplineWithAnimation->setLineWidth(2);
    blendingSplineWithAnimation->sample(200, 0);
    this->scene()->insert(blendingSplineWithAnimation);


}



void Scenario::cleanupScenario() {

}




void Scenario::callDefferedGL() {

    GMlib::Array< const GMlib::SceneObject*> e_obj;
    this->scene()->getEditedObjects(e_obj);

    for(int i=0; i < e_obj.getSize(); i++)
        if(e_obj(i)->isVisible()) e_obj[i]->replot();
}
