#include <iostream>

#include "scenario.h"
//#include "testtorus.h"
#include "rol086/lissajouscurve_implement_point_three.h"
#include "rol086/bspline_implement_point_one.h"
#include "rol086/closed_subdivision_curve_implement_point_two.h"
#include "rol086/blending_spline_curve_implement_point_four.h"
#include "rol086/blending_spline_surface_implement_point_six.h"


// hidmanager
//#include "hidmanager/defaulthidmanager.h"

// gmlib
#include <parametrics/surfaces/gmpplane.h>
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



    // Plotting
    bool implement_point_1_a = false; // BSpline Constructor 1
    bool implement_point_1_b = false; // BSpline Constructor 2: Least Square
    bool implement_point_2 = false; // Closed Subdivision Curve: Lane Riesenfeld
    bool implement_point_3 = false; // Model Curve: Lissajous Curve
    bool implement_point_4 = false; // Blending Spline Curve
    bool implement_point_5 = true; // Blending Spline Curve with transformation and rotation
    bool implement_point_6 = false; // Blending Surface



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
    curve1->setColor(GMlib::GMcolor::green());
    curve1->sample(100);

    // By changing parameter "a" from value 3 to 7 we see that the curve stretches on x-axis
    auto* curve2 = new GMlib::LissajousCurve<float>(7, 3, 3, 2);
    curve2->toggleDefaultVisualizer();
    curve2->setColor(GMlib::GMcolor::blue());
    curve2->sample(100);
    curve2->translateGlobal({10.5, 0, 0});

    if (implement_point_3 == true) {
        this->scene()->insert(curve1);
        this->scene()->insert(curve2);
    }



    // Implement point 1: Own version of 2nd-degree B-spline curve
    auto bspline1 = new GMlib::BSpline<float>(c_points);
    bspline1-> toggleDefaultVisualizer();
    bspline1-> sample(100,0);
    if (implement_point_1_a == true) {
        this->scene()-> insert(bspline1);
    }
    bspline1->setLineWidth(5);

    for (int i = 0; i < 9; i++)
    {
        auto line = new GMlib::PLine<float>(GMlib::Point<float,3>(c_points[i]),GMlib::Point<float,3>(c_points[i+1]));
        line-> toggleDefaultVisualizer();
        line->setColor(GMlib::GMcolor::blue());
        line-> sample(3,0);

        auto* sphere = new GMlib::PSphere<float>(0.05f);
        sphere->toggleDefaultVisualizer();
        sphere->sample(8, 8, 1, 1);
        sphere->translate(c_points[i]);

        if (implement_point_1_a) {
            this->scene()-> insert(line);
            this->scene()->insert(sphere);
        }
    }



    // Implement point 1: BSpline with Least Square
    int number_of_points = 50;
    GMlib::DVector<GMlib::Vector<float,3>> P(50);
    auto circle = new GMlib::PCircle<float>(10);
    for (int i = 0; i < 50; i++) {
        P[i] = circle->getPosition(circle->getParStart() + i * circle->getParDelta() / 49);
    }

    auto BSplineLeastSquare = new GMlib::BSpline<float> (P, 3);
    BSplineLeastSquare->toggleDefaultVisualizer();
    BSplineLeastSquare->setColor(GMlib::GMcolor::green());
    BSplineLeastSquare->sample(60);
    BSplineLeastSquare->setLineWidth(3);

    if (implement_point_1_b) {
        this->scene()->insert(BSplineLeastSquare);
    }



    // Implement point 2: Closed Subdivision Curve by using Lane-Riesenfeld algorithm
    // Second degree
    auto* subdiv = new GMlib::LaneRiesenfeldCurve<float>(c_points, 2);
    subdiv->toggleDefaultVisualizer();
    subdiv->sample(10, 2);

    // Third degree
    auto* subdiv_third_degree = new GMlib::LaneRiesenfeldCurve<float>(c_points, 2);
    subdiv_third_degree->toggleDefaultVisualizer();
    subdiv_third_degree->sample(10, 3);

    // Fourth degree
    auto* subdiv_forth_degree = new GMlib::LaneRiesenfeldCurve<float>(c_points, 2);
    subdiv_forth_degree->toggleDefaultVisualizer();
    subdiv_forth_degree->sample(10, 4);

    if (implement_point_2) {
        this->scene()->insert(subdiv);
        this->scene()->insert(subdiv_third_degree);
        this->scene()->insert(subdiv_forth_degree);
    }

    subdiv->translateGlobal({0, 7.5, 0});
    subdiv_third_degree->translateGlobal({10, 7.5, 0});
    subdiv_forth_degree->translateGlobal({20, 7.5, 0});


    /**************************************************************************
    *                                                                         *
    * Class Tasks Week 2, implemention point 4, 5 and 6                       *
    * -Roy E Olsen                                                            *
    **************************************************************************/



    // Implement point 4: Implement your own version of a Blending Spline Curve
    auto* blendingSpline = new GMlib::BlendSplineCurve<float>(curve1, 8);
    blendingSpline->translateGlobal({-7.5, 0, 0});
    blendingSpline->toggleDefaultVisualizer();
    blendingSpline->setLineWidth(2);
    blendingSpline->sample(200, 0);

    if (implement_point_4) {
        this->scene()->insert(blendingSpline);
    }



    // Implement point 5:
    auto* blendingSplineWithAnimation = new GMlib::BlendSplineCurve<float>(curve1, 8, true);
    blendingSplineWithAnimation->toggleDefaultVisualizer();
    blendingSplineWithAnimation->setLineWidth(2);
    blendingSplineWithAnimation->sample(200, 0);

    if (implement_point_5) {
        this->scene()->insert(blendingSplineWithAnimation);
    }


    // Implement point 6:
    GMlib::Point<float, 3> p(1, 0, 0);
    GMlib::Vector<float, 3> u(0, 5, 0);
    GMlib::Vector<float, 3> v(0, 0, 5);

    //Draw blending plane
    GMlib::PSurf<float, 3>* plane = new GMlib::PPlane<float>(p, u, v);
    auto* plane_surf = new GMlib::MyBlendingSurf<float>(plane, 3, 3);
    plane_surf->translateGlobal({2.5, -10, 0});
    plane_surf->rotateGlobal(-M_PI/2, {0, 1, 0});
    plane_surf->toggleDefaultVisualizer();
    plane_surf->sample(20, 20, 2, 2);

    if (implement_point_6) {
        this->scene()->insert(plane_surf);
    }
}



void Scenario::cleanupScenario() {}



void Scenario::callDefferedGL() {
    GMlib::Array< const GMlib::SceneObject*> e_obj;
    this->scene()->getEditedObjects(e_obj);

    for(int i=0; i < e_obj.getSize(); i++)
        if(e_obj(i)->isVisible()) e_obj[i]->replot();
}
