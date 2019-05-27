#include <iostream>
#include <string>
#include <algorithm>
#include <cmath>
#include "m3.h"
#include "m1.h"
#include "m2.h"
#include "m3.h"
#include "LatLon.h"
#include "StreetsDatabaseAPI.h"
#include "ezgl/application.hpp"
#include "ezgl/graphics.hpp"
#include "ezgl/camera.hpp"
#include "math.h"
#include "OSMDatabaseAPI.h"
#include <ezgl/point.hpp>
#include "ezgl/global.h"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include "m1.cpp"
#include <ezgl/helpers.h>
#include "class.h"

//prototype all functions
void initial_setup(ezgl::application *application);
void find_button(ezgl::application *application);
void load_maps (ezgl::application *application);
void draw_street_names(ezgl::renderer &g, ezgl::rectangle &view);
void act_on_mouse_click(ezgl::application* app, GdkEventButton* event, double x, double y);
void act_on_mouse_move(ezgl::application *application, GdkEventButton *event, double x, double y);
void draw_poi (ezgl::renderer &g, ezgl::rectangle &view);
void draw_intersections(ezgl::renderer &g,ezgl::rectangle &view);
void draw_streets(ezgl::renderer &g,ezgl::rectangle &view);
void draw_features(ezgl::renderer &g, ezgl::rectangle &view);
void draw_POIbox(ezgl::renderer &g, ezgl::rectangle &view);
void draw_main_canvas (ezgl::renderer &g);
void directions (ezgl::application *application); 
void draw_path(ezgl::renderer &g, const std::vector<unsigned>& path);
int find_intersection_id(std::string street1, std::string street2);
bool check;
float find_angle_from_horizontal(float x1, float y1, float x2, float y2);
ezgl::point2d find_midpoint(ezgl::point2d point1, ezgl::point2d point2); 
double distance_between_xy_points(ezgl::point2d point1, ezgl::point2d point2); 
ezgl::point2d find_point(ezgl::point2d point1, ezgl::point2d point2); 
float angle_for_one_way(float x1, float y1, float x2, float y2);
std::vector<unsigned> route;
//global data structures
struct intersection_data {
    LatLon position;
    std::string name;
    bool highlight = false;                 //check whether to highlight and display information of an intersection
    bool select = false;                    //check whether an intersection has been selected for shortest path
};
struct poi_data {
    LatLon position;
    std::string name;
    bool highlight = false;
};

std::vector<intersection_data> intersections;
std::vector<poi_data> poi;

//global variables
double g_width;
double g_height;
std::string xs, ys;
ezgl::rectangle current_view({0,0},{0,0});
int num_selected = 0; //the number of intersections selected for path drawing (can be 0, 1, or 2)
unsigned id1; 
unsigned id2;
bool clicked_no_where = true;

float x_from_lon(float lon){
    float x;
    x = ((lon)*cos(latavg))*DEG_TO_RAD;
    return x;
}
float y_from_lat(float lat){
    return lat*DEG_TO_RAD;
}

float lon_from_x(float x){
    float lon;
    lon = x/(cos(latavg)*DEG_TO_RAD);
    return lon;
}
float lat_from_y(float y){
    return y/DEG_TO_RAD;

}
float find_angle_from_horizontal(float x1, float y1, float x2, float y2) {
    float del_x = x2-x1;
    float del_y = y2-y1;

    if (del_x < 0) {
        del_x = -del_x;
        del_y = -del_y;
    }

    float angle = atan2(del_y, del_x)/DEG_TO_RAD;

    return angle;
}

namespace bg = boost::geometry;

void load_streets_to_draw(){
    segments.resize(getNumStreetSegments());
    for (int i = 0; i < getNumStreetSegments(); i++){ 
        segments[i].highlight = false;
        std::string value, key;                                                     //initialize OSM data
        OSMID osmid = getInfoStreetSegment(i).wayOSMID;
        auto find = street_types.find(osmid);
        for (int j = 0; j < getTagCount(getWayByIndex(find->second)); j++){
            key = getTagPair(getWayByIndex(find->second), j).first;
            value = getTagPair(getWayByIndex(find->second), j).second; 
            if (key == "highway") {
                segments[i].key = key;
                segments[i].value = value;
                break;                                //already found key and value, no need to keep looking
            }
        }       
        segments[i].oneway = getInfoStreetSegment(i).oneWay;     //check if current seg is oneway
        
        segments[i].start_point = {x_from_lon(getIntersectionPosition((getInfoStreetSegment(i).from)).lon()),    //initialize start and end points of the segmen t
            y_from_lat(getIntersectionPosition((getInfoStreetSegment(i).from)).lat())};
        segments[i].end_point = {x_from_lon(getIntersectionPosition((getInfoStreetSegment(i).to)).lon()), 
            y_from_lat(getIntersectionPosition((getInfoStreetSegment(i).to)).lat())};
        
        segments[i].curvepoint_count = getInfoStreetSegment(i).curvePointCount;  //check number of curve-points
        
        if (segments[i].curvepoint_count == 0){                                                        //if segment is straight, the following is found based on the start 
            segments[i].mid_point = find_midpoint(segments[i].start_point, segments[i].end_point);          //and end point
            segments[i].seg_length = distance_between_xy_points(segments[i].start_point, segments[i].end_point);
            segments[i].text_rotation = find_angle_from_horizontal(segments[i].start_point.x, segments[i].start_point.y, segments[i].end_point.x, segments[i].end_point.y);
            if (segments[i].oneway){
                segments[i].arrow_rotation = angle_for_one_way(segments[i].start_point.x, segments[i].start_point.y, segments[i].end_point.x, segments[i].end_point.y);
                segments[i].arrow_point = find_point(segments[i].start_point, segments[i].end_point); 
            }
        }
                
        if (segments[i].curvepoint_count != 0){                                       
            segments[i].curve_points.resize(segments[i].curvepoint_count);
            for (int j = 0; j < segments[i].curvepoint_count; j++){                //initialize each curve point position
                segments[i].curve_points[j].point = {x_from_lon(getStreetSegmentCurvePoint(j, i).lon()), y_from_lat(getStreetSegmentCurvePoint(j, i).lat())};               
            }
            segments[i].mid_point = find_midpoint(segments[i].start_point, segments[i].curve_points[0].point);                              //
            segments[i].seg_length = distance_between_xy_points(segments[i].start_point, segments[i].curve_points[0].point);                //
            segments[i].text_rotation = find_angle_from_horizontal(segments[i].start_point.x, segments[i].start_point.y,                    //      
                                            segments[i].curve_points[0].point.x, segments[i].curve_points[0].point.y);                      //     this is for the first 
            if (segments[i].oneway){                                                                                                        //          curve segment
                segments[i].arrow_rotation = angle_for_one_way(segments[i].start_point.x, segments[i].start_point.y,                        //    
                                            segments[i].curve_points[0].point.x, segments[i].curve_points[0].point.y);                      //
                segments[i].arrow_point = find_point(segments[i].start_point, segments[i].curve_points[0].point);                           //    
            }
            int k;
            for (k = 0; k < (segments[i].curvepoint_count - 1); k++){ 
                segments[i].curve_points[k].mid = find_midpoint(segments[i].curve_points[k].point, segments[i].curve_points[k+1].point);                            //
                segments[i].curve_points[k].length = distance_between_xy_points(segments[i].curve_points[k].point, segments[i].curve_points[k+1].point);            //
                segments[i].curve_points[k].text_rot = find_angle_from_horizontal(segments[i].curve_points[k].point.x, segments[i].curve_points[k].point.y,         //this
                                                                                    segments[i].curve_points[k+1].point.x, segments[i].curve_points[k+1].point.y);  //is
                if(segments[i].oneway){                                                                                                                             //for      
                    segments[i].curve_points[k].arrow_rot = angle_for_one_way(segments[i].curve_points[k].point.x, segments[i].curve_points[k].point.y,             //intermediate
                                                                                    segments[i].curve_points[k+1].point.x, segments[i].curve_points[k+1].point.y);  //curve
                    segments[i].curve_points[k].arrow = find_point(segments[i].curve_points[k].point, segments[i].curve_points[k+1].point);                   //segments
                }               
            }
            segments[i].curve_points[k].mid = find_midpoint(segments[i].curve_points[k].point, segments[i].end_point);                                  //
            segments[i].curve_points[k].length = distance_between_xy_points(segments[i].curve_points[k].point, segments[i].end_point);                  //
            segments[i].curve_points[k].text_rot = find_angle_from_horizontal(segments[i].curve_points[k].point.x, segments[i].curve_points[k].point.y, //  this is for   
                                                                                segments[i].end_point.x, segments[i].end_point.y);                      //  the last
            if (segments[i].oneway){                                                                                                                    //  curve segment
                segments[i].curve_points[k].arrow_rot = angle_for_one_way(segments[i].curve_points[k].point.x, segments[i].curve_points[k].point.y,     //
                                                                                segments[i].end_point.x, segments[i].end_point.y);                      //
                segments[i].curve_points[k].arrow = find_point(segments[i].curve_points[k].point, segments[i].end_point);                         //
            }
        
        }               
    }
}

//#include "streets.cpp"

//check if mouse moves over intersection and highlights it
void act_on_mouse_move(ezgl::application *application, GdkEventButton *event, double x, double y)
{
    double err = 0.0000005;
    LatLon pos = LatLon(lat_from_y(y), lon_from_x(x));

    int id = find_closest_intersection(pos);

    for (unsigned i = 0; i < intersections.size(); ++i) {
        //highlight only if mouses moves within intersection node
        if ((i == id) && (((x >= x_from_lon(getIntersectionPosition(id).lon())-err) && (x <= x_from_lon(getIntersectionPosition(id).lon())+err))
           || ((y >= y_from_lat(getIntersectionPosition(id).lat())-err )&& (y <= y_from_lat(getIntersectionPosition(id).lat())+err))))
        {
            intersections[id].highlight = true;
        } 
        
        else 
        {
            intersections[i].highlight = false;
        }
    }
    
    //convert mouse movements into showing lat and lon of map
    float lat = lat_from_y(y);
    float lon = lon_from_x(x);
    
    std::string xtemp = "LAT: ";
    std::string ytemp = "LON: ";
    
    xs= std::to_string(lat);
    ys= std::to_string(lon);
    
    xs = xtemp +xs;
    ys = ytemp+ys;
    
    //get fixed points to draw lat and lon regardless of canvas changes
    g_width = application->get_canvas(application->get_main_canvas_id())->width()/2;
    g_height = application->get_canvas(application->get_main_canvas_id())->height()-10;
    application->refresh_drawing();

}

void load_street_segment_names() {
    
    for (int i = 0; i < getNumStreetSegments(); i++) 
    {
        street_seg_names.push_back(getStreetName(getInfoStreetSegment(i).streetID));
    }
}

void load_intersections() {

    max_lat = getIntersectionPosition(0).lat();    //initialize min and max latitudes and longitudes
    min_lat = max_lat;
    max_lon = getIntersectionPosition(0).lon();
    min_lon = max_lon;

    intersections.resize(getNumIntersections());
    for (int i = 0; i < getNumIntersections(); i++) {                      //initialize the intersections
        intersections[i].position = getIntersectionPosition(i);
        intersections[i].name = getIntersectionName(i);

        max_lat = std::max(max_lat, intersections[i].position.lat());
        min_lat = std::min(min_lat, intersections[i].position.lat());
        max_lon = std::max(max_lon, intersections[i].position.lon());
        min_lon = std::min(min_lon, intersections[i].position.lon());
    }
    latavg = ((min_lat + max_lat)/2)*DEG_TO_RAD;

}

void load_POI() {
    
    //get all POI data holding id and name
    poi.resize(getNumPointsOfInterest());
    
    for (int i = 0; i<getNumPointsOfInterest(); ++i){
        
        poi[i].position = getPointOfInterestPosition(i);
        poi[i].name = getPointOfInterestName(i);
    }

}

void draw_POIbox(ezgl::renderer &g, ezgl::rectangle &view) {
    // draws the box for poi
    double width = view.width();
    for (unsigned i = 0; i < poi.size(); i++) {                         //draw the intersections
        float x = x_from_lon(poi[i].position.lon());
        float y = y_from_lat(poi[i].position.lat());
        if(view.contains({x, y})) {

            float radius = 0.0000005;                                      //dimensions of the circles to show intersections
            if (radius >= view.width()/100) radius = view.width()/100;
            ezgl::point2d start(x-width/5,y+radius);
            if ((poi[i].highlight)&&(width <= 2.22875e-05)) {

                ezgl::rectangle le (start,width/2,width/12);
                g.set_color(ezgl::BLACK);
                g.fill_rectangle(le);
                g.set_color(ezgl::WHITE);
                g.set_text_rotation(0);
                g.set_font_size(10);
                g.draw_text(le.center(), poi[i].name);
                g.set_color(ezgl::RED);
            }
        }
    }
}
void act_on_mouse_press(ezgl::application *application, GdkEventButton* event, double x, double y) {
    
    double err = 0.0000005*10;
    double err2 = 0.0000005;
    LatLon pos = LatLon(lat_from_y(y), lon_from_x(x));
    
    clicked_no_where = true;

    int id = find_closest_point_of_interest(pos);
    int id_2 = find_closest_intersection(pos);
    //if (num_selected == 2) num_selected = 0;

    for (unsigned i = 0; i < poi.size(); ++i) {
        if ((i == id) && (((x >= x_from_lon(getPointOfInterestPosition(id).lon())-err) && (x <= x_from_lon(getPointOfInterestPosition(id).lon())+err))|| ((y >= y_from_lat(getPointOfInterestPosition(id).lat())-err )&& (y <= y_from_lat(getPointOfInterestPosition(id).lat())+err))))
        {  
            poi[id].highlight = true;
        }
        else 
        {
            poi[i].highlight = false;
        }
    }

    for (unsigned i = 0; i < intersections.size(); i++){
        if ((i == id_2) && (((x >= x_from_lon(getIntersectionPosition(id_2).lon())-err2) && (x <= x_from_lon(getIntersectionPosition(id_2).lon())+err2))
           || ((y >= y_from_lat(getIntersectionPosition(id_2).lat())-err2)&& (y <= y_from_lat(getIntersectionPosition(id_2).lat())+err2)))) {
            
            intersections[id_2].select = true;
            num_selected++;
            clicked_no_where = false;
            
        } 
               
    }
   /* if (clicked_no_where || num_selected == 3){
        num_selected = 0;
        for (unsigned i = 0; i < intersections.size(); i++){            
            intersections[i].select = false;
            id1 = 0;
            id2 = 0;
        }
        std::cout << "id's reset" << std::endl;
    }*/
    //if (clicked_no_where) num_selected = 0;
    
    //if (num_selected == 2) num_selected = 0;
    
    std::cout << "mouse pressed " << num_selected <<std::endl;

    application->refresh_drawing();
}



void draw_map() {
    //load_POI();

    ezgl::application::settings settings;
    settings.main_ui_resource = "libstreetmap/resources/main.ui";
    settings.window_identifier = "MainWindow";
    settings.canvas_identifier = "MainCanvas";

    ezgl::application application(settings);

    ezgl::rectangle initial_world({x_from_lon(min_lon), y_from_lat(min_lat)}, {x_from_lon(max_lon), y_from_lat(max_lat)});

    application.add_canvas("MainCanvas", draw_main_canvas, initial_world);




    application.run(nullptr, act_on_mouse_press, nullptr, nullptr);

}



void draw_main_canvas (ezgl::renderer &g) {
    

    g.set_color(232,232,232,250);
    g.fill_rectangle(g.get_visible_world());
    ezgl::rectangle current_view = g.get_visible_world();
    draw_features(g,current_view);        //helper function for drawing intersections
    draw_streets(g, current_view);                   //helper function for drawing streets
    g.set_color(ezgl::BLACK);
    g.set_font_size(10);
    g.set_coordinate_system(ezgl::SCREEN);
    //g.draw_text({g_width,g_height}, xs,75,65);
    //g.draw_text({g_width-100, g_height}, ys,75,65);
    g.set_coordinate_system(ezgl::WORLD);
    draw_path(g,route);
    
    draw_street_names(g, current_view);               //helper function for drawing street names
    draw_POIbox(g, current_view);
    draw_intersections(g, current_view);                            //helper function for drawing intersections
    draw_poi(g,current_view);
    select_path(g, current_view);


}

void draw_path(ezgl::renderer &g, const std::vector<unsigned>& path){
    g.set_color(ezgl::RED);                                       //line settings for streets
    g.set_line_dash(ezgl::line_dash::none); 
    g.set_line_width(10);
    //std::cout << "the path segments are: " << path.size() <<std::endl;
   
    for (int i = 0; i < path.size(); i++){
        //std::cout << getStreetName(getInfoStreetSegment(path[i]).streetID) << std::endl; 
        //if (i == 0) g.set_color(ezgl::RED); 
        //else g.set_color(ezgl::ORANGE);
       
                segments[path[i]].highlight = true;
          
        
       
        
        /*(if (segments[path[i]].curvepoint_count == 0){  
            //if there are no curve points     
            g.draw_line(segments[path[i]].start_point, segments[path[i]].end_point);             //connect the start and end points of street segments without curve points            
        }
        else{                                                                                                   //if curve points exist                            
            g.draw_line(segments[path[i]].start_point, segments[path[i]].curve_points[0].point);                            
            int count;
            for (count = 1; count < segments[path[i]].curvepoint_count; count++){											//if there are curve points present, connect each point to form the segment                                
                g.draw_line(segments[path[i]].curve_points[count-1].point, segments[path[i]].curve_points[count].point);                                    
            }                            
            g.draw_line(segments[path[i]].curve_points[count-1].point, segments[path[i]].end_point);                
        }*/
    }
    route.clear();
    
}

std::string display_path_directions(std::vector<unsigned>& path){
    std::vector<unsigned>::iterator end = path.end();
    std::vector<unsigned>::iterator it = path.begin();
    TurnType turn;
    std::string directions = "";
    std::string length;
    double length_of_current_street;
    int current_street_id, upcoming_street_id;
    std::cout<< path.size()<<std::endl;
    while (it != end){
        length_of_current_street = 0;                                                                   //reset the length when new street found in path
        //std::cout << "Follow " << getStreetName(getInfoStreetSegment(*it).streetID) << "for ";
        directions = directions + "Follow " + getStreetName(getInfoStreetSegment(*it).streetID) + "for ";
        length_of_current_street+= find_street_segment_length(*it);
        
        while ((it+1) != end){
            if (getInfoStreetSegment(*it).streetID == getInfoStreetSegment(*(it+1)).streetID){
                length_of_current_street+= find_street_segment_length(*(it+1));
                it++;
            }
            else {               
                break;
            }
        }
        //std::cout << length_of_current_street << "m" << std::endl;
        length = std::to_string(length_of_current_street);
        directions = directions + length + "m" +"\n";
        if ((it+1)!= end){
            turn = find_turn_type(*it, *(it+1));
            if (turn == TurnType::LEFT){
                //std::cout << "Turn LEFT onto " << getStreetName(getInfoStreetSegment(*(it+1)).streetID) << std::endl;
                directions = directions + "Turn LEFT onto " + getStreetName(getInfoStreetSegment(*(it+1)).streetID) + "\n";
            //std::cout<<"inside the left for loop: "<< directions<<std::endl;
            }
            else if (turn == TurnType::RIGHT){
                //std::cout << "Turn RIGHT onto " << getStreetName(getInfoStreetSegment(*(it+1)).streetID) << std::endl;
                directions = directions + "Turn RIGHT onto " + getStreetName(getInfoStreetSegment(*(it+1)).streetID) + "\n";
            //std::cout<<"inside the right for loop: "<< directions<<std::endl;
            }
        }
        it++;
    }
    //td::cout << directions << std::endl;
    //std::cout << "You have reached your destination!" << std::endl;
    directions = directions + "You have reached your destination!" + "\n";
    //std::cout << directions;
    return directions;
    
}

void select_path(ezgl::renderer &g, ezgl::rectangle &view){                                //function that displays shortest path between 2 intersections once those
    g.set_color(ezgl::RED);                                                                //two intersections have been clicked
    g.set_line_width(20);
    std::vector<unsigned> path1;
    std::string directions;
    if (clicked_no_where || num_selected == 3){
        num_selected = 0;
        for (unsigned i = 0; i < intersections.size(); i++){            
            intersections[i].select = false;
            id1 = 0;
            id2 = 0;
        }
        //std::cout << "id's reset" << std::endl;
    }
    for (unsigned i = 0; i < intersections.size(); i++){
        if (intersections[i].select && num_selected < 3){
            if (num_selected == 1) id1 = i;
            else if (num_selected == 2 && i != id1) id2 = i;
            float radius = 0.000001;  
            float x = x_from_lon(intersections[i].position.lon());
            float y = y_from_lat(intersections[i].position.lat());
            g.fill_arc({x, y}, radius, 0, 360);
        }
    }
    std::cout << id1 << " " << id2 << std::endl;
    //if (num_selected == 2) g.draw_line({x_from_lon(intersections[id1].position.lon()), y_from_lat(intersections[id1].position.lat())}, 
      //                                       {x_from_lon(intersections[id2].position.lon()), y_from_lat(intersections[id2].position.lat())}); 
    if (id2 != 0 && id1 != 0){
        path1 = find_path_between_intersections(id1, id2, 15, 25);
        //std::cout << "path found" << std::endl;
        draw_path(g, path1);
        directions = display_path_directions(path1);
        std::cout<<directions<<std::endl;
    }
}




void draw_intersections(ezgl::renderer &g, ezgl::rectangle &view) {
    g.set_color(ezgl::BLACK);
    double width = view.width();

    for (unsigned i = 0; i < intersections.size(); i++) {                         //draw the intersections
        float x = x_from_lon(intersections[i].position.lon());
        float y = y_from_lat(intersections[i].position.lat());
        if(view.contains({x, y})) {

            float radius = 0.0000005;                                      //dimensions of the circles to show intersections
            if (radius >= view.width()/100) radius = view.width()/100;
            ezgl::point2d start(x-width/5,y+radius);
            if (intersections[i].highlight){//&&(width <= 0.000220482)) {

                ezgl::rectangle le (start,width/2,width/12);
                g.set_color(ezgl::BLACK);
                g.fill_rectangle(le);
                g.set_color(ezgl::WHITE);
                g.set_text_rotation(0);
                g.set_font_size(10);
                
                //g.draw_text(le.center(), intersections[i].name);
                g.draw_text(le.center(), intersections[i].name);
                g.set_color(ezgl::RED);
            }
            else {

                g.set_color(ezgl::BLACK);
            }
            g.fill_arc({x, y}, radius, 0, 360);
        }

    }

}
bool compare(const feature_data &a, const feature_data &b)
{
    return a.area < b.area;
}
void load_features() {
    //load all features and sorts it based on area
    poly.resize(getNumFeatures());
    int count = poly.size();
    typedef bg::model::point<float, 2, bg::cs::cartesian> point_t;
    typedef bg::model::ring<point_t> ring_t;
    
    //draws polygon based on points and calculates inside area
    for (int a=0; a<count ; ++a) {
        ring_t ring1;
        int count2 = getFeaturePointCount(a);

        for (int b = 0; b<count2 ; ++b) {
            float startx = x_from_lon(getFeaturePoint(0,a).lon());
            float starty = y_from_lat(getFeaturePoint(0,a).lat());
            float endx = x_from_lon(getFeaturePoint(count2-1,a).lon());
            float endy = y_from_lat(getFeaturePoint(count2-1,a).lat());
            if (startx == endx && starty == endy) {
                float start_x = x_from_lon(getFeaturePoint(b,a).lon());
                float start_y = y_from_lat(getFeaturePoint(b,a).lat());
                ezgl::point2d points(start_x, start_y);
                poly[a].points.push_back(points);
                bg::append(ring1, point_t(start_x, start_y));
            }
            else {
                bg::append(ring1, point_t(0.0, 0.0));
                ezgl::point2d points(0.0, 0.0);
                poly[a].points.push_back(points);
            }
        }
        poly[a].area = fabs(bg::area(ring1));
        poly[a].id = a;
    }

    std::sort(poly.rbegin(),poly.rend(),compare);
}

void draw_features(ezgl::renderer &g, ezgl::rectangle &view) {
    
    int count = poly.size();
    double width = view.width();
    double check;

    for (int l=0; l<count ; ++l) {
        
        bool test = false;
        for (int i= 0; i<poly[l].points.size(); ++i) {
            if (view.contains(poly[l].points[i]) && (poly[l].area <= 2.34479e-10)) {
                test = true;
                check = 0.000877698;
            }
            else if(poly[l].area > 2.34479e-10) {
                test = true;
                check = 1000;
            }
            else {
                test = false;
                check=1000;
            }
        }

        int i = poly[l].id;
        int count2 = getFeaturePointCount(poly[l].id);
        if (getFeatureType(i) == Lake && (count2 > 1) && (width <= check) && (test)) {
            g.set_color(170, 218, 255,250);
            g.fill_poly(poly[l].points);
        }

        else if (getFeatureType(i) == Golfcourse && (count2 > 1) && (width <= check) && (test)) {
            g.set_color(195, 236, 178,250);
            g.fill_poly(poly[l].points);
        }

        else if (getFeatureType(i) == Island && (count2 > 1) && (width <= check) && (test)) {
            g.set_color(195, 236, 178,250);
            g.fill_poly(poly[l].points);

        }
        else if (getFeatureType(i) == Greenspace && (count2 > 1) && (width <= check) && (test)) {
            g.set_color(195, 236, 178,250);
            g.fill_poly(poly[l].points);
        }

        else if (getFeatureType(i) == Park && (count2 > 1) && (width <= check) && (test)) {
            g.set_color(195, 236, 178,250);
            g.fill_poly(poly[l].points);
        }
        
        else if (getFeatureType(i) == River && (count2 > 1) && (width <= check) && (test)) {
            g.set_color(170, 218, 255,250);
            float startx = x_from_lon(getFeaturePoint(0,i).lon());
            float starty = y_from_lat(getFeaturePoint(0,i).lat());
            float endx = x_from_lon(getFeaturePoint(count2-1,i).lon());
            float endy = y_from_lat(getFeaturePoint(count2-1,i).lat());
            if (startx == endx && starty == endy) {
                g.fill_poly(poly[l].points);
            }
            else {
                for (int j = 0; j<count2-1 ; ++j) {
                    g.set_line_dash(ezgl::line_dash::none);
                    g.set_line_width(2);
                    g.draw_line(poly[l].points[j], poly[l].points[j+1]);
                }
            }

        }

        else if (getFeatureType(i) == Building && (count2 > 1) && (width <= check) && (test) ) {
            g.set_color(213, 216, 219,250);
            g.fill_poly(poly[l].points);

        }
        else if (getFeatureType(i) == Beach && (count2 > 1) && (width <= check) && (test)) {
            g.set_color(255, 242, 175,250);
            g.fill_poly(poly[l].points);
        }
        
        else if (getFeatureType(i) == Stream && (count2 > 1) ) {
            g.set_color(170, 218, 255,250);
            for (int j = 0; j<count2-1 ; ++j) {
                g.set_line_dash(ezgl::line_dash::none);
                g.set_line_width(2);
                g.draw_line(poly[l].points[j], poly[l].points[j+1]);
            }
        }

        else if (getFeatureType(i) == Unknown && (count2 > 1) && (width <= check) && (test) ) {
            g.set_color(255, 242, 175,250);
            float startx = x_from_lon(getFeaturePoint(0,i).lon());
            float starty = y_from_lat(getFeaturePoint(0,i).lat());
            float endx = x_from_lon(getFeaturePoint(count2-1,i).lon());
            float endy = y_from_lat(getFeaturePoint(count2-1,i).lat());
            for (int j = 0; j<count2-1 ; ++j) {
                g.set_line_dash(ezgl::line_dash::none);
                g.set_line_width(2);
                g.draw_line(poly[l].points[j], poly[l].points[j+1]);
            }

        }
    }
}

std::string auto_correct_find(std::string input) {
    
    std::vector<unsigned> list = find_street_ids_from_partial_street_name(input);
    
    if (list.size() != 0) {                                                                 //if name is a prefix
        return getStreetName(list[0]);
    }
    //if not a prefix or full correct street name, find closest matching one
    std::transform(input.begin(), input.end(), input.begin(), ::tolower);

    int greatest_match_count = 0;
    int index;
    for (unsigned i = 0; i < getNumStreets(); i++) {
        std::string str = getStreetName(i);
        std::transform(str.begin(), str.end(), str.begin(), ::tolower);
        int length_in = input.length();
        int length_str = str.length();
        int current_match_count = 0;

        for (int i = 0, j = 0; (i < length_str) && (j < length_in); i++, j++) {
            if (str[i] == input[j]) current_match_count++;
            else if (str[i] == input[j+1]) i--;
            else if (str[i+1] == input[j]) j--;
        }
        if (current_match_count > greatest_match_count) {
            greatest_match_count = current_match_count;
            index = i;
        }
    }
    return getStreetName(index);
}

float angle_for_one_way(float x1, float y1, float x2, float y2) {
    float del_x = x2-x1;
    float del_y = y2-y1;

    float angle = atan2(del_y, del_x)/DEG_TO_RAD;

    return angle;
}

ezgl::point2d find_midpoint(ezgl::point2d point1, ezgl::point2d point2) {
    ezgl::point2d midpoint = {(point1.x + point2.x)/2, (point2.y + point1.y)/2};
    return midpoint;
}

double distance_between_xy_points(ezgl::point2d point1, ezgl::point2d point2) {
    double distance = sqrt((point2.x - point1.x)*(point2.x - point1.x) + (point2.y - point1.y)*(point2.y - point1.y));
    return distance;
}

ezgl::point2d find_point(ezgl::point2d point1, ezgl::point2d point2) {
    double del_x = (point2.x - point1.x)/40;
    double del_y = (point2.y - point1.y)/40;
    return {point1.x + 3*del_x, point1.y + 3*del_y};
}

void find_button(GtkWidget *widget, ezgl::application *application)
{
    application->update_message("Find Button Pressed");
    application->refresh_drawing();
}


void draw_poi (ezgl::renderer &g,ezgl::rectangle &view) {
    double width = view.width();
    ezgl::rectangle check = g.get_visible_world();
    for(int i = 0; i<getNumPointsOfInterest(); ++i) {

        if (getPointOfInterestType(i) == "atm" &&(width <= 2.22875e-05) ) {
            double start_x = x_from_lon(getPointOfInterestPosition(i).lon());
            double start_y = y_from_lat(getPointOfInterestPosition(i).lat());
            ezgl::point2d p_atm(start_x, start_y);
            if (check.contains(p_atm)) {
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/atm.png");
                g.draw_surface(png_surface, p_atm);
                ezgl::renderer::free_surface(png_surface);
            }
        }
        else if ((getPointOfInterestType(i) == "bank" || getPointOfInterestType(i) == "bureau_de_change" ) &&(width <= 2.22875e-05) ) {

            double start_x = x_from_lon(getPointOfInterestPosition(i).lon());
            double start_y = y_from_lat(getPointOfInterestPosition(i).lat());
            ezgl::point2d p_atm(start_x, start_y);
            if (check.contains(p_atm)) {
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/bank.png");
                g.draw_surface(png_surface, p_atm);
                ezgl::renderer::free_surface(png_surface);
            }
        }
        else if ((getPointOfInterestType(i) == "bar" || getPointOfInterestType(i) == "pub" ) &&(width <= 2.22875e-05) ) {

            double start_x = x_from_lon(getPointOfInterestPosition(i).lon());
            double start_y = y_from_lat(getPointOfInterestPosition(i).lat());
            ezgl::point2d p_atm(start_x, start_y);
            if (check.contains(p_atm)) {
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/bar.png");
                g.draw_surface(png_surface, p_atm);
                ezgl::renderer::free_surface(png_surface);
            }
        }
        else if (getPointOfInterestType(i) == "bicycle_rental" &&(width <= 2.22875e-05) ) {

            double start_x = x_from_lon(getPointOfInterestPosition(i).lon());
            double start_y = y_from_lat(getPointOfInterestPosition(i).lat());
            ezgl::point2d p_atm(start_x, start_y);
            if (check.contains(p_atm)) {
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/bicycle_rental.png");
                g.draw_surface(png_surface, p_atm);
                ezgl::renderer::free_surface(png_surface);
            }
        }
        else if (getPointOfInterestType(i) == "bus_station" &&(width <= 2.22875e-05) ) {

            double start_x = x_from_lon(getPointOfInterestPosition(i).lon());
            double start_y = y_from_lat(getPointOfInterestPosition(i).lat());
            ezgl::point2d p_atm(start_x, start_y);
            if (check.contains(p_atm)) {
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/busstop.png");
                g.draw_surface(png_surface, p_atm);
                ezgl::renderer::free_surface(png_surface);
            }
        }
        else if (getPointOfInterestType(i) == "cafe" &&(width <= 2.22875e-05) ) {

            double start_x = x_from_lon(getPointOfInterestPosition(i).lon());
            double start_y = y_from_lat(getPointOfInterestPosition(i).lat());
            ezgl::point2d p_atm(start_x, start_y);
            if (check.contains(p_atm)) {
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/coffee.png");
                g.draw_surface(png_surface, p_atm);
                ezgl::renderer::free_surface(png_surface);
            }
        }
        else if (getPointOfInterestType(i) == "car_rental" &&(width <= 2.22875e-05) ) {

            double start_x = x_from_lon(getPointOfInterestPosition(i).lon());
            double start_y = y_from_lat(getPointOfInterestPosition(i).lat());
            ezgl::point2d p_atm(start_x, start_y);
            if (check.contains(p_atm)) {
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/carrental.png");
                g.draw_surface(png_surface, p_atm);
                ezgl::renderer::free_surface(png_surface);
            }
        }
        else if (getPointOfInterestType(i) == "car_wash" &&(width <= 2.22875e-05) ) {

            double start_x = x_from_lon(getPointOfInterestPosition(i).lon());
            double start_y = y_from_lat(getPointOfInterestPosition(i).lat());
            ezgl::point2d p_atm(start_x, start_y);
            if (check.contains(p_atm)) {
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/carwash.png");
                g.draw_surface(png_surface, p_atm);
                ezgl::renderer::free_surface(png_surface);
            }
        }
        else if (getPointOfInterestType(i) == "car_sharing" &&(width <= 2.22875e-05) ) {

            double start_x = x_from_lon(getPointOfInterestPosition(i).lon());
            double start_y = y_from_lat(getPointOfInterestPosition(i).lat());
            ezgl::point2d p_atm(start_x, start_y);
            if (check.contains(p_atm)) {
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/car_share.png");
                g.draw_surface(png_surface, p_atm);
                ezgl::renderer::free_surface(png_surface);
            }

        }
        else if (getPointOfInterestType(i) == "childcare" &&(width <= 2.22875e-05) ) {

            double start_x = x_from_lon(getPointOfInterestPosition(i).lon());
            double start_y = y_from_lat(getPointOfInterestPosition(i).lat());
            ezgl::point2d p_atm(start_x, start_y);
            if (check.contains(p_atm)) {
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/childcare.png");
                g.draw_surface(png_surface, p_atm);
                ezgl::renderer::free_surface(png_surface);
            }

        }
        else if (getPointOfInterestType(i) == "cinema" &&(width <= 2.22875e-05) ) {

            double start_x = x_from_lon(getPointOfInterestPosition(i).lon());
            double start_y = y_from_lat(getPointOfInterestPosition(i).lat());
            ezgl::point2d p_atm(start_x, start_y);
            if (check.contains(p_atm)) {
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/cinema.png");
                g.draw_surface(png_surface, p_atm);
                ezgl::renderer::free_surface(png_surface);
            }

        }
        else if (getPointOfInterestType(i) == "clinic" &&(width <= 2.22875e-05) ) {

            double start_x = x_from_lon(getPointOfInterestPosition(i).lon());
            double start_y = y_from_lat(getPointOfInterestPosition(i).lat());
            ezgl::point2d p_atm(start_x, start_y);
            if (check.contains(p_atm)) {
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/clinic.png");
                g.draw_surface(png_surface, p_atm);
                ezgl::renderer::free_surface(png_surface);
            }

        }
        else if (getPointOfInterestType(i) == "college" &&(width <= 2.22875e-05) ) {

            double start_x = x_from_lon(getPointOfInterestPosition(i).lon());
            double start_y = y_from_lat(getPointOfInterestPosition(i).lat());
            ezgl::point2d p_atm(start_x, start_y);
            if (check.contains(p_atm)) {
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/university.png");
                g.draw_surface(png_surface, p_atm);
                ezgl::renderer::free_surface(png_surface);
            }

        }
        else if (getPointOfInterestType(i) == "community_centre" &&(width <= 2.22875e-05) ) {

            double start_x = x_from_lon(getPointOfInterestPosition(i).lon());
            double start_y = y_from_lat(getPointOfInterestPosition(i).lat());
            ezgl::point2d p_atm(start_x, start_y);
            if (check.contains(p_atm)) {
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/communitycentre.png");
                g.draw_surface(png_surface, p_atm);
                ezgl::renderer::free_surface(png_surface);
            }

        }
        else if (getPointOfInterestType(i) == "dentist" &&(width <= 2.22875e-05) ) {

            double start_x = x_from_lon(getPointOfInterestPosition(i).lon());
            double start_y = y_from_lat(getPointOfInterestPosition(i).lat());
            ezgl::point2d p_atm(start_x, start_y);
            if (check.contains(p_atm)) {
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/dentist.png");
                g.draw_surface(png_surface, p_atm);
                ezgl::renderer::free_surface(png_surface);
            }

        }
        else if (getPointOfInterestType(i) == "doctors" &&(width <= 2.22875e-05) ) {

            double start_x = x_from_lon(getPointOfInterestPosition(i).lon());
            double start_y = y_from_lat(getPointOfInterestPosition(i).lat());
            ezgl::point2d p_atm(start_x, start_y);
            if (check.contains(p_atm)) {
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/medicine.png");
                g.draw_surface(png_surface, p_atm);
                ezgl::renderer::free_surface(png_surface);
            }

        }
        else if ((getPointOfInterestType(i) == "fast_food" || getPointOfInterestType(i) == "restaurant" ) &&(width <= 2.22875e-05) ) {

            double start_x = x_from_lon(getPointOfInterestPosition(i).lon());
            double start_y = y_from_lat(getPointOfInterestPosition(i).lat());
            ezgl::point2d p_atm(start_x, start_y);
            if (check.contains(p_atm)) {
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/fastfood.png");
                g.draw_surface(png_surface, p_atm);
                ezgl::renderer::free_surface(png_surface);
            }

        }
        else if ((getPointOfInterestType(i) == "fire_station" ) &&(width <= 2.22875e-05) ) {

            double start_x = x_from_lon(getPointOfInterestPosition(i).lon());
            double start_y = y_from_lat(getPointOfInterestPosition(i).lat());
            ezgl::point2d p_atm(start_x, start_y);
            if (check.contains(p_atm)) {
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/firemen.png");
                g.draw_surface(png_surface, p_atm);
                ezgl::renderer::free_surface(png_surface);
            }

        }
        else if (getPointOfInterestType(i) == "food_court" &&(width <= 2.22875e-05) ) {

            double start_x = x_from_lon(getPointOfInterestPosition(i).lon());
            double start_y = y_from_lat(getPointOfInterestPosition(i).lat());
            ezgl::point2d p_atm(start_x, start_y);
            if (check.contains(p_atm)) {
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/foodcourt.png");
                g.draw_surface(png_surface, p_atm);
                ezgl::renderer::free_surface(png_surface);
            }

        }
        else if (getPointOfInterestType(i) == "fuel" &&(width <= 2.22875e-05) ) {

            double start_x = x_from_lon(getPointOfInterestPosition(i).lon());
            double start_y = y_from_lat(getPointOfInterestPosition(i).lat());
            ezgl::point2d p_atm(start_x, start_y);
            if (check.contains(p_atm)) {
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/fuel.png");
                g.draw_surface(png_surface, p_atm);
                ezgl::renderer::free_surface(png_surface);
            }

        }
        else if (getPointOfInterestType(i) == "hospital" &&(width <= 2.22875e-05) ) {

            double start_x = x_from_lon(getPointOfInterestPosition(i).lon());
            double start_y = y_from_lat(getPointOfInterestPosition(i).lat());
            ezgl::point2d p_atm(start_x, start_y);
            if (check.contains(p_atm)) {
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/hospital-2.png");
                g.draw_surface(png_surface, p_atm);
                ezgl::renderer::free_surface(png_surface);
            }

        }
        else if (getPointOfInterestType(i) == "kindergarten" &&(width <= 2.22875e-05) ) {

            double start_x = x_from_lon(getPointOfInterestPosition(i).lon());
            double start_y = y_from_lat(getPointOfInterestPosition(i).lat());
            ezgl::point2d p_atm(start_x, start_y);
            if (check.contains(p_atm)) {
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/kindergarten.png");
                g.draw_surface(png_surface, p_atm);
                ezgl::renderer::free_surface(png_surface);
            }

        }
        else if (getPointOfInterestType(i) == "lawyer" &&(width <= 2.22875e-05) ) {

            double start_x = x_from_lon(getPointOfInterestPosition(i).lon());
            double start_y = y_from_lat(getPointOfInterestPosition(i).lat());
            ezgl::point2d p_atm(start_x, start_y);
            if (check.contains(p_atm)) {
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/lawyer.png");
                g.draw_surface(png_surface, p_atm);
                ezgl::renderer::free_surface(png_surface);
            }

        }
        else if (getPointOfInterestType(i) == "library" &&(width <= 2.22875e-05) ) {

            double start_x = x_from_lon(getPointOfInterestPosition(i).lon());
            double start_y = y_from_lat(getPointOfInterestPosition(i).lat());
            ezgl::point2d p_atm(start_x, start_y);
            if (check.contains(p_atm)) {
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/library.png");
                g.draw_surface(png_surface, p_atm);
                ezgl::renderer::free_surface(png_surface);
            }

        }
        else if (getPointOfInterestType(i) == "nightclub" &&(width <= 2.22875e-05) ) {

            double start_x = x_from_lon(getPointOfInterestPosition(i).lon());
            double start_y = y_from_lat(getPointOfInterestPosition(i).lat());
            ezgl::point2d p_atm(start_x, start_y);
            if (check.contains(p_atm)) {
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/nightclub.png");
                g.draw_surface(png_surface, p_atm);
                ezgl::renderer::free_surface(png_surface);
            }

        }
        else if (getPointOfInterestType(i) == "parking" &&(width <= 2.22875e-05) ) {

            double start_x = x_from_lon(getPointOfInterestPosition(i).lon());
            double start_y = y_from_lat(getPointOfInterestPosition(i).lat());
            ezgl::point2d p_atm(start_x, start_y);
            if (check.contains(p_atm)) {
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/parking.png");
                g.draw_surface(png_surface, p_atm);
                ezgl::renderer::free_surface(png_surface);
            }

        }
        else if (getPointOfInterestType(i) == "pharmacy" &&(width <= 2.22875e-05) ) {

            double start_x = x_from_lon(getPointOfInterestPosition(i).lon());
            double start_y = y_from_lat(getPointOfInterestPosition(i).lat());
            ezgl::point2d p_atm(start_x, start_y);
            if (check.contains(p_atm)) {
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/pharmacy.png");
                g.draw_surface(png_surface, p_atm);
                ezgl::renderer::free_surface(png_surface);
            }

        }
        else if (getPointOfInterestType(i) == "place_of_worship" &&(width <= 2.22875e-05) ) {

            double start_x = x_from_lon(getPointOfInterestPosition(i).lon());
            double start_y = y_from_lat(getPointOfInterestPosition(i).lat());
            ezgl::point2d p_atm(start_x, start_y);
            if (check.contains(p_atm)) {
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/worship.png");
                g.draw_surface(png_surface, p_atm);
                ezgl::renderer::free_surface(png_surface);
            }

        }
        else if (getPointOfInterestType(i) == "police" &&(width <= 2.22875e-05) ) {

            double start_x = x_from_lon(getPointOfInterestPosition(i).lon());
            double start_y = y_from_lat(getPointOfInterestPosition(i).lat());
            ezgl::point2d p_atm(start_x, start_y);
            if (check.contains(p_atm)) {
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/police2.png");
                g.draw_surface(png_surface, p_atm);
                ezgl::renderer::free_surface(png_surface);
            }

        }
        else if (getPointOfInterestType(i) == "post_office" &&(width <= 2.22875e-05) ) {

            double start_x = x_from_lon(getPointOfInterestPosition(i).lon());
            double start_y = y_from_lat(getPointOfInterestPosition(i).lat());
            ezgl::point2d p_atm(start_x, start_y);
            if (check.contains(p_atm)) {
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/postal.png");
                g.draw_surface(png_surface, p_atm);
                ezgl::renderer::free_surface(png_surface);
            }

        }
        else if (getPointOfInterestType(i) == "public_building" &&(width <= 2.22875e-05) ) {

            double start_x = x_from_lon(getPointOfInterestPosition(i).lon());
            double start_y = y_from_lat(getPointOfInterestPosition(i).lat());
            ezgl::point2d p_atm(start_x, start_y);
            if (check.contains(p_atm)) {
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/public.png");
                g.draw_surface(png_surface, p_atm);
                ezgl::renderer::free_surface(png_surface);
            }

        }
        else if (getPointOfInterestType(i) == "school" &&(width <= 2.22875e-05) ) {

            double start_x = x_from_lon(getPointOfInterestPosition(i).lon());
            double start_y = y_from_lat(getPointOfInterestPosition(i).lat());
            ezgl::point2d p_atm(start_x, start_y);
            if (check.contains(p_atm)) {
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/school.png");
                g.draw_surface(png_surface, p_atm);
                ezgl::renderer::free_surface(png_surface);
            }

        }
        else if (getPointOfInterestType(i) == "spa" &&(width <= 2.22875e-05) ) {

            double start_x = x_from_lon(getPointOfInterestPosition(i).lon());
            double start_y = y_from_lat(getPointOfInterestPosition(i).lat());
            ezgl::point2d p_atm(start_x, start_y);
            if (check.contains(p_atm)) {
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/spa.png");
                g.draw_surface(png_surface, p_atm);
                ezgl::renderer::free_surface(png_surface);
            }

        }
        else if (getPointOfInterestType(i) == "storage" &&(width <= 2.22875e-05) ) {

            double start_x = x_from_lon(getPointOfInterestPosition(i).lon());
            double start_y = y_from_lat(getPointOfInterestPosition(i).lat());
            ezgl::point2d p_atm(start_x, start_y);
            if (check.contains(p_atm)) {
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/storage.png");
                g.draw_surface(png_surface, p_atm);
                ezgl::renderer::free_surface(png_surface);
            }

        }
        else if (getPointOfInterestType(i) == "stripclub" &&(width <= 2.22875e-05) ) {

            double start_x = x_from_lon(getPointOfInterestPosition(i).lon());
            double start_y = y_from_lat(getPointOfInterestPosition(i).lat());
            ezgl::point2d p_atm(start_x, start_y);
            if (check.contains(p_atm)) {
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/stripclub2.png");
                g.draw_surface(png_surface, p_atm);
                ezgl::renderer::free_surface(png_surface);
            }

        }
        else if (getPointOfInterestType(i) == "theatre" &&(width <= 2.22875e-05) ) {

            double start_x = x_from_lon(getPointOfInterestPosition(i).lon());
            double start_y = y_from_lat(getPointOfInterestPosition(i).lat());
            ezgl::point2d p_atm(start_x, start_y);
            if (check.contains(p_atm)) {
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/theater.png");
                g.draw_surface(png_surface, p_atm);
                ezgl::renderer::free_surface(png_surface);
            }

        }
        else if (getPointOfInterestType(i) == "tutoring" &&(width <= 2.22875e-05) ) {

            double start_x = x_from_lon(getPointOfInterestPosition(i).lon());
            double start_y = y_from_lat(getPointOfInterestPosition(i).lat());
            ezgl::point2d p_atm(start_x, start_y);
            if (check.contains(p_atm)) {
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/tutor.png");
                g.draw_surface(png_surface, p_atm);
                ezgl::renderer::free_surface(png_surface);
            }

        }
        else if (getPointOfInterestType(i) == "vending_machine" &&(width <= 2.22875e-05) ) {

            double start_x = x_from_lon(getPointOfInterestPosition(i).lon());
            double start_y = y_from_lat(getPointOfInterestPosition(i).lat());
            ezgl::point2d p_atm(start_x, start_y);
            if (check.contains(p_atm)) {
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/vending.png");
                g.draw_surface(png_surface, p_atm);
                ezgl::renderer::free_surface(png_surface);
            }

        }
        else if (getPointOfInterestType(i) == "veterinary" &&(width <= 2.22875e-05) ) {

            double start_x = x_from_lon(getPointOfInterestPosition(i).lon());
            double start_y = y_from_lat(getPointOfInterestPosition(i).lat());
            ezgl::point2d p_atm(start_x, start_y);
            if (check.contains(p_atm)) {
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/veterinary.png");
                g.draw_surface(png_surface, p_atm);
                ezgl::renderer::free_surface(png_surface);
            }

        }
    }
} 
void find_button(ezgl::application *application)
{
    std::string street2, street1;

    const char* text = gtk_entry_get_text(ezgl::text_entry); //user input from search bar as a single string with two streets
    std::string street3(text);

    int pos = street3.find_first_of("&"); //string of two streets is divided into street 1 and street 2
    street1 = street3.substr(0,pos-1);
    street2 = street3.substr(pos+2,street3.size());

    std::cout << street1 << std::endl;
    std::cout << street2 << std::endl;

    std::string street4 = auto_correct_find(street1); //autocorrect feature that can fix small spelling mistakes in streets
    std::cout << "auto correct leads to: ";
    std::string street5 = auto_correct_find(street2);

    std::vector<unsigned> street_id1 = find_street_ids_from_partial_street_name(street4); //find all street id's from the street or partial street given
    std::vector<unsigned> street_id2 = find_street_ids_from_partial_street_name(street5);

    std::vector<unsigned> intersect;

    if (street_id1.size() == 0) { //vector is unpopulated meaning no street exists
        std::cout<< street1 << " does not exist " << std::endl;
        
    }

    if (street_id2.size() == 0) {
        std::cout << street2 << " does not exist " << std::endl;
    }


    for (int i = 0; i < street_id1.size(); i++) {
        for (int j = 0; j < street_id2.size(); j++) {
            intersect = find_intersection_ids_from_street_ids(street_id1[i], street_id2[j]); //cross reference all street id's found to see what two streets actually intersect
            if (intersect.size() > 0) {
                for (auto id : intersect) {
                    if (!intersections[id].highlight) {
                        intersections[id].highlight = true; //highlight intersection points
                        std::cout << getIntersectionName(id) << std::endl;
                    }
                }
            }
        }
    }

    intersect.clear();

    for (size_t i = 0; i < intersections.size(); i++) {
        if (intersections[i].highlight)
            intersect.push_back(i); //add back all highlighted intersections to vector
    }

    std::cout.flush();

    auto canvas1 = application->get_canvas(application->get_main_canvas_id());


    if (intersect.size()!=0) {
        LatLon point = getIntersectionPosition(intersect[0]); //if intersections exist, get LatLon coordinates for it

        ezgl::point2d zoom (0,0);

        zoom = {x_from_lon(point.lon()), y_from_lat(point.lat())};

        zoom = canvas1->get_camera().world_to_screen(zoom);

        ezgl::zoom_fit(canvas1, canvas1->get_camera().get_initial_world());
        double centerx = (x_from_lon((max_lon + min_lon)/2)); //find center for map you are in
        double centery = (y_from_lat((max_lat + min_lat)/2));
        double dx = x_from_lon(point.lon()) - centerx; //find translation distance
        double dy = y_from_lat(point.lat()) - centery;
        ezgl::translate(canvas1, dx, dy); //translate map to intersection and zoom into intersection
        ezgl::zoom_in(canvas1, 100.0);
    }
}

void load_maps (ezgl::application *application) {

    std::string input = gtk_combo_box_text_get_active_text(ezgl::load_entry); //input of which map is to be loaded
    std::cout<<input<<std::flush;
    close_map(); //close current map you are in
    std::string map_path = "/cad2/ece297s/public/maps/" + input + ".streets.bin";
    
    bool load_success = load_map(map_path);
    
    if(!load_success)
        std::cout <<"Failed to load map" <<map_path << "\n"; //if map cannot be loaded
    else {
        std::string main_canvas = application->get_main_canvas_id();
        auto my_canvas=application->get_canvas(main_canvas);
        ezgl::rectangle new_bounds({x_from_lon(min_lon), y_from_lat(min_lat)}, {x_from_lon(max_lon), y_from_lat(max_lat)}); //find new bounds of map

        my_canvas->get_camera().set_world(new_bounds);
        my_canvas->get_camera().set_initial_world(new_bounds); //set new bounds
        my_canvas->get_camera().update_widget(my_canvas->width(), my_canvas->height());

        application->refresh_drawing(); //draw new map

    }

}

void draw_streets(ezgl::renderer &g, ezgl::rectangle &view){
    double width = view.width();
    g.set_color(ezgl::GREY_55);                                       //line settings for streets
    g.set_line_dash(ezgl::line_dash::none);
    for (int i = 0; i < getNumStreetSegments(); i++){
        if (width <= 0.000103474) {g.set_line_width(20);}
        else {g.set_line_width(5);}                                                                                                               
        
            if (view.contains(segments[i].start_point) || view.contains(segments[i].end_point) || width <= find_street_segment_length(i)){
                if (segments[i].key == "highway"){
                    if (((segments[i].value == "motorway") || (segments[i].value == "motorway_link") || (segments[i].value == "trunk") || (segments[i].value == "trunk_link")) && (width <= 0.0122)){          //check if current street segment is a highway, as well as zoom level
                        if (!(segments[i].highlight)){
                            g.set_color(246, 207, 101,250);}
                        else{
                            g.set_color(ezgl::RED);}
                        //if (width <= 0.000103474) {g.set_line_width(20);}
                        //else {g.set_line_width(5);}                       
                        if (segments[i].curvepoint_count == 0){                                                   //if there are no curve points                            
                            for (int k = 5; k <= 15; k++){
                                g.set_font_size(k);
                                g.draw_line(segments[i].start_point, segments[i].end_point);                                                         //connect the start and end points of street segments without curve points
                            }
                        }
                        else{                                                                                                   //if curve points exist
                            for (int k = 5; k <= 15; k++){
                                g.set_font_size(k);
                                g.draw_line(segments[i].start_point, segments[i].curve_points[0].point);                
                            }
                            int count;
                            for (count = 1; count < segments[i].curvepoint_count; count++){											//if there are curve points present, connect each point to form the segment
                                for (int k = 5; k <= 15; k++){
                                    g.set_font_size(k);
                                    g.draw_line(segments[i].curve_points[count-1].point, 
                                                segments[i].curve_points[count].point);
                                }                    
                            }
                            for (int k = 5; k <= 15; k++){
                                g.set_font_size(k);
                                g.draw_line(segments[i].curve_points[count-1].point, segments[i].end_point);
                            }    
                        }
                    }
                    else if (((segments[i].value == "primary_link") || (segments[i].value == "secondary_link") || (segments[i].value == "tertiary_link")                                              //check for the next set of roads
                                || (segments[i].value == "primary") || (segments[i].value == "secondary") || (segments[i].value == "tertiary")) && (width <= 0.00134)){
                        g.set_line_width(10);
                        if (segments[i].highlight != true){
                        g.set_color(255, 242, 175,250); }
                        else{
                            g.set_color(ezgl::RED);}
                        if (getInfoStreetSegment(i).curvePointCount == 0){                                                   //if there are no curve points
                            g.draw_line(segments[i].start_point, segments[i].end_point); 
                        }
                        else{                                                                                                   //if curve points exist
                            g.draw_line(segments[i].start_point, segments[i].curve_points[0].point);       
                            int count;
                            for (count = 1; count < getInfoStreetSegment(i).curvePointCount; count++){
                                g.draw_line(segments[i].curve_points[count-1].point, 
                                                segments[i].curve_points[count].point);
                            }
                            g.draw_line(segments[i].curve_points[count-1].point, segments[i].end_point);
                        }
                    }
                    else if(width <= 0.000297428){                                                                                                                                //check if its any other road, and draw at the closest zoom
                        g.set_line_width(8);
                        if (segments[i].highlight != true){
                         g.set_color(ezgl::WHITE); }
                        else{
                            g.set_color(ezgl::RED);}
                        if (getInfoStreetSegment(i).curvePointCount == 0){                                                   //if there are no curve points
                            g.draw_line(segments[i].start_point, segments[i].end_point);       
                        }
                        else{                                                                                                   //if curve points exist
                            g.draw_line(segments[i].start_point, segments[i].curve_points[0].point); 
                            int count;
                            for (count = 1; count < getInfoStreetSegment(i).curvePointCount; count++){
                                g.draw_line(segments[i].curve_points[count-1].point, 
                                                segments[i].curve_points[count].point);
                            }
                            g.draw_line(segments[i].curve_points[count-1].point, segments[i].end_point);
                        }
                    }

                }
            }              
    }
    
}

void draw_street_names(ezgl::renderer &g, ezgl::rectangle &view){                                      //we are drawing street names in a seperate function from streets themselves
    
    double width = view.width();
    g.set_color(ezgl::BLACK);                                       //text settings
    ezgl::point2d midpoint = {0, 0};
    for (int i = 0; i < getNumStreetSegments(); i++){                                        
            if (view.contains(segments[i].start_point) || view.contains(segments[i].end_point) || width <= find_street_segment_length(i)){
                if ((segments[i].key == "highway")){//&&(getStreetName(getInfoStreetSegment(i).streetID) != "<unknown>")){
                    if ((segments[i].value == "motorway") || (segments[i].value == "motorway_link") || (segments[i].value == "trunk") || (segments[i].value == "trunk_link")){                        
                        if (segments[i].curvepoint_count == 0){                                                       //if there are no curve points                        
                            g.set_color(ezgl::BLACK);
                            g.set_font_size(15);
                            midpoint = segments[i].mid_point;                          
                            if (segments[i].oneway) {
                                g.set_text_rotation(segments[i].arrow_rotation);
                                g.draw_text(segments[i].arrow_point, "->", segments[i].seg_length*0.15, 10);            //arrows showing one way streets are drawn seperate
                            }																																								//from the street names themselves
                            g.set_text_rotation(segments[i].text_rotation);
                            g.draw_text(midpoint, getStreetName(getInfoStreetSegment(i).streetID), segments[i].seg_length*0.7, 10);								//arrows are draw at the corner of each street segment
                        }																																				//they are drawn in the first 15% length of each one way segment 
                        else{                                                                                                   //if curve points exist
                            g.set_font_size(15);
                            midpoint = segments[i].mid_point; 
                            
                            if (segments[i].oneway) {
                                g.set_text_rotation(segments[i].arrow_rotation);                    //arrow for first curve seg
                                g.draw_text(segments[i].arrow_point, 
                                        "->", segments[i].seg_length*0.15, 10);
                                int count;             
                                for (count = 0; count < segments[i].curvepoint_count; count++){             //arrow for the rest of the curve segments
                                    g.set_text_rotation(segments[i].curve_points[count].arrow_rot);
                                    g.draw_text(segments[i].curve_points[count].arrow, 
                                            "->", segments[i].curve_points[count].length*0.15, 10);
                                }                                                                 
                            }
                            
                            g.set_text_rotation(segments[i].text_rotation);                             //street name for first curve segment
                            g.draw_text(midpoint, getStreetName(getInfoStreetSegment(i).streetID), 
                                            segments[i].seg_length*0.7, 10);
                            int count;
                            for (count = 0; count < segments[i].curvepoint_count; count++){                       //street name for all other curve segments
                                midpoint = segments[i].curve_points[count].mid;     
                                        
                                g.set_text_rotation(segments[i].curve_points[count].text_rot);
                                
                                g.draw_text(midpoint, getStreetName(getInfoStreetSegment(i).streetID),
                                            segments[i].curve_points[count].length*0.7, 10);
                                
                            }
                            
                        }
                    }
                    else if (((segments[i].value == "primary_link") || (segments[i].value == "secondary_link") || (segments[i].value == "tertiary_link")
                                                   || (segments[i].value == "primary") || (segments[i].value == "secondary") || (segments[i].value == "tertiary")) && (width <= 0.00134)){
                        if (segments[i].curvepoint_count == 0){                                                       //if there are no curve points                        
                            g.set_color(ezgl::BLACK);
                            g.set_font_size(13);
                            midpoint = segments[i].mid_point;                          
                            //double distance = distance_between_xy_points({start_x, start_y}, {end_x, end_y});
                            if (segments[i].oneway) {
                                g.set_text_rotation(segments[i].arrow_rotation);
                                g.draw_text(segments[i].arrow_point, "->", segments[i].seg_length*0.15, 10);            //arrows showing one way streets are drawn seperate
                            }
                            g.set_text_rotation(segments[i].text_rotation);
                            g.draw_text(midpoint, getStreetName(getInfoStreetSegment(i).streetID), segments[i].seg_length*0.7, 10);	
                        }
                        else{                                                                                                   //if curve points exist
                            g.set_font_size(13);
                            midpoint = segments[i].mid_point; 
                            
                            if (segments[i].oneway) {
                                g.set_text_rotation(segments[i].arrow_rotation);                    //arrow for first curve seg
                                g.draw_text(segments[i].arrow_point, 
                                        "->", segments[i].seg_length*0.15, 10);
                                int count;            
                                for (count = 0; count < segments[i].curvepoint_count; count++){
                                    g.set_text_rotation(segments[i].curve_points[count].arrow_rot);
                                    g.draw_text(segments[i].curve_points[count].arrow, 
                                                    "->", segments[i].curve_points[count].length*0.15, 10);
                                }                                   
                                
                            }
                            
                            g.set_text_rotation(segments[i].text_rotation);                             //street name for first curve segment
                            g.draw_text(midpoint, getStreetName(getInfoStreetSegment(i).streetID), 
                                            segments[i].seg_length*0.7, 10);
                            int count;
                            for (count = 0; count < segments[i].curvepoint_count; count++){
                                midpoint = segments[i].curve_points[count].mid;     
                                        
                                g.set_text_rotation(segments[i].curve_points[count].text_rot);
                                
                                g.draw_text(midpoint, getStreetName(getInfoStreetSegment(i).streetID),
                                            segments[i].curve_points[count].length*0.7, 10);
                                
                            }                           
                        }
                    }
                    else if(width <= 0.000297428){
                        if (segments[i].curvepoint_count == 0){                                                       //if there are no curve points                        
                            g.set_color(ezgl::BLACK);                           
                            midpoint = segments[i].mid_point;                 
                            if (segments[i].oneway) {
                                g.set_font_size(15);
                                g.set_text_rotation(segments[i].arrow_rotation);
                                g.draw_text(segments[i].arrow_point, "->", segments[i].seg_length*0.15, 10);            //arrows showing one way streets are drawn seperate
                            }
                            g.set_font_size(10);
                            g.set_text_rotation(segments[i].text_rotation);
                            g.draw_text(midpoint, getStreetName(getInfoStreetSegment(i).streetID), segments[i].seg_length*0.7, 10);	
                        }
                        else{                                                                                                   //if curve points exist
                            g.set_font_size(10);
                            midpoint = segments[i].mid_point; 
                            
                            if (segments[i].oneway) {
                                g.set_text_rotation(segments[i].arrow_rotation);                    //arrow for first curve seg
                                g.draw_text(segments[i].arrow_point, 
                                        "->", segments[i].seg_length*0.15, 10);
                                int count;            
                                for (count = 0; count < segments[i].curvepoint_count; count++){
                                    g.set_text_rotation(segments[i].curve_points[count].arrow_rot);
                                    g.draw_text(segments[i].curve_points[count].arrow, 
                                                    "->", segments[i].curve_points[count].length*0.15, 10);
                                }                                                                  
                            }
                            
                            g.set_text_rotation(segments[i].text_rotation);                             //street name for first curve segment
                            g.draw_text(midpoint, getStreetName(getInfoStreetSegment(i).streetID), 
                                            segments[i].seg_length*0.7, 10);
                            int count;
                            for (count = 0; count < segments[i].curvepoint_count; count++){
                                midpoint = segments[i].curve_points[count].mid;     
                                        
                                g.set_text_rotation(segments[i].curve_points[count].text_rot);
                                
                                g.draw_text(midpoint, getStreetName(getInfoStreetSegment(i).streetID),
                                            segments[i].curve_points[count].length*0.7, 10);
                                
                            }                            
                        }
                    }

                }
            }
        
        
        
    }
    
}

int find_intersection_id(std::string street1, std::string street2){
    
    std::string street4 = auto_correct_find(street1); //autocorrect feature that can fix small spelling mistakes in streets
    std::string street5 = auto_correct_find(street2);

    std::vector<unsigned> street_id1 = find_street_ids_from_partial_street_name(street4); //find all street id's from the street or partial street given
    std::vector<unsigned> street_id2 = find_street_ids_from_partial_street_name(street5);

    std::vector<unsigned> intersect;

    if (street_id1.size() == 0) { //vector is unpopulated meaning no street exists
        std::cout<< street1 << " does not exist " << std::endl;
        return 0;
    }

    if (street_id2.size() == 0) {
        std::cout << street2 << " does not exist " << std::endl;
        return 0;
    }
   

    for (int i = 0; i < street_id1.size(); i++) {
        for (int j = 0; j < street_id2.size(); j++) {
            intersect = find_intersection_ids_from_street_ids(street_id1[i], street_id2[j]); //cross reference all street id's found to see what two streets actually intersect
            if (intersect.size() > 0) {
                for (auto id : intersect) {
                    if (!intersections[id].highlight) {
                        intersections[id].highlight = true; //highlight intersection points
                        std::cout << getIntersectionName(id) << std::endl;
                    }
                }
            }
        }
    }
   //std::cout<<"inside the "<<std::endl;
    intersect.clear();

    for (size_t i = 0; i < intersections.size(); i++) {
       if (intersections[i].highlight)
            intersect.push_back(i); //add back all highlighted intersections to vector
    }
   std::cout<< intersect[0] <<std::endl;
    return intersect[0];

}

void directions (ezgl::application *application) {
    
    std::string street1, street2;
    std::vector<unsigned> test;
    const char* text = gtk_entry_get_text(ezgl::text_entry_start); //user input from search bar as a single string with two streets
    std::string street3(text);
    
    int pos = street3.find_first_of("&"); //string of two streets is divided into street 1 and street 2
    street1 = street3.substr(0,pos-1);
    street2 = street3.substr(pos+2,street3.size());

    int intersection1 = find_intersection_id(street1, street2);

    std::string street5, street6;

    const char* text2 = gtk_entry_get_text(ezgl::text_entry_end); //user input from search bar as a single string with two streets
    std::string street4(text2);

    int pos2 = street4.find_first_of("&"); //string of two streets is divided into street 1 and street 2
    street5 = street4.substr(0,pos2-1);
    street6 = street4.substr(pos2+2,street4.size());
    
    int intersection2 = find_intersection_id(street5, street6);
    
    route = find_path_between_intersections(intersection1, intersection2, 15, 25);
    //id1 = intersection1;
    //id2 = intersection2;
    //route.clear();
    //std::cout << id1 << " " << id2 <<std::endl;
    //route = test;
    //draw_path(g,route);
    //std::cout << route.size() << std::endl;
    std::string direction = display_path_directions(route);
    
    std::cout << direction << std::endl;
   
    //gchar* g_string = &direction[0];
    //gchar* axe = new std::string(direction);
    //gtk_entry_set_text (ezgl::text_entry_dir, g_string); 
    
    auto zoom_canvas = application->get_canvas(application->get_main_canvas_id()); //retrieve canvas to change zoom position
    
    

    ezgl::zoom_fit(zoom_canvas, zoom_canvas->get_camera().get_initial_world()); //refit screen to focus on center
   
}

