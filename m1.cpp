/* 
 * Copyright 2020 University of Toronto
 *
 * Permission is hereby granted, to use this software and associated 
 * documentation files (the "Software") in course work at the University 
 * of Toronto, or for personal use. Other uses are prohibited, in 
 * particular the distribution of the Software either publicly or to third 
 * parties.
 *
 * The above copyright notice and this permission notice shall be included in 
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "m1.h"
#include "StreetsDatabaseAPI.h"
#include <cmath>
#include <vector> 
#include <algorithm>
#include <iostream>  
#include <map>
#include "OSMDatabaseAPI.h"
#include <ezgl/point.hpp>
#include "ezgl/global.h"
#include <ezgl/helpers.h>
#include "class.h"

struct feature_data{                                            
    double area;
    unsigned int id;
    std::vector<ezgl::point2d> points;
};

                              

struct curve_point_info{
    ezgl::point2d point = {0,0};   //start point
    ezgl::point2d end = {0,0};
    ezgl::point2d mid = {0,0};
    ezgl::point2d arrow = {0,0};
    double text_rot;
    double arrow_rot;
    double length;
};


struct seg_info{

    ezgl::point2d start_point = {0,0};
    ezgl::point2d end_point = {0,0};
    ezgl::point2d mid_point = {0,0};
    ezgl::point2d arrow_point = {0,0};
    std::string key, value;
    int curvepoint_count;
    double text_rotation, arrow_rotation, seg_length;
    bool oneway;
    std::vector<curve_point_info> curve_points; 

}; 

std::vector<seg_info> segments;                                   //MY NEW STUFF


std::vector<feature_data> poly;

std::vector<std::vector<unsigned> > intersection_street_segments;   //nope

std::vector<std::pair<double,unsigned> > street_ID_Segments;  //no

std::vector<double> street_segment_time;               //nope

std::vector<std::vector<unsigned> > all_intersections;    //don't need

std::vector<std::vector<unsigned> > all_segments;      //dont need

std::vector<std::string> street_seg_names; 

std::multimap<std::string, unsigned> street_names;      //need this

std::multimap<OSMID, unsigned> street_types;              //this is for streets using OSMID's

unsigned int numOfPOI;
unsigned int numOfIntersection;

double latavg, max_lat, min_lat, max_lon, min_lon;

std::string osm_path = "/cad2/ece297s/public/maps/toronto_canada.osm.bin";

void remove(std::vector<unsigned> &v)
{
	auto end = v.end();
	for (auto it = v.begin(); it != end; ++it) {
		end = std::remove(it + 1, end, *it);
	}
	v.erase(end, v.end());
}

void insert_sorted(std::vector<unsigned>& v, int n)
{
    // find the correct sorted insert position
    auto insert_itr = std::lower_bound(std::begin(v), std::end(v), n);

    // only insert if not a duplicate
    // test for end() first to make duplicate test safe
    if(insert_itr == std::end(v) || *insert_itr != n)
    {
        v.insert(insert_itr, n);
        
    }
}

void load_intersections(); 
void load_street_segment_names();  
void load_POI();
void load_features();

void load_streets_to_draw();          






 

bool load_map(std::string map_path) {
    bool load_successful = loadStreetsDatabaseBIN(map_path); //Indicates whether the map has loaded 
                                  //successfully
    //
    //Load your map related data structures here
    //


    if (load_successful){
        intersection_street_segments.resize(getNumIntersections());   //declare enough space for each data structure
        all_intersections.resize(getNumStreets());
        all_segments.resize(getNumStreets());

     
        numOfPOI = getNumPointsOfInterest(); 
        numOfIntersection = getNumIntersections();

        street_ID_Segments.resize(getNumStreetSegments());
        street_segment_time.reserve(getNumStreetSegments());  
        
        for (int intersection = 0; intersection < getNumIntersections(); intersection++){                               //initializing a nested vector where the index's 
            for (int segment_num = 0; segment_num < getIntersectionStreetSegmentCount(intersection); segment_num++) {  //represent intersection_ids and each corresponding
                StreetSegmentIndex seg_id = getIntersectionStreetSegment(segment_num, intersection);                    //vector stores the segment_ids connected to that 
                intersection_street_segments[intersection].push_back(seg_id);                                           //intersection
                insert_sorted(all_intersections[getInfoStreetSegment(seg_id).streetID], intersection);
                insert_sorted(all_segments[getInfoStreetSegment(seg_id).streetID], seg_id);  
            }
        } 
        
        load_intersections();
        load_street_segment_names();       //initialize the intersection data_structure for drawing
        load_POI();
        load_features();
       
        
        //default:
    std::string folder_path = "/cad2/ece297s/public/maps/";
    //std::string map_name;
    std::string map_name;
    for (unsigned i = folder_path.length(); i < map_path.length(); i++){
        if (map_path[i] == '.') break;
        map_name += map_path[i];
    }

    std::string osm_path = folder_path + map_name + ".osm.bin";
    std::cout << osm_path << std::endl;
    //Indicates whether the map has loaded successfully
        
        if (loadOSMDatabaseBIN(osm_path)){
            for (int i = 0; i<getNumberOfWays(); i++){                                      //initialize data structure with omsids and index
                OSMID ID = getWayByIndex(i)->id();
                street_types.insert(std::make_pair(ID, i));                       
            }
        }
        //load_streets_to_draw();                                                         //MY NEW STUFF
        id1 = 0;
        id2 = 0; 

        
        std::string name;
        for (unsigned i=0; i < getNumStreets(); i++){
            name = getStreetName(i);
            std::transform(name.begin(), name.end(), name.begin(), ::tolower);

            street_names.insert(std::make_pair(name, i));
        }
        
        int strCnt = getNumStreets();
   
        int strSeg = getNumStreetSegments();
        
        street_ID_Segments.resize(strSeg);

                                 
        for (int j = 0; j < getNumStreetSegments(); ++j){
            LatLon start_point, end_point;
  
            //street_segment_time.push_back(find_street_segment_length((j))/getInfoStreetSegment(j).speedLimit*3.6); //travel time for every street segment
            
            double distance=0;
            
            start_point = getIntersectionPosition(getInfoStreetSegment(j).from);
            end_point = getIntersectionPosition(getInfoStreetSegment(j).to);
            
            //check if there is curve point - if so increment through curve points and add up distances
            if ((getInfoStreetSegment(j).curvePointCount) != 0){
        
                distance += find_distance_between_two_points(start_point, getStreetSegmentCurvePoint(0, j));
                int i;
       
                for (i = 1; i < getInfoStreetSegment(j).curvePointCount; i++){
                    distance += find_distance_between_two_points(getStreetSegmentCurvePoint(i, j), getStreetSegmentCurvePoint(i-1, j)); 
                    }
                    
                    //increment distance after counting curve points
                    distance += find_distance_between_two_points(end_point, getStreetSegmentCurvePoint(i-1, j)); 
                    
                    street_ID_Segments[j].first = distance;
                    street_ID_Segments[j].second = j;
             } 
              
            else{
                    street_ID_Segments[j].first = find_distance_between_two_points(start_point, end_point);
                    street_ID_Segments[j].second = j;
             }
            street_segment_time.push_back(find_street_segment_length((j))/getInfoStreetSegment(j).speedLimit*3.6); //travel time for every street segment
            
        }  
        load_streets_to_draw();
        EdgeMap();
        NodeMap();               

    }
        
        
      

    return load_successful;
} 

void close_map() {
    intersection_street_segments.clear();
    all_intersections.clear();
    street_segment_time.clear();
    street_names.clear();
    street_types.clear();
    street_seg_names.clear();
    street_ID_Segments.clear();
    closeStreetDatabase();
    closeOSMDatabase();
    poly.clear();
    Egraph.clear();
    Ngraph.clear();
    segments.clear();                     
    //Clean-up your map related data structures here
}


std::vector<unsigned> find_intersection_street_segments(unsigned intersection_id){
  
    //use open map vector vector structure of street segments and intersection
    return intersection_street_segments[intersection_id];
}

std::vector<std::string> find_intersection_street_names(unsigned intersection_id){
    
    //get all street segments at the intersection
    std::vector<unsigned> seg = intersection_street_segments[intersection_id];
    
    //holds all the street indexes
    std::vector<StreetIndex> si;
    std::vector<std::string> name;
    
    for (unsigned int i = 0; i < seg.size(); i++){
        si.push_back(getInfoStreetSegment(seg[i]).streetID);
    }
    
    //get names for each street id of all segments in the intersection
    for (unsigned int i = 0; i < seg.size(); i++){
        name.push_back(getStreetName(si[i]));
    }
  
    return name;
}

bool are_directly_connected(unsigned intersection_id1, unsigned intersection_id2){
  
    std::vector<unsigned> segments = find_intersection_street_segments(intersection_id1);
    
    //check if its directly connected 
    if (intersection_id1 == intersection_id2)
        return true;
    
    //else go through all the segments checking if its one way and correct direction else check
    for (unsigned int i = 0; i < segments.size(); ++i){
        
        if (getInfoStreetSegment(segments[i]).oneWay && getInfoStreetSegment(segments[i]).to == intersection_id2)
            return true;
        
        else if (getInfoStreetSegment(segments[i]).to == intersection_id2 || getInfoStreetSegment(segments[i]).from == intersection_id2)
            return true;
    }
    return false;
}


std::vector<unsigned> find_adjacent_intersections(unsigned intersection_id){
    
    std::vector<unsigned> d;    
    int n = getIntersectionStreetSegmentCount(intersection_id);
    
    for (int i=0; i<n; i++) {
        
        if ((getInfoStreetSegment(getIntersectionStreetSegment(i,intersection_id)).oneWay)){ //if street segment is one way, adjacent intersection has to be the 'to' ID 
            
            if ((getInfoStreetSegment(getIntersectionStreetSegment(i,intersection_id)).to) != intersection_id){
               d.push_back(getInfoStreetSegment(getIntersectionStreetSegment(i,intersection_id)).to); 
            }
        }
        
        else {
            if (getInfoStreetSegment(getIntersectionStreetSegment(i,intersection_id)).to != intersection_id) //if street segment is two way, can be either to or from
             d.push_back(getInfoStreetSegment(getIntersectionStreetSegment(i,intersection_id)).to);
            
            else d.push_back(getInfoStreetSegment(getIntersectionStreetSegment(i,intersection_id)).from);
        }          
    }
    remove(d);
    
    return d;
}

std::vector<unsigned> find_street_street_segments(unsigned street_id){

 
     
   return all_segments[street_id]; 
    
}





double find_street_length(unsigned street_id) {
    double length = 0;
    std::vector<unsigned> n = find_street_street_segments(street_id);
   
    for (int i=0; i<n.size(); i++){ 
            length += find_street_segment_length(n[i]); //add length of every street segment for the street
    }
    return length; 
        
}

double find_street_segment_travel_time(unsigned street_segment_id){
   
    return street_segment_time[street_segment_id]; //return vector with specific time for that street segment
   
} 

double find_distance_between_two_points(LatLon point1, LatLon point2){

        double distance;
        
        double lat_avg = (point1.lat() + point2.lat())/2;
        
        double x1 = point1.lon()*cos(lat_avg*DEG_TO_RAD)*DEG_TO_RAD;
        double x2 = point2.lon()*cos(lat_avg*DEG_TO_RAD)*DEG_TO_RAD;    
        
        double y1 = point1.lat()*DEG_TO_RAD;   
        double y2 = point2.lat()*DEG_TO_RAD;
        
        distance = EARTH_RADIUS_IN_METERS * sqrt((y2-y1)*(y2-y1)+(x2-x1)*(x2-x1));

    return distance;
}

double find_street_segment_length(unsigned street_segment_id){
    
   return street_ID_Segments[street_segment_id].first;
}

//Returns the nearest point of interest to the given position
unsigned find_closest_point_of_interest(LatLon my_position){
 
    std::vector< std::pair <double,unsigned int> > closestPOI (numOfPOI);
 
    //loop hold distance between user and POI and holds original index
    for (unsigned int j=0; j < numOfPOI; ++j){
        closestPOI[j].first = find_distance_between_two_points(my_position, getPointOfInterestPosition(j));
        closestPOI[j].second = j;
    }    
    
    std::sort(closestPOI.begin(), closestPOI.end());
    
   
    return closestPOI[0].second;
}

//Returns the nearest intersection to the given position
unsigned find_closest_intersection(LatLon my_position){

    std::vector < std::pair <double,unsigned int> > nearestIntersection (numOfIntersection);
   
 
    //loop hold distance between user and Intersection and holds original index
    for (unsigned int j=0; j < numOfIntersection; ++j){
        nearestIntersection[j].first = find_distance_between_two_points(my_position, getIntersectionPosition(j));
        nearestIntersection[j].second = j;
    }    
    
    std::sort(nearestIntersection.begin(), nearestIntersection.end());
    
    
    return nearestIntersection[0].second;

}

std::vector<unsigned> find_street_ids_from_partial_street_name(std::string street_prefix){

    std::vector<unsigned> matching_streets;        
    std::transform(street_prefix.begin(), street_prefix.end(), street_prefix.begin(), ::tolower);    
    
    std::multimap<std::string, unsigned>::iterator low;
    std::multimap<std::string, unsigned>::iterator high;
    std::multimap<std::string, unsigned>::iterator it;

    low = street_names.lower_bound(street_prefix);
    
    street_prefix[street_prefix.size()-1]++;
    high = street_names.upper_bound(street_prefix);
    
    for (it = low; it != high; it++){
        matching_streets.push_back(it->second);
    }
    return matching_streets;
               
}


std::vector<unsigned> find_all_street_intersections(unsigned street_id){
    return all_intersections[street_id]; 
}

std::vector<unsigned> find_intersection_ids_from_street_ids(unsigned street_id1, unsigned street_id2){

   std::vector <unsigned> intersections1 = find_all_street_intersections(street_id1);
   std::vector <unsigned> intersections2 = find_all_street_intersections(street_id2);
   std::vector <unsigned> matching_intersections;

   
   std::set_intersection(intersections1.begin(), intersections1.end(),
                         intersections2.begin(), intersections2.end(),
                         std::back_inserter(matching_intersections));
   
   return matching_intersections;
    
}