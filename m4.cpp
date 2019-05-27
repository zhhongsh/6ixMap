/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include "m4.h" 
#include "m1.h"
#include "m2.h"
#include "m3.h"
#include "LatLon.h"
#include "StreetsDatabaseAPI.h"
#include <ezgl/point.hpp>
#include <ezgl/helpers.h>
#include "ezgl/global.h"
#include "ezgl/application.hpp"
#include "math.h"
#include <iostream>
#include <string>
#include <algorithm>
#include <cmath>
#include "class.h"
#include <list>
#include <queue>
#include <chrono>   // Time utilities
#define TIME_LIMIT 45  // m4: 45 second time limit

struct delivery_info{                        //need bool variables for each item, so might as well make a new struct
    unsigned pick_Up;
    unsigned drop_Off;
    float weight;
    bool picked_Up;
    bool dropped_Off;            //not using yet
};

double GIANT_NUMBER = 9999999999;

std::vector<unsigned> shortest_path_between_source_and_multipleDest(unsigned source_id, std::vector<delivery_info>& destinations, 
                                                                    double right_turn_penalty, double left_turn_penalty, unsigned &dest_index);

double compute_path_distance(const std::vector<unsigned>& path); 

std::vector<unsigned> shortest_path_between_dropoff_and_depot(unsigned source_id, const std::vector<unsigned>& depots, 
                                                              double right_turn_penalty, double left_turn_penalty, unsigned &depot_index);

std::vector<unsigned> path_to_closest_pickup_or_dropoff(unsigned source_id, std::vector<delivery_info>& destinations, 
                                                        double right_turn_penalty, double left_turn_penalty, unsigned &dest_index, float &truck_capacity, bool &pickup);

std::vector<CourierSubpath> combine_duplicates(std::vector<CourierSubpath>& travel_path);

double find_travel_path_time(std::vector<CourierSubpath> travel_path, const float right_turn_penalty, const float left_turn_penalty); 

std::vector<CourierSubpath> best_depot(unsigned source_id, std::vector<delivery_info> deliveryVec, const std::vector<unsigned>& depots, const std::vector<DeliveryInfo>& deliveries,
                                        const float right_turn_penalty, const float left_turn_penalty, float capacity);

std::vector<CourierSubpath> best_starting_pickup(unsigned depot_id, unsigned source_id, std::vector<delivery_info> deliveryVec, const std::vector<DeliveryInfo>& deliveries, 
                                                 unsigned pickup_index, const float right_turn_penalty, const float left_turn_penalty, float capacity);


std::vector<CourierSubpath> traveling_courier(
		const std::vector<DeliveryInfo>& deliveries,
	       	const std::vector<unsigned>& depots, 
		const float right_turn_penalty, 
		const float left_turn_penalty, 
		const float truck_capacity){

    
    int Num_deliveries = deliveries.size();  //# of items to be delivered 
    std::vector<delivery_info> deliveryVec;
    deliveryVec.resize(Num_deliveries);
    
    std::cout << "number of depots: " << depots.size() << std::endl; 
    std::cout << "number of deliveries" << Num_deliveries <<std::endl;
    
    float capacity = truck_capacity;
    
    for(int i = 0; i < Num_deliveries; i++){                          //initialize the new delivery info vector
        deliveryVec[i].pick_Up = deliveries[i].pickUp;
        deliveryVec[i].drop_Off = deliveries[i].dropOff;
        deliveryVec[i].weight = deliveries[i].itemWeight;
        deliveryVec[i].picked_Up = false;                           //initialy nothing has been picked up or dropped off
        deliveryVec[i].dropped_Off= false; 
    }
       
    std::vector<CourierSubpath> best_travel_path;
    std::vector<CourierSubpath> travel_path;
    double best_time = GIANT_NUMBER;
    double current_time;
    if (depots.size() > 1){
        for (unsigned i = 0; i < depots.size(); i++){
            travel_path = best_depot(depots[i], deliveryVec, depots, deliveries, right_turn_penalty, left_turn_penalty, capacity);
            current_time = find_travel_path_time(travel_path, right_turn_penalty, left_turn_penalty);

            if((current_time < best_time) && (travel_path.size()!= 0)){
                best_time = current_time;
                best_travel_path = travel_path;
            }
        }
    }
        /*else{
        //find intersection
        int partition = Num_deliveries/4;
        
        std::vector<CourierSubpath> best_travel_path2;
        std::vector<CourierSubpath> travel_path2;
        double best_time2 = GIANT_NUMBER;
        double current_time2, current_time3, current_time4;
        std::vector<CourierSubpath> best_travel_path3;
        std::vector<CourierSubpath> travel_path3;
        double best_time3 = GIANT_NUMBER;
        std::vector<CourierSubpath> best_travel_path4;
        std::vector<CourierSubpath> travel_path4;
        double best_time4 = GIANT_NUMBER;
          
#pragma omp parallel for 
        for (unsigned i = 0; i < partition; i++){
             #pragma omp critical
            travel_path = best_starting_pickup(depots[0], deliveryVec[i].pick_Up, deliveryVec, deliveries, i, right_turn_penalty, left_turn_penalty, capacity);
             #pragma omp critical
            current_time = find_travel_path_time(travel_path, right_turn_penalty, left_turn_penalty);
            
            if((current_time < best_time) && (travel_path.size()!= 0)){
                #pragma omp critical
                best_time = current_time;
                #pragma omp critical
                best_travel_path = travel_path;
            }
        }
 #pragma omp parallel for 
        for (unsigned i = partition; i < 2*partition; i++){
             #pragma omp critical
            travel_path2 = best_starting_pickup(depots[0], deliveryVec[i].pick_Up, deliveryVec, deliveries, i, right_turn_penalty, left_turn_penalty, capacity);
             #pragma omp critical
            current_time2 = find_travel_path_time(travel_path2, right_turn_penalty, left_turn_penalty);
            
            if((current_time2 < best_time2) && (travel_path2.size()!= 0)){
                 #pragma omp critical
                best_time2 = current_time2;
                 #pragma omp critical
                best_travel_path2 = travel_path2;
            }

        }
#pragma omp parallel for 
        for (unsigned i = 2*partition; i < 3*partition; i++){
             #pragma omp critical
            travel_path3 = best_starting_pickup(depots[0], deliveryVec[i].pick_Up, deliveryVec, deliveries, i, right_turn_penalty, left_turn_penalty, capacity);
             #pragma omp critical
            current_time3 = find_travel_path_time(travel_path3, right_turn_penalty, left_turn_penalty);
            
            if((current_time3 < best_time3) && (travel_path3.size()!= 0)){
                 #pragma omp critical
                best_time3 = current_time3;
                 #pragma omp critical
                best_travel_path3 = travel_path3;
            }
        }
#pragma omp parallel for 
        for (unsigned i = 3*partition; i < 4*partition; i++){
             #pragma omp critical
            travel_path4 = best_starting_pickup(depots[0], deliveryVec[i].pick_Up, deliveryVec, deliveries, i, right_turn_penalty, left_turn_penalty, capacity);
             #pragma omp critical
            current_time4 = find_travel_path_time(travel_path4, right_turn_penalty, left_turn_penalty);
            
            if((current_time4 < best_time4) && (travel_path4.size()!= 0)){
                 #pragma omp critical
                best_time4 = current_time4;
                 #pragma omp critical
                best_travel_path4 = travel_path4;
            }
        }
    
    
    
        if (best_time2 < best_time && best_time2 < best_time3 && best_time2 < best_time4){
            return best_travel_path2;
        }
        else if (best_time3 < best_time && best_time3 < best_time2 && best_time3 < best_time4){
            return best_travel_path3;
        }
        else if (best_time4 < best_time && best_time4 < best_time3 && best_time4 < best_time2){
            return best_travel_path4;
        }
        
    }*/
   else{
        
        std::vector< std::pair <std::vector<CourierSubpath>, double> > paired_times;
        paired_times.resize(Num_deliveries);
        //find intersection
        auto startTime = std::chrono::high_resolution_clock::now();
        bool timeOut = false;
        
        unsigned depot = depots[0];
        std::vector<delivery_info> del = deliveryVec;
        
        
#pragma omp parallel for        
        for (unsigned i = 0; i < Num_deliveries; i++){ 
            //myOptimizer ();
            //if(timeOut) break;
            //auto currentTime = std::chrono::high_resolution_clock::now();
            //auto wallClock = std::chrono::duration_cast<std::chrono::duration<double>> (currentTime - startTime);
            //if (wallClock.count() > 0.85 * TIME_LIMIT) timeOut = true;
            
            std::vector<CourierSubpath> travel_path2 = best_starting_pickup(depot, deliveryVec[i].pick_Up, del, deliveries, i, right_turn_penalty, left_turn_penalty, capacity);
          
            double current_time2 = find_travel_path_time(travel_path2, right_turn_penalty, left_turn_penalty);
            
            //if((current_time < best_time) && (travel_path.size()!= 0)){
            //    best_time = current_time;
            //    best_travel_path = travel_path;
            //}
            #pragma omp critical
            paired_times[i].first = travel_path2;
            #pragma omp critical
            paired_times[i].second = current_time2;
        }
        
        
        for (int i=0; i <Num_deliveries; i++){          
            if ((paired_times[i].second < best_time) && (paired_times[i].first.size()!= 0)){
                best_time = paired_times[i].second;
                best_travel_path = paired_times[i].first;
          
            }
        }
        
    }
//    for (int j = 0; j < travel_path.size();  j++){
//        std::cout << "start " << best_travel_path[j].start_intersection << " end: " << best_travel_path[j].end_intersection << std::endl; 
//    }
    
    return best_travel_path;
}

std::vector<CourierSubpath> best_starting_pickup(unsigned depot_id, unsigned source_id, std::vector<delivery_info> deliveryVec, const std::vector<DeliveryInfo>& deliveries, 
                                                 unsigned pickup_index, const float right_turn_penalty, const float left_turn_penalty, float capacity){
    
    std::vector<CourierSubpath> travel_path;
    travel_path.resize(2*deliveries.size()+1);
    
    unsigned n = 0;
    
    for(int i = 0; i < deliveries.size(); i++){                          //initialize the new delivery info vector
        deliveryVec[i].pick_Up = deliveries[i].pickUp;
        deliveryVec[i].drop_Off = deliveries[i].dropOff;
        deliveryVec[i].weight = deliveries[i].itemWeight;
        deliveryVec[i].picked_Up = false;                           //initialy nothing has been picked up or dropped off
        deliveryVec[i].dropped_Off= false; 
    }
    
    travel_path[0].start_intersection = depot_id;
    travel_path[0].end_intersection = source_id;
    travel_path[0].subpath = find_path_between_intersections(travel_path[0].start_intersection, travel_path[0].end_intersection, right_turn_penalty, left_turn_penalty);
    travel_path[0].pickUp_indices.resize(0);
    deliveryVec[pickup_index].picked_Up = true;
    
    if (travel_path[0].subpath.size() == 0){
        travel_path.resize(0);
        return travel_path;
    }
    
    capacity = capacity - deliveryVec[pickup_index].weight;
    travel_path[1].pickUp_indices.resize(1);
    travel_path[1].pickUp_indices[0] = pickup_index;
    
    bool going_to_pickup = true;
    unsigned i;

    unsigned dest_index;
    for (i = 1; i < travel_path.size()-1; i++){

        travel_path[i].start_intersection = travel_path[i-1].end_intersection;

        travel_path[i].subpath = path_to_closest_pickup_or_dropoff(travel_path[i].start_intersection, deliveryVec, 
                                                                    right_turn_penalty, left_turn_penalty, dest_index, capacity, going_to_pickup);

        if (going_to_pickup){
            travel_path[i].end_intersection = deliveryVec[dest_index].pick_Up;
            travel_path[i+1].pickUp_indices.resize(1);
            travel_path[i+1].pickUp_indices[0] = dest_index;
            deliveryVec[dest_index].picked_Up = true;
        }
        
        else {

            travel_path[i].end_intersection = deliveryVec[dest_index].drop_Off;
            travel_path[i+1].pickUp_indices.resize(0);
            deliveryVec[dest_index].dropped_Off = true;
        }

    }
    unsigned depots_index;
    travel_path[i].start_intersection = travel_path[i-1].end_intersection; 
    travel_path[i].end_intersection = depot_id;
    travel_path[i].subpath = find_path_between_intersections(travel_path[i].start_intersection, travel_path[i].end_intersection, right_turn_penalty, left_turn_penalty);
    travel_path[i].pickUp_indices.resize(0);
    
    travel_path = combine_duplicates(travel_path);
    return travel_path;
    
}

std::vector<CourierSubpath> best_depot(unsigned source_id, std::vector<delivery_info> deliveryVec, const std::vector<unsigned>& depots, const std::vector<DeliveryInfo>& deliveries,
                                        const float right_turn_penalty, const float left_turn_penalty, float capacity){
    
    std::vector<CourierSubpath> travel_path;
    travel_path.resize(2*deliveries.size()+1);
    
    unsigned n = 0;   
    
    for(int i = 0; i < deliveries.size(); i++){                          //initialize the new delivery info vector
        deliveryVec[i].pick_Up = deliveries[i].pickUp;
        deliveryVec[i].drop_Off = deliveries[i].dropOff;
        deliveryVec[i].weight = deliveries[i].itemWeight;
        deliveryVec[i].picked_Up = false;                           //initialy nothing has been picked up or dropped off
        deliveryVec[i].dropped_Off= false; 
    }
    
    travel_path[0].start_intersection = source_id;
    travel_path[0].subpath = shortest_path_between_source_and_multipleDest(travel_path[0].start_intersection, deliveryVec, right_turn_penalty, left_turn_penalty, n);     
    travel_path[0].end_intersection = deliveryVec[n].pick_Up;

    if (travel_path[0].subpath.size() == 0){
        travel_path.resize(0);
        return travel_path;
    }
    
    travel_path[0].pickUp_indices.resize(0);
    deliveryVec[n].picked_Up = true;
    capacity = capacity - deliveryVec[n].weight;
    travel_path[1].pickUp_indices.resize(1);
    travel_path[1].pickUp_indices[0] = n;
    
    bool going_to_pickup = true;
    unsigned i;

    unsigned dest_index;
    for (i = 1; i < travel_path.size()-1; i++){

        travel_path[i].start_intersection = travel_path[i-1].end_intersection;

        travel_path[i].subpath = path_to_closest_pickup_or_dropoff(travel_path[i].start_intersection, deliveryVec, 
                                                                    right_turn_penalty, left_turn_penalty, dest_index, capacity, going_to_pickup);

        if (going_to_pickup){
            travel_path[i].end_intersection = deliveryVec[dest_index].pick_Up;
            travel_path[i+1].pickUp_indices.resize(1);
            travel_path[i+1].pickUp_indices[0] = dest_index;
            deliveryVec[dest_index].picked_Up = true;
        }
        
        else {

            travel_path[i].end_intersection = deliveryVec[dest_index].drop_Off;
            travel_path[i+1].pickUp_indices.resize(0);
            deliveryVec[dest_index].dropped_Off = true;
        }

    }
    
    unsigned depots_index;
    travel_path[i].start_intersection = travel_path[i-1].end_intersection; 
    travel_path[i].subpath = shortest_path_between_dropoff_and_depot(travel_path[i].start_intersection, depots, right_turn_penalty, left_turn_penalty, depots_index);
    travel_path[i].end_intersection = depots[depots_index];
    travel_path[i].pickUp_indices.resize(0);
    
    travel_path = combine_duplicates(travel_path);
    return travel_path;    
    
}

double find_travel_path_time(std::vector<CourierSubpath> travel_path, const float right_turn_penalty, const float left_turn_penalty){
    double time = 0;
    for (int i = 0; i < travel_path.size(); i++){
        time += compute_path_travel_time (travel_path[i].subpath, right_turn_penalty, left_turn_penalty);
    }
    return time;
}

std::vector<CourierSubpath> combine_duplicates(std::vector<CourierSubpath>& travel_path){
    std::vector<CourierSubpath> new_travel_path;
    new_travel_path.resize(travel_path.size());
    
    int index = 0;
    for (int i = 0; i < travel_path.size(); i++){
        if (travel_path[i].start_intersection == travel_path[i].end_intersection){    //have identified a redundant path
            if (travel_path[i].pickUp_indices.size() > 0){                                  //this means that this is a pickup 
                for (unsigned j = 0; j < travel_path[i].pickUp_indices.size(); j++){                    //concatenate the current pickup indices with the next one
                    travel_path[i+1].pickUp_indices.push_back(travel_path[i].pickUp_indices[j]);
                }
            }
            else if (travel_path[i].pickUp_indices.size() == 0){
                //DO NOTHING
            }
        }
        else {
            new_travel_path[index] = travel_path[i];
            index++;
        }
    }
    new_travel_path.resize(index);
    return new_travel_path;
}


std::vector<unsigned> path_to_closest_pickup_or_dropoff(unsigned source_id, std::vector<delivery_info>& destinations, 
                                                        double right_turn_penalty, double left_turn_penalty, unsigned &dest_index, float &truck_capacity, bool &pickup){
    double best_distance = GIANT_NUMBER;
    int n = destinations.size();
    double current_distance;
    std::vector<unsigned> best_path;
    
    for(unsigned i = 0; i < n; i++){
        if (!destinations[i].picked_Up && (destinations[i].weight <= truck_capacity)){
            current_distance = find_distance_between_two_points(get_intersection_position(source_id), get_intersection_position(destinations[i].pick_Up));
            if (current_distance < best_distance){
                best_distance = current_distance;
                dest_index = i;
                pickup = true;
            }
        }
        if (destinations[i].picked_Up && !destinations[i].dropped_Off){
            current_distance = find_distance_between_two_points(get_intersection_position(source_id), get_intersection_position(destinations[i].drop_Off));
            if(current_distance < best_distance){
                best_distance = current_distance;
                dest_index = i;
                pickup = false;
            }
        }
    }
    
    if (pickup){
        best_path = find_path_between_intersections(source_id, destinations[dest_index].pick_Up, right_turn_penalty, left_turn_penalty);
        truck_capacity = truck_capacity - destinations[dest_index].weight;
    }
    else if (!pickup){
        best_path = find_path_between_intersections(source_id, destinations[dest_index].drop_Off, right_turn_penalty, left_turn_penalty);
        truck_capacity = truck_capacity + destinations[dest_index].weight;
    }
    
    return best_path;
}


std::vector<unsigned> shortest_path_between_source_and_multipleDest(unsigned source_id, std::vector<delivery_info>& destinations, 
                                                                    double right_turn_penalty, double left_turn_penalty, unsigned &dest_index){
    int n = destinations.size();
    double best_travel_distance = GIANT_NUMBER;   //initialize with giant number that will most definitely not be the best travel distance; 
    double current_travel_distance;
    std::vector<unsigned> path;
    std::vector<unsigned> bestPath;

    for (unsigned i = 0; i < n; i++){
        if (!destinations[i].picked_Up){
            current_travel_distance = find_distance_between_two_points(get_intersection_position(source_id), get_intersection_position(destinations[i].pick_Up));
            if (current_travel_distance < best_travel_distance){
                best_travel_distance = current_travel_distance;
                //bestPath = path;
                dest_index = i;
            } 
        }
    }
    bestPath = find_path_between_intersections(source_id, destinations[dest_index].pick_Up, right_turn_penalty, left_turn_penalty);
    return bestPath;
}

double compute_path_distance(const std::vector<unsigned>& path){             //not using atm 
    double distance = 0; 
    for (unsigned i = 0; i < path.size(); i++){
        distance += find_street_segment_length( path[i]);
    }
    return distance;
}

std::vector<unsigned> shortest_path_between_dropoff_and_depot(unsigned source_id, const std::vector<unsigned>& depots, 
                                                              double right_turn_penalty, double left_turn_penalty, unsigned &depot_index){
    
    int n = depots.size();
    double best_travel_distance = 9999999999;
    double current_travel_distance;
    std::vector<unsigned> best_path;
    
    for (unsigned i = 0; i < n; i++){
        current_travel_distance = find_distance_between_two_points(get_intersection_position(source_id), get_intersection_position(depots[i]));
        if (current_travel_distance < best_travel_distance) {
             best_travel_distance = current_travel_distance;
             depot_index = i;
        }
    }
    best_path = find_path_between_intersections(source_id, depots[depot_index], right_turn_penalty, left_turn_penalty);
    return best_path;
    
}





 