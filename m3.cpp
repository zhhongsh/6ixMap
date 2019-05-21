/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

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

#define NO_EDGE -1 //illegal edge
std::vector<Node> Ngraph; 
std::vector<Edge> Egraph;

double towards(unsigned a, unsigned b);
void setClear();
double cross_product(double x1, double y1, double x2, double y2);
bool check_legal(int edge, int node);

double cross_product(double x1, double y1, double x2, double y2){
    double z = x1*y2 - x2*y1;
    return z;
}

TurnType find_turn_type(unsigned street_segment1, unsigned street_segment2){
    unsigned seg1 = street_segment1;
    unsigned seg2 = street_segment2;
    if (getInfoStreetSegment(seg1).streetID == getInfoStreetSegment(seg2).streetID) return TurnType::STRAIGHT;  //if the 2 street_seg ids are the same
    ezgl::point2d start1 (0,0);
    ezgl::point2d end1 (0,0);
    ezgl::point2d start2 (0,0);
    ezgl::point2d end2 (0,0);
    double x1, y1, x2, y2;       //the x and y components of the segment vectors
    unsigned from1 = getInfoStreetSegment(seg1).from;  //just to make for less typing
    unsigned to1 = getInfoStreetSegment(seg1).to;
    unsigned from2 = getInfoStreetSegment(seg2).from;
    unsigned to2 = getInfoStreetSegment(seg2).to;
    
    if (from1 != from2 && from1 != to2           //if the segs are not
            && to1 != from2 && to1 != to2){          //connected
        return TurnType::NONE;
    }
    
    
    if (getInfoStreetSegment(seg1).curvePointCount == 0 && getInfoStreetSegment(seg2).curvePointCount == 0){   //if both segments have no curve points
            
        //find which intersection the 2 segments intersect at
        if (from1 == from2){
            start2 = {getIntersectionPosition(from1).lon(), getIntersectionPosition(from1).lat()};
            end1 = start2;
            start1 = {getIntersectionPosition(to1).lon(), getIntersectionPosition(to1).lat()};
            end2 = {getIntersectionPosition(to2).lon(), getIntersectionPosition(to2).lat()};
        }
        else if (from1 == to2){
            start2 = {getIntersectionPosition(to2).lon(), getIntersectionPosition(to2).lat()};
            end1 = start2;
            start1 = {getIntersectionPosition(to1).lon(), getIntersectionPosition(to1).lat()};
            end2 = {getIntersectionPosition(from2).lon(), getIntersectionPosition(from2).lat()};
        }
        else if (to1 == from2){
            start2 = {getIntersectionPosition(from2).lon(), getIntersectionPosition(from2).lat()};
            end1 = start2;
            start1 = {getIntersectionPosition(from1).lon(), getIntersectionPosition(from1).lat()};
            end2 = {getIntersectionPosition(to2).lon(), getIntersectionPosition(to2).lat()};
        }
        else if (to1 == to2){
            start2 = {getIntersectionPosition(to2).lon(), getIntersectionPosition(to2).lat()};
            end1 = start2;
            start1 = {getIntersectionPosition(from1).lon(), getIntersectionPosition(from1).lat()};
            end2 = {getIntersectionPosition(from2).lon(), getIntersectionPosition(from2).lat()};
        }
        x1 = start1.x - end1.x;  //remember, we want these vectors to be tail-to-tail
        y1 = start1.y - end1.y;
        x2 = end2.x - start2.x;
        y2 = end2.y - start2.y;
        
        if (cross_product(x1, y1, x2, y2) >= 0) return TurnType::RIGHT;
        else return TurnType::LEFT;
                   

    }
    
    else if(getInfoStreetSegment(seg1).curvePointCount > 0 && getInfoStreetSegment(seg2).curvePointCount == 0){   //if the incoming street (seg1) has curve points
        if (from1 == from2){
            start2 = {getIntersectionPosition(from2).lon(), getIntersectionPosition(from2).lat()};
            end1 = start2;
            start1 = {getStreetSegmentCurvePoint(0, seg1).lon(), getStreetSegmentCurvePoint(0, seg1).lat()};
            end2 = {getIntersectionPosition(to2).lon(), getIntersectionPosition(to2).lat()};
        }
        else if (from1 == to2){
            start2 = {getIntersectionPosition(to2).lon(), getIntersectionPosition(to2).lat()};
            end1 = start2;
            start1 = {getStreetSegmentCurvePoint(0, seg1).lon(), getStreetSegmentCurvePoint(0, seg1).lat()};
            end2 = {getIntersectionPosition(from2).lon(), getIntersectionPosition(from2).lat()};
        }
        else if(to1 == from2){
            start2 = {getIntersectionPosition(from2).lon(), getIntersectionPosition(from2).lat()};
            end1 = start2;
            start1 = {getStreetSegmentCurvePoint(getInfoStreetSegment(seg1).curvePointCount - 1, seg1).lon(), 
                getStreetSegmentCurvePoint(getInfoStreetSegment(seg1).curvePointCount - 1, seg1).lat()};
            end2 = {getIntersectionPosition(to2).lon(), getIntersectionPosition(to2).lat()};
        }
        else if(to1 == to2){
            start2 = {getIntersectionPosition(to2).lon(), getIntersectionPosition(to2).lat()};
            end1 = start2;
            start1 = {getStreetSegmentCurvePoint(getInfoStreetSegment(seg1).curvePointCount - 1, seg1).lon(), 
                getStreetSegmentCurvePoint(getInfoStreetSegment(seg1).curvePointCount - 1, seg1).lat()};
            end2 = {getIntersectionPosition(from2).lon(), getIntersectionPosition(from2).lat()};
        }
        x1 = start1.x - end1.x;  //remember, we want these vectors to be tail-to-tail
        y1 = start1.y - end1.y;
        x2 = end2.x - start2.x;
        y2 = end2.y - start2.y;
        
        if (cross_product(x1, y1, x2, y2) >= 0) return TurnType::RIGHT;
        else return TurnType::LEFT;
    }
    else if(getInfoStreetSegment(seg1).curvePointCount == 0 && getInfoStreetSegment(seg2).curvePointCount > 0){
        if (from1 == from2){
            start2 = {getIntersectionPosition(from2).lon(), getIntersectionPosition(from2).lat()};
            end1 = start2;
            start1 = {getIntersectionPosition(to1).lon(), getIntersectionPosition(to1).lat()};
            end2 = {getStreetSegmentCurvePoint(0, seg2).lon(), getStreetSegmentCurvePoint(0, seg2).lat()};
        }
        else if(from1 == to2){
            start2 = {getIntersectionPosition(to2).lon(), getIntersectionPosition(to2).lat()};
            end1 = start2;
            start1 = {getIntersectionPosition(to1).lon(), getIntersectionPosition(to1).lat()};
            end2 = {getStreetSegmentCurvePoint(getInfoStreetSegment(seg2).curvePointCount - 1, seg2).lon(), 
                getStreetSegmentCurvePoint(getInfoStreetSegment(seg2).curvePointCount - 1, seg2).lat()};
        }
        else if(to1 == from2){
            start2 = {getIntersectionPosition(from2).lon(), getIntersectionPosition(from2).lat()};
            end1 = start2;
            start1 = {getIntersectionPosition(from1).lon(), getIntersectionPosition(from1).lat()};
            end2 = {getStreetSegmentCurvePoint(0, seg2).lon(), getStreetSegmentCurvePoint(0, seg2).lat()};
        }
        else if(to1 == to2){
            start2 = {getIntersectionPosition(to2).lon(), getIntersectionPosition(to2).lat()};
            end1 = start2;
            start1 = {getIntersectionPosition(from1).lon(), getIntersectionPosition(from1).lat()};
            end2 = {getStreetSegmentCurvePoint(getInfoStreetSegment(seg2).curvePointCount - 1, seg2).lon(), 
                getStreetSegmentCurvePoint(getInfoStreetSegment(seg2).curvePointCount - 1, seg2).lat()};
        }
        x1 = start1.x - end1.x;  //remember, we want these vectors to be tail-to-tail
        y1 = start1.y - end1.y;
        x2 = end2.x - start2.x;
        y2 = end2.y - start2.y;
        
        if (cross_product(x1, y1, x2, y2) >= 0) return TurnType::RIGHT;
        else return TurnType::LEFT;
    }
    else if(getInfoStreetSegment(seg1).curvePointCount > 0 && getInfoStreetSegment(seg2).curvePointCount > 0){
        if (from1 == from2){
            start2 = {getIntersectionPosition(from2).lon(), getIntersectionPosition(from2).lat()};
            end1 = start2;
            start1 = {getStreetSegmentCurvePoint(0, seg1).lon(), getStreetSegmentCurvePoint(0, seg1).lat()};
            end2 = {getStreetSegmentCurvePoint(0, seg2).lon(), getStreetSegmentCurvePoint(0, seg2).lat()};
        }
        else if(from1 == to2){
            start2 = {getIntersectionPosition(to2).lon(), getIntersectionPosition(to2).lat()};
            end1 = start2;
            start1 = {getStreetSegmentCurvePoint(0, seg1).lon(), getStreetSegmentCurvePoint(0, seg1).lat()};
            end2 = {getStreetSegmentCurvePoint(getInfoStreetSegment(seg2).curvePointCount - 1, seg2).lon(), 
                getStreetSegmentCurvePoint(getInfoStreetSegment(seg2).curvePointCount - 1, seg2).lat()};
        }
        else if(to1 == from2){
            start2 = {getIntersectionPosition(from2).lon(), getIntersectionPosition(from2).lat()};
            end1 = start2;
            start1 = {getStreetSegmentCurvePoint(getInfoStreetSegment(seg1).curvePointCount - 1, seg1).lon(), 
                getStreetSegmentCurvePoint(getInfoStreetSegment(seg1).curvePointCount - 1, seg1).lat()};
            end2 = {getStreetSegmentCurvePoint(0, seg2).lon(), getStreetSegmentCurvePoint(0, seg2).lat()};
        }
        else if (to1 == to2){
            start2 = {getIntersectionPosition(to2).lon(), getIntersectionPosition(to2).lat()};
            end1 = start2;
            start1 = {getStreetSegmentCurvePoint(getInfoStreetSegment(seg1).curvePointCount - 1, seg1).lon(), 
                getStreetSegmentCurvePoint(getInfoStreetSegment(seg1).curvePointCount - 1, seg1).lat()};
            end2 = {getStreetSegmentCurvePoint(getInfoStreetSegment(seg2).curvePointCount - 1, seg2).lon(), 
                getStreetSegmentCurvePoint(getInfoStreetSegment(seg2).curvePointCount - 1, seg2).lat()};
        }
        x1 = start1.x - end1.x;  //remember, we want these vectors to be tail-to-tail
        y1 = start1.y - end1.y;
        x2 = end2.x - start2.x;
        y2 = end2.y - start2.y;
        
        if (cross_product(x1, y1, x2, y2) >= 0) return TurnType::RIGHT;
        else return TurnType::LEFT;
    }
    
}


double compute_path_travel_time(const std::vector<unsigned>& path, 
                                const double right_turn_penalty,
                                const double left_turn_penalty) 
{
    double time = 0;          
    if (path.size() == 0){
        return time; 
    }
    TurnType turn;
    
    for (unsigned i = 0; i < path.size(); i++){                        //iterate through the path and find turn types
        time = time + find_street_segment_travel_time(path[i]);             //add penalties and segment travel times accordingly
        
        if (i != (path.size()-1)){ 
            turn = find_turn_type(path[i], path[i+1]);
            if (turn == TurnType::LEFT) time = time + left_turn_penalty;
            else if (turn == TurnType::RIGHT) time = time + right_turn_penalty;  
            else if (turn == TurnType::NONE) return 0;
        }
    
    }
    return time;
}

//used for A* to determine distance from goal to next
double towards(unsigned a, unsigned b){
    
    LatLon point1 = getIntersectionPosition(a);
    LatLon point2 = getIntersectionPosition(b);
    
    double weight = find_distance_between_two_points( point1, point2);
    
    return (abs(weight/35)); //divide by 35 to lower the weight- to optimize 
}


std::vector<unsigned> find_path_between_intersections(
		  const unsigned intersect_id_start, 
                  const unsigned intersect_id_end,
                  const double right_turn_penalty, 
                  const double left_turn_penalty){
    
    //std::cout << "begin find" << std::endl; 
    
    //get source node and send into findpath
    Node source = getNodeByID(intersect_id_start);
    Node *src = &source;
    //std::cout << "here" << std::endl; 
    bool found = findpath (src,intersect_id_end,right_turn_penalty,left_turn_penalty);
    //std::cout << "there" << std::endl; 
    
    if (found){

        std::vector<unsigned> zero;
        std::list<Edge> trace = traceback(intersect_id_end);
        
        for (auto it: trace){
            zero.push_back(it.id);
            //std::cout << "stuck trace" << std::endl; 
        }
        //if found set the edges to zero
        setClear();
        return zero;
       
    }
    
    else {
        
        std::vector<unsigned> zero (0);
        return zero;
    }
    //delete the source memory
    delete src;
}

void setClear(){
    
    auto count = Ngraph.size();
    for (auto i = 0; i < count; i++) {
        
        Ngraph[i].SetEdge(0);
    }
}

void NodeMap() { 
    //load up all nodes with constructor
    int total = getNumIntersections();
    for (int i = 0 ; i < total; i++){
        Node node(i, getIntersectionStreetSegmentCount(i),0,0); //set outgoing edges for each node
        Ngraph.push_back(node);
    }
}

void EdgeMap() {
    //load up all edges with constructor
    int total = getNumStreetSegments();
    for (int i = 0 ; i < total; i++){
        InfoStreetSegment seg1 = getInfoStreetSegment(i);
        int from = seg1.from;
        int to  = seg1.to;
        double b = find_street_segment_travel_time(i);
        Edge edge(i,b,from,to);
        Egraph.push_back(edge);
    }
}

Node getNodeByEdgeID(int edgeid,int nodeid){
    //search through map for node matching id we need
    Edge getEdge = Egraph[edgeid];
    Node from = getNodeByID(getEdge.from);
    if (nodeid == getEdge.from){
        Node to = getNodeByID(getEdge.to);
        return to;
    }
    
    else {
        return from;
    }
}

Node getNodeByID(int id){
    //search through map for node matching id we need
    Node getNode = Ngraph[id];
    return getNode;

}

Edge getEdgeByID(int id){
    //search through map for edge matching id we need
    Edge getEdge; 
    getEdge = Egraph[id];
    return getEdge;

}

bool check_legal(int edge, int node){
    
    if (getInfoStreetSegment(edge).oneWay){
        if (getInfoStreetSegment(edge).to == node) {
            return (false);
        }
        else{
            return(true);
        }
    }
    else {
        return (true);
    }
}

bool findpath (Node* start, int endID, const double right_turn_penalty,const double left_turn_penalty){
    
    //create a vector for nodes to delete
    std::vector<Node*>free;
    
    //create a priority queue for searching next nodes
    std::priority_queue <waveElem, std::vector<waveElem>, myComparator > pq; 
    
    pq.push(waveElem (start,NO_EDGE, 0)); // push in start node
    Ngraph[start->id].SetEdge(NO_EDGE); 
    
    int b = getNumIntersections();
    int x = getNumStreetSegments();
    
    std::vector<bool> visited(x,false);
    std::vector<double> dist (b, 10e20);//set node table with 0 for start and INF for rest
    
    dist[start->id] = 0;
    Ngraph[start->id].SetTotal(0);

    // gets first node and check if its visited or not and if cost getting there and how close it is
    //use this to create priority cost- so nodes which are better- shorter and closer search first
    while (!pq.empty()){
        
        //std::cout << "stuck here" << std::endl; 
        
        waveElem curr = pq.top(); 
        pq.pop(); 
        curr.node->reachingEdge = curr.edgeID; 
        
        if (curr.node->id == endID){
            
            for(int i; i<free.size(); i++){
                delete free[i];
            }
            return true;
        }
        
        int outgoing = curr.node->outgoing;
        int id = curr.node->id;
        
        for (int i = 0; i<outgoing; i++){
            
            int edge = getIntersectionStreetSegment(i,id);
            Node point =  getNodeByEdgeID(edge,id);
            double turntime;
            
            //get turn time weight
            if(curr.node->reachingEdge != -1){
                if (find_turn_type(curr.node->reachingEdge,edge) == TurnType::RIGHT){
                    turntime = right_turn_penalty; 
                }
                else if (find_turn_type(curr.node->reachingEdge,edge) == TurnType::LEFT){
                    turntime = left_turn_penalty; }
                else if (find_turn_type(curr.node->reachingEdge,edge) == TurnType::NONE)
                {turntime = 0;
                }
                else{
                    turntime = 0;
                }
            }
            else{
                turntime = 0;
            }
            
            int u = curr.node->id;
            int v =  point.id;
            double cost = getEdgeByID(edge).cost + turntime + Ngraph[u].total;//get cost weight
            
            if ((!(visited[edge])) && check_legal(edge,u) && (dist[v] >  cost)){
                
                Ngraph[point.id].SetEdge(edge); 
                visited[edge]=true;
                
                dist[v] =  cost;
                Ngraph[point.id].SetTotal(dist[v]);
                
                double update = dist[v] + towards(endID,v);
                Node* test = new Node(point);
                
                waveElem d = waveElem(test,edge,update);
                pq.push(d); 
                
                free.push_back(test);

            }
        }

        
    }
    //delete all pointer nodes
    for(int i; i<free.size(); i++){
        //std::cout << "free" << std::endl; 
        delete free[i];
    }
    

    return (false);
   
}

std::list<Edge> traceback (int destid){
    
    std::vector<Node*>free;
    std::list<Edge> path1; 
    
    Node* curr = new Node(getNodeByID(destid));
    free.push_back(curr);
    int prev = curr->reachingEdge;
    
    while (prev != NO_EDGE) {
        
        //std::cout << "still in loop #2" << std::endl; 
        
        path1.push_front(getEdgeByID(prev));
        curr = new Node(getNodeByEdgeID(prev,curr->id));
        prev = curr->reachingEdge;
        free.push_back(curr);

        
    }  
    
    for (int i; i < free.size(); i++) {
        //std::cout << "free #2" << std::endl; 
        delete free[i];
    }
    
    return path1;
}




        
