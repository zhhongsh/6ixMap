/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   class.h
 * Author: solank23
 *
 * Created on March 14, 2019, 8:58 PM
 */

#ifndef CLASS_H
#define CLASS_H
#define NO_EDGE -1 //illegal edge

#include <vector>
#include <string>
#include <functional>

class Node;
class Edge;

//holds all node and edges
extern std::vector<Node> Ngraph; 
extern std::vector<Edge> Egraph;

extern unsigned id1;
extern unsigned id2;

//hold node to look at next
struct waveElem {
    Node* node; // all the intersection nodes
    int edgeID; //street segment used to get to it
    double totalCost;
    waveElem (Node *n, int id, double cost) {
        node =  n;
        edgeID = id;
        totalCost = cost;
    }
    
    
};
//creates nodes as intersections
class Node{
    
public: 
    int id; //intersection id
    int outgoing; //all segments leaving an intersection
    int reachingEdge; //stores how it got there for later recall
    double total;
    int ParentID;
    void SetEdge(int id){
    reachingEdge = id;
    }
    void SetTotal(double id){
    total = id;
    }
    Node() {id=0;}
    Node(int a, int b, int c,double  d) {id = a; outgoing = b; reachingEdge = c; total =d;}
    
};

//creates edges as street segments
class Edge{
public:
    int id; //get segment id
    int from;
    int to;
    double cost;
    Edge(){id = 0;}
    Edge(int a,double b)//,/Node f, Node t) 
    {
        id = a; 
        cost = b;
        //from = f;
        //to = t;
    }
    Edge(int a,double b,int f, int t) 
    {
        id = a; 
        cost = b;
        from = f;
        to = t;
    }
};

//comparator for priority queue
class myComparator 
{ 
public: 
    int operator() (const waveElem& p1, const waveElem& p2) 
    { 
        return (p1.totalCost > p2.totalCost);
    } 
};


#endif /* CLASS_H */

