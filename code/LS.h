/*
 * LS.h
 *
 *  Created on: 18 Apr 2020
 *      Author: Peng
 */

#ifndef LS_H_
#define LS_H_
#include "Individual.h"
#include "read_data.h"
class LS {
public:
	// methods
	explicit LS(read_data * data);
	virtual ~LS();
	void local_search_run(Individual *s,double penaltyDepot, double penaltyVeh);
private:
	//methods
	void srun();
	void loadSolution();
//	void updateRouteInfor(int &r);
	double compute(bool istrue, double value);

	//moves
	bool Relocate_one();//  0-1 insert
	bool Relocate_two();//  2-0 insert
	bool Relocate_three();//0-2 insert
	bool Relocate_four();// 1-1 swap
	bool Relocate_five();// 2-1 swap
	bool Relocate_six();//  2-2 swap
	bool Relocate_seven();//2-2 swap R
	bool Relocate_eight();//2-opt
	bool Relocate_nine();//2-opt* R
	bool Relocate_ten();//2-opt*

	//methods for Moves
	void setLocalVariableRouteU();
	void setLocalVariableRouteV();
	void insertNode(Node *node1, Node *node2);
	void swapNode(Node *node1, Node *node2);
	//methods for split clients

	/*******************************************Split operators*/
	bool Splitb();
	bool Splitf();
	/********************************************/
	read_data* data;
	Individual* s;
	int numV;
	int numC;
	std::vector< int>orderNodes;
	Node *nodeU;
	Node *nodeX;
	Node *nodeV;
	Node *nodeY;
	Route *routeU;
	Route *routeV;
	int nodeUPrevIndex, nodeUIndex, nodeXIndex, nodeXNextIndex ;
	int nodeVPrevIndex, nodeVIndex, nodeYIndex, nodeYNextIndex ;
	int loopID;
	int nbMoves;
	double moveGain;
	bool searchComplete;

};
#endif /* LS_H_ */

