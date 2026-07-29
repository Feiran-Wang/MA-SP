/******************************************************************************
 * File: crossover.h
 * Author: Feiran Wang
 * Date: 2026-06-24
 * Description: 
 *   Crossover operators (e.g. EAX) for breeding offspring solutions.
 ******************************************************************************/

#ifndef CROSSOVER_H_
#define CROSSOVER_H_
#include "EaxParentMode.h"
#include "Individual.h"
#include "read_data.h"
#include "popManager.h"
struct Node0{
	int cour;
	bool isFather;
};
struct Node1{
	bool isDepot;
	Node0 *edgeF;
	int nbeF;
	Node0 *edgeM;
	int nbeM;
};
struct Node2{
	Node0 *sque;
	int nbCity;
};
struct Node5{
	std::set<int >cycles;
};
struct Node6{
	int father;
	int mother;
};
class crossover {
public:
	crossover(read_data * data,  popManager * popman);
	virtual ~crossover();
	bool gEAX(Individual *fa, Individual *ma, Individual *subPop, int &numSubPop);
	static bool selectInitialEdgeSource(
		bool fatherAvailable,
		bool motherAvailable,
		hgasp::EaxParentMode mode,
		int randomBit);
	// Returns 1 when the predecessor endpoint is open and -1 when the
	// successor endpoint is open. preferredDirection may be 0, 1, or -1.
	// A fully linked node is an invalid intermediate EAX endpoint.
	static int selectOpenEndpoint(
		bool predecessorOpen,
		bool successorOpen,
		int preferredDirection);
	popManager * popman;
private:
	//
	void loadFaCapMa();
	//
	void extractABcycles();
	void addNodeToABcycles(int &nb, Node0 &tem);
	void emptyACity(int &city, int &numAvaCity);
	void exchangeInforNode0(Node0 &node1,Node0 &node2);
	void randomTheAcapB();
	//
	void generationEsets();
	//
	void generationIntermediateSolutions();
	void flipSubString(Node *nodeU, Node *nodeV,bool reverse);
	void updateTheLocation();// including elimination giant tours
	//
	void eliminateSubtours();
	void compoundSubtours(Node *begin);
	//
	void adjustSolutions();

	//
	void checkTheSolution();

	/*useful variables*/
	Individual * off;
	Individual * bestOff;
	Individual * subPop;
	read_data * data;
	int numV;
	int numL;
	int numC;
	Individual * fa;
	Individual * ma;
	int sizeSubPop;
	/*************************/
	Node1 *AcapB;          // these two variables used in STEP 1

	Node2 *ABcycles;      //these four variables used in STEP  2
	int nbABcycles;
	int *auxArray1;
	int *auxArray2;
	//variables for generating E-sets
	std::vector<std:: set <int > >whichCycles;
	std::vector<std:: set <int > >commconCycles;
	Node6 *auxArray3;
	std::set<int>auxArray4;

	Node5 *eSets;
	int nbeSets;
	int *existCity;
	//
	Node *gStart;// variables for generating intermediate solutions
	Node *gEnd;
	bool *auxA1;
	bool *auxA2;
	std::vector<Node * >headSubstring;
};

#endif /* CROSSOVER_H_ */
