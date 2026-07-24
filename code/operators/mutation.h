/******************************************************************************
 * File: mutation.h
 * Author: Wang
 * Date: 2026-06-24
 * Description: 
 *   Mutation operators for genetic algorithm diversification.
 ******************************************************************************/

#ifndef MUTATION_H_
#define MUTATION_H_
#include "Individual.h"
class mutation {
public:
	mutation(read_data *data);
	virtual ~mutation();
	void mutationRun(Individual *s);

private:
	Individual *s;
	void shawRemoval();
	void distanceRemove();
	double compute(bool istrue, double value);
	void insertNode(Node *U, Node *V);
	//
	void locationRemove();



	read_data *data;
	int numV;
	int numL;
	int numC;
	int *auxArray1;
	int perturbLength;
	std::vector< int >listRemove;
	int *cityRank;

};

#endif /* MUTATION_H_ */
