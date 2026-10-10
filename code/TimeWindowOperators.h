#ifndef HGASP2_TIME_WINDOW_OPERATORS_H
#define HGASP2_TIME_WINDOW_OPERATORS_H
class Individual;
class read_data;
namespace hgasp2tw {
bool construct(Individual*, read_data*, int rankPop);
bool repair(Individual*, read_data*);
void mutate(Individual*, read_data*);
void localSearch(Individual*, read_data*);
bool validate(const Individual*, const read_data*);
}
#endif
