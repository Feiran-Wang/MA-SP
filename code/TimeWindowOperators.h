// Author: Feiran
// Declare time-window operators and route validation.

#ifndef MASP_TIME_WINDOW_OPERATORS_H
#define MASP_TIME_WINDOW_OPERATORS_H
class Individual;
class read_data;
namespace masptw {
bool construct(Individual *individual, read_data *data, int rankPop);
bool repair(Individual *individual, read_data *data);
void mutate(Individual *individual, read_data *data);
void localSearch(Individual *individual, read_data *data);
bool validate(const Individual *individual, const read_data *data);
}
#endif
